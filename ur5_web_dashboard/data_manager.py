"""Historical CSV data loader, validator, metric engine, and run comparison utility."""

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import pandas as pd

from .config import ANNOTATIONS_DIR, JOINTS


class DataManager:
    """Manages historical trajectory CSV logs, validation, and analytics."""

    def __init__(self, workspace_root: Optional[Path] = None):
        if workspace_root is None:
            workspace_root = Path(__file__).resolve().parent.parent
        self.workspace_root = workspace_root
        self.logs_root = self.workspace_root / "experiment_logs"

        # Cached datasets to prevent redundant disk I/O
        self._cache: Dict[str, Dict[str, Any]] = {}

    def list_available_runs(self) -> List[Dict[str, Any]]:
        """Scan experiment_logs directory for valid run directories."""
        runs = []
        if not self.logs_root.exists():
            return runs

        for path in sorted(self.logs_root.iterdir(), reverse=True):
            if path.is_dir():
                csv_file = path / "joint_trajectory_log.csv"
                if csv_file.exists():
                    info_file = path / "experiment_info.json"
                    info_data = {}
                    if info_file.exists():
                        try:
                            with open(info_file, "r", encoding="utf-8") as f:
                                info_data = json.load(f)
                        except Exception:
                            pass

                    runs.append({
                        "id": path.name,
                        "path": str(csv_file),
                        "dir": str(path),
                        "trajectory_id": info_data.get("trajectory_id", path.name),
                        "controller_mode": info_data.get("controller_mode", "baseline_position"),
                        "payload_kg": info_data.get("payload_kg", "N/A"),
                        "created_utc": info_data.get("configuration_provenance", {}).get("created_utc", ""),
                        "ee_available": info_data.get("ee_tracking", {}).get("available", False),
                    })
        return runs

    def load_run(self, csv_path_str: str) -> Dict[str, Any]:
        """Safely load and analyze CSV data in read-only mode without altering the original file."""
        csv_path = Path(csv_path_str).resolve()
        cache_key = f"{csv_path}:{csv_path.stat().st_mtime}"
        if cache_key in self._cache:
            return self._cache[cache_key]

        if not csv_path.exists():
            raise FileNotFoundError(f"Trajectory log not found: {csv_path}")

        # Read CSV with pandas in read-only mode
        df = pd.read_csv(csv_path)

        # Check required columns
        req_cols = ["time_sec"]
        for j in JOINTS:
            req_cols.extend([f"{j}__reference_position_rad", f"{j}__actual_position_rad"])
        missing = [c for c in req_cols if c not in df.columns]
        if missing:
            raise ValueError(f"CSV missing mandatory columns: {missing[:4]}")

        # Ensure error column exists or compute reference minus actual
        for j in JOINTS:
            err_col = f"{j}__position_error_rad"
            ref_col = f"{j}__reference_position_rad"
            act_col = f"{j}__actual_position_rad"
            if err_col not in df.columns:
                df[err_col] = df[ref_col] - df[act_col]

        # Load metadata if present in parent directory
        info_path = csv_path.parent / "experiment_info.json"
        info = {}
        if info_path.exists():
            try:
                with open(info_path, "r", encoding="utf-8") as f:
                    info = json.load(f)
            except Exception:
                pass

        # Compute comprehensive performance metrics
        metrics = self.compute_performance_metrics(df)

        # User notes / annotations
        notes = self.load_annotation(csv_path.parent.name)

        result = {
            "df": df,
            "metrics": metrics,
            "info": info,
            "notes": notes,
            "run_id": csv_path.parent.name,
            "csv_path": str(csv_path),
            "row_count": len(df),
            "duration_sec": metrics["cycle"]["logged_duration_sec"],
        }
        self._cache[cache_key] = result
        return result

    @staticmethod
    def compute_performance_metrics(df: pd.DataFrame) -> Dict[str, Any]:
        """Compute RMSE, Max Error, IAE, ISE, and cycle durations."""
        time_series = df["time_sec"].to_numpy()
        valid_time_mask = np.isfinite(time_series)
        valid_times = time_series[valid_time_mask]

        if len(valid_times) > 1:
            total_duration = float(valid_times[-1] - valid_times[0])
            dt = np.diff(time_series)
            # Replace non-positive or excessive dt with 0
            dt = np.where(dt > 0.0, dt, 0.0)
        else:
            total_duration = 0.0
            dt = np.array([])

        joint_metrics = {}
        moving_samples = 0
        active_motion_duration = 0.0

        # Activity threshold: 0.01 rad/s (~0.57 deg/s) across any joint
        vel_cols = [f"{j}__actual_velocity_rad_s" for j in JOINTS if f"{j}__actual_velocity_rad_s" in df.columns]
        if vel_cols:
            max_vel = df[vel_cols].abs().max(axis=1).to_numpy()
            moving_mask = max_vel >= 0.01
            moving_samples = int(np.sum(moving_mask))
            if len(dt) > 0:
                active_motion_duration = float(np.sum(dt[moving_mask[:-1]]))

        worst_rmse = 0.0
        worst_max_err = 0.0
        worst_iae = 0.0

        for j in JOINTS:
            err_col = f"{j}__position_error_rad"
            errors = df[err_col].to_numpy()
            finite_mask = np.isfinite(errors)
            finite_err = errors[finite_mask]

            if len(finite_err) > 0:
                rmse = float(np.sqrt(np.mean(finite_err ** 2)))
                max_abs_err = float(np.max(np.abs(finite_err)))

                # Trapezoidal integration for IAE and ISE
                if len(dt) > 0:
                    e_abs = np.abs(errors)
                    e_sq = errors ** 2
                    iae = float(np.sum(0.5 * (e_abs[:-1] + e_abs[1:]) * dt))
                    ise = float(np.sum(0.5 * (e_sq[:-1] + e_sq[1:]) * dt))
                else:
                    iae = 0.0
                    ise = 0.0

                max_idx = int(np.argmax(np.abs(errors)))
                peak_time = float(time_series[max_idx]) if max_idx < len(time_series) else 0.0

                joint_metrics[j] = {
                    "rmse_rad": rmse,
                    "rmse_deg": float(np.degrees(rmse)),
                    "max_err_rad": max_abs_err,
                    "max_err_deg": float(np.degrees(max_abs_err)),
                    "iae_rad_s": iae,
                    "ise_rad2_s": ise,
                    "sample_count": len(finite_err),
                    "peak_time_sec": peak_time,
                }

                if rmse > worst_rmse:
                    worst_rmse = rmse
                if max_abs_err > worst_max_err:
                    worst_max_err = max_abs_err
                if iae > worst_iae:
                    worst_iae = iae
            else:
                joint_metrics[j] = {
                    "rmse_rad": float("nan"),
                    "rmse_deg": float("nan"),
                    "max_err_rad": float("nan"),
                    "max_err_deg": float("nan"),
                    "iae_rad_s": float("nan"),
                    "ise_rad2_s": float("nan"),
                    "sample_count": 0,
                    "peak_time_sec": float("nan"),
                }

        # End-Effector 3D metrics if present
        ee_metrics = {"available": False}
        if "ee_error_3d_mm" in df.columns:
            ee_errors = df["ee_error_3d_mm"].to_numpy()
            ee_finite = ee_errors[np.isfinite(ee_errors)]
            if len(ee_finite) > 0:
                ee_metrics = {
                    "available": True,
                    "max_3d_mm": float(np.max(ee_finite)),
                    "mean_3d_mm": float(np.mean(ee_finite)),
                    "rmse_3d_mm": float(np.sqrt(np.mean(ee_finite ** 2))),
                    "sample_count": len(ee_finite),
                }

        return {
            "joints": joint_metrics,
            "ee": ee_metrics,
            "cycle": {
                "logged_duration_sec": total_duration,
                "active_duration_sec": active_motion_duration,
                "total_samples": len(df),
                "moving_samples": moving_samples,
            },
            "summary": {
                "worst_rmse_rad": worst_rmse,
                "worst_rmse_deg": float(np.degrees(worst_rmse)),
                "worst_max_err_rad": worst_max_err,
                "worst_max_err_deg": float(np.degrees(worst_max_err)),
                "worst_iae_rad_s": worst_iae,
            },
        }

    @staticmethod
    def load_annotation(run_id: str) -> Dict[str, Any]:
        """Load user notes for a specific run without touching the original log file."""
        note_file = ANNOTATIONS_DIR / f"{run_id}_notes.json"
        if note_file.exists():
            try:
                with open(note_file, "r", encoding="utf-8") as f:
                    return json.load(f)
            except Exception:
                pass
        return {"notes": "", "tags": [], "operator": "", "verified": False}

    @staticmethod
    def save_annotation(run_id: str, data: Dict[str, Any]) -> bool:
        """Persist notes and review flags in a separate annotations file."""
        note_file = ANNOTATIONS_DIR / f"{run_id}_notes.json"
        try:
            with open(note_file, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            return True
        except Exception:
            return False

