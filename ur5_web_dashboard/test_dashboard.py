"""Verification suite for UR5 Web Dashboard components, kinematics, data loading, and metrics."""

import math
from pathlib import Path
import sys
import unittest

WS_ROOT = Path(__file__).resolve().parent.parent
if str(WS_ROOT) not in sys.path:
    sys.path.insert(0, str(WS_ROOT))

import numpy as np
import pandas as pd

from ur5_web_dashboard.config import JOINTS
from ur5_web_dashboard.data_manager import DataManager
from ur5_web_dashboard.fk_engine import VerifiedFK
from ur5_web_dashboard.ros_monitor import ROSMonitor


class TestUR5Dashboard(unittest.TestCase):
    def setUp(self):
        self.ws_root = Path(__file__).resolve().parent.parent
        self.data_manager = DataManager(self.ws_root)

    def test_historical_runs_discovery(self):
        runs = self.data_manager.list_available_runs()
        self.assertGreater(len(runs), 0, "Should discover at least one historical run in experiment_logs")
        first_run = runs[0]
        self.assertIn("path", first_run)
        self.assertTrue(Path(first_run["path"]).exists())

    def test_csv_loading_and_metrics(self):
        runs = self.data_manager.list_available_runs()
        self.assertGreater(len(runs), 0)
        run_data = self.data_manager.load_run(runs[0]["path"])
        df = run_data["df"]
        metrics = run_data["metrics"]

        # Check required columns
        for j in JOINTS:
            self.assertIn(f"{j}__reference_position_rad", df.columns)
            self.assertIn(f"{j}__actual_position_rad", df.columns)
            self.assertIn(f"{j}__position_error_rad", df.columns)

        # Check calculated metrics
        for j in JOINTS:
            jm = metrics["joints"][j]
            self.assertGreater(jm["sample_count"], 0)
            self.assertTrue(math.isfinite(jm["rmse_rad"]))
            self.assertTrue(math.isfinite(jm["max_err_rad"]))
            self.assertTrue(math.isfinite(jm["iae_rad_s"]))
            self.assertTrue(math.isfinite(jm["ise_rad2_s"]))
            self.assertGreaterEqual(jm["max_err_rad"], 0.0)

        # Verify RMSE formula against direct numpy calculation
        err_j1 = df[f"{JOINTS[0]}__position_error_rad"].dropna().to_numpy()
        expected_rmse = float(np.sqrt(np.mean(err_j1 ** 2)))
        self.assertAlmostEqual(metrics["joints"][JOINTS[0]]["rmse_rad"], expected_rmse, places=6)

    def test_ros_monitor_initialization(self):
        monitor = ROSMonitor(buffer_size=100)
        snapshot = monitor.get_snapshot()
        self.assertEqual(snapshot["diagnostics"]["buffer_count"], 0)
        self.assertEqual(snapshot["diagnostics"]["buffer_capacity"], 100)
        self.assertFalse(snapshot["diagnostics"]["effort_available"])

    def test_all_tabs_rendering(self):
        from ur5_web_dashboard.app import create_app
        app = create_app(buffer_size=50)
        runs = self.data_manager.list_available_runs()
        self.assertGreater(len(runs), 0)
        run_path = runs[0]["path"]

        # Directly invoke underlying callback function for all tabs
        for callback_id, cb_dict in app.callback_map.items():
            func = cb_dict["callback"]
            raw_func = getattr(func, "__wrapped__", func)
            # Test update_main_content callback
            if "kpi-summary-grid.children" in callback_id:
                for tab_id in ["tab-tracking", "tab-velocities", "tab-cartesian", "tab-metrics", "tab-pid", "tab-settings"]:
                    kpi, content = raw_func(0, tab_id, "historical", run_path, "none", JOINTS)
                    self.assertIsNotNone(kpi)
                    self.assertIsNotNone(content)

        app.ros_monitor.stop()


if __name__ == "__main__":
    unittest.main()
