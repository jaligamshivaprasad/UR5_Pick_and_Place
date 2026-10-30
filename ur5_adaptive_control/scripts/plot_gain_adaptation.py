#!/usr/bin/env python3
"""
plot_gain_adaptation.py
Generates comprehensive analysis plots for Phase 4 / Gate G4:
1. Joint gains Kp(t) and Kd(t) vs time for all 6 joints with bounds.
2. Composite error s(t) and supervisor adaptation state.
3. Comparative End-Effector Error Profile (Fixed C1 vs Adaptive Disabled vs Adaptive Enabled).
"""

import sys
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def generate_plots(log_csv: Path, output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)
    df = pd.read_csv(log_csv)

    # Relative time from start
    t0 = df["time_sec"].min()
    df["t_rel"] = df["time_sec"] - t0

    joints = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    joint_labels = ["J1: Shoulder Pan", "J2: Shoulder Lift", "J3: Elbow", "J4: Wrist 1", "J5: Wrist 2", "J6: Wrist 3"]

    # -------------------------------------------------------------
    # 1. Gains Kp(t) and Kd(t) vs Time for All 6 Joints
    # -------------------------------------------------------------
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    axes = axes.flatten()

    for idx, (joint, label) in enumerate(zip(joints, joint_labels)):
        ax = axes[idx]
        j_df = df[df["joint"] == joint].sort_values("t_rel")

        kp0 = j_df["kp"].iloc[0]
        kd0 = j_df["kd"].iloc[0]

        t_rel = j_df["t_rel"].to_numpy()
        kp = j_df["kp"].to_numpy()
        kd = j_df["kd"].to_numpy()

        ax.plot(t_rel, kp, "b-", linewidth=1.5, label=f"$K_p(t)$")
        ax.axhline(kp0, color="blue", linestyle=":", alpha=0.6, label=f"$K_p^0={kp0:.1f}$")
        ax.axhline(kp0 * 1.35, color="red", linestyle="--", alpha=0.5, label="$K_p$ max (+35%)")
        ax.axhline(kp0 * 0.75, color="red", linestyle="--", alpha=0.5, label="$K_p$ min (-25%)")

        ax2 = ax.twinx()
        ax2.plot(t_rel, kd, "g-", linewidth=1.2, label=f"$K_d(t)$")
        ax2.axhline(kd0, color="green", linestyle=":", alpha=0.6, label=f"$K_d^0={kd0:.2f}$")
        ax2.set_ylabel("$K_d$ [Nms/rad]", color="green")
        ax2.tick_params(axis="y", labelcolor="green")

        ax.set_title(f"{label}", fontsize=11, fontweight="bold")
        ax.set_ylabel("$K_p$ [Nm/rad]", color="blue")
        ax.tick_params(axis="y", labelcolor="blue")
        ax.grid(True, alpha=0.3)

        if idx == 0:
            lines1, labels1 = ax.get_legend_handles_labels()
            lines2, labels2 = ax2.get_legend_handles_labels()
            ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8)

    axes[-1].set_xlabel("Time [s]", fontsize=10)
    axes[-2].set_xlabel("Time [s]", fontsize=10)
    plt.suptitle("UR5 Online PID Gain Adaptation (Kp, Kd vs Time) - Gate G4", fontsize=14, fontweight="bold")
    plt.tight_layout()
    gain_plot_path = output_dir / "gain_adaptation_profiles.png"
    plt.savefig(gain_plot_path, dpi=200)
    plt.close()
    print(f"Saved: {gain_plot_path}")

    # -------------------------------------------------------------
    # 2. Tracking Error & Commanded Effort
    # -------------------------------------------------------------
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    axes = axes.flatten()

    for idx, (joint, label) in enumerate(zip(joints, joint_labels)):
        ax = axes[idx]
        j_df = df[df["joint"] == joint].sort_values("t_rel")

        t_rel = j_df["t_rel"].to_numpy()
        pos_err = (j_df["pos_error"] * 1000.0).to_numpy()
        effort = j_df["effort"].to_numpy()

        ax.plot(t_rel, pos_err, "r-", linewidth=1.2, label="Pos Error [mrad]")
        ax.set_ylabel("Error [mrad]", color="red")
        ax.tick_params(axis="y", labelcolor="red")
        ax.grid(True, alpha=0.3)

        ax2 = ax.twinx()
        ax2.plot(t_rel, effort, "k-", linewidth=1.0, alpha=0.7, label="Commanded Torque [Nm]")
        ax2.set_ylabel("Torque [Nm]", color="black")
        ax2.tick_params(axis="y", labelcolor="black")

        ax.set_title(f"{label}", fontsize=11, fontweight="bold")
        if idx == 0:
            lines1, labels1 = ax.get_legend_handles_labels()
            lines2, labels2 = ax2.get_legend_handles_labels()
            ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8)

    axes[-1].set_xlabel("Time [s]", fontsize=10)
    axes[-2].set_xlabel("Time [s]", fontsize=10)
    plt.suptitle("UR5 Tracking Error and Commanded Joint Torque - Gate G4", fontsize=14, fontweight="bold")
    plt.tight_layout()
    error_plot_path = output_dir / "tracking_error_and_torque.png"
    plt.savefig(error_plot_path, dpi=200)
    plt.close()
    print(f"Saved: {error_plot_path}")

def plot_comparison(fixed_csv: Path, disabled_csv: Path, adaptive_csv: Path, output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)
    df_fix = pd.read_csv(fixed_csv)
    df_dis = pd.read_csv(disabled_csv)
    df_ada = pd.read_csv(adaptive_csv)

    # Filter for trajectory execution phase (after align/settle)
    for d in [df_fix, df_dis, df_ada]:
        if "t_rel" not in d.columns:
            d["t_rel"] = d["time_sec"] - d["time_sec"].min()

    fig, ax = plt.subplots(figsize=(12, 6))

    # Calculate EE error norm if Cartesian cols present
    # In joint_trajectory_log.csv: ee_x_error, ee_y_error, ee_z_error
    for name, d, color, ls in [
        ("Fixed C1 Baseline (arm_controller)", df_fix, "gray", "--"),
        ("Adaptive Controller (Disabled Mode)", df_dis, "blue", "-."),
        ("Adaptive Controller (Enabled Online)", df_ada, "red", "-"),
    ]:
        if "ee_x_error" in d.columns:
            ee_err_mm = np.sqrt(d["ee_x_error"].to_numpy()**2 + d["ee_y_error"].to_numpy()**2 + d["ee_z_error"].to_numpy()**2) * 1000.0
            ax.plot(d["t_rel"].to_numpy(), ee_err_mm, label=name, color=color, linestyle=ls, linewidth=1.5)

    ax.set_title("Cartesian End-Effector Tracking Error Comparison (path01)", fontsize=13, fontweight="bold")
    ax.set_xlabel("Elapsed Time [s]", fontsize=11)
    ax.set_ylabel("EE Tracking Error [mm]", fontsize=11)
    ax.axhline(3.0, color="black", linestyle=":", alpha=0.7, label="Target Bound (3.0 mm)")
    ax.legend(loc="upper right", fontsize=10)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    comp_plot_path = output_dir / "ee_tracking_error_comparison.png"
    plt.savefig(comp_plot_path, dpi=200)
    plt.close()
    print(f"Saved: {comp_plot_path}")

if __name__ == "__main__":
    base_dir = Path("/home/shiva/ur5/ur5_ws/experiment_logs")
    adaptive_dir = base_dir / "test_phase4_g4_adaptive"
    fixed_dir = base_dir / "test_phase4_g4_fixed"
    disabled_dir = base_dir / "test_phase4_g4_disabled"

    if (adaptive_dir / "gain_adaptation_log.csv").exists():
        generate_plots(adaptive_dir / "gain_adaptation_log.csv", adaptive_dir)

    if (fixed_dir / "joint_trajectory_log.csv").exists() and (disabled_dir / "joint_trajectory_log.csv").exists() and (adaptive_dir / "joint_trajectory_log.csv").exists():
        plot_comparison(
            fixed_dir / "joint_trajectory_log.csv",
            disabled_dir / "joint_trajectory_log.csv",
            adaptive_dir / "joint_trajectory_log.csv",
            adaptive_dir
        )
