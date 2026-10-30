#!/usr/bin/env python3
"""
plot_phase6_payload_variation.py
Generates comprehensive visual telemetry plots for Phase 6 Payload Variation Validation:
1. End-Effector Tracking Error Comparison across 0.3 kg, 0.5 kg, 1.0 kg, and 3.0 kg payloads.
   - 3D tracking error trajectory profiles over path01.
   - Error scaling metrics (Max Error and RMSE) vs Payload Mass.
2. 6-Joint Commanded Torque Profiles across Payloads.
3. Online Adaptive Gain Profiles (Kp and Kd) across Payloads.
"""

from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ARTIFACTS_DIR = Path("/home/shiva/.gemini/antigravity/brain/4d17afdd-60b6-4a5a-8b6b-96c941c8cbf6")
BASE_DIR = Path("/home/shiva/ur5/ur5_ws/experiment_logs")

EXP_DIRS = {
    0.3: BASE_DIR / "test_phase6_0_3kg",
    0.5: BASE_DIR / "test_phase6_0_5kg",
    1.0: BASE_DIR / "test_phase6_1_0kg",
    3.0: BASE_DIR / "test_phase5_payload_adaptive",
}

COLORS = {
    0.3: "#2ca02c",  # Green
    0.5: "#1f77b4",  # Blue
    1.0: "#ff7f0e",  # Orange
    3.0: "#d62728",  # Red
}

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
JOINT_LABELS = ["J1: Shoulder Pan", "J2: Shoulder Lift", "J3: Elbow", "J4: Wrist 1", "J5: Wrist 2", "J6: Wrist 3"]


def get_motion_data(df):
    m = df[df['trajectory_id'].str.contains('path/')].copy()
    m['rel_time'] = m['time_sec'] - m['time_sec'].min()
    return m


def plot_ee_comparison():
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 11), gridspec_kw={'height_ratios': [1.3, 1.0]})

    masses = [0.3, 0.5, 1.0, 3.0]
    max_errors = []
    rmse_errors = []
    settled_errors = []

    for mass in masses:
        df = pd.read_csv(EXP_DIRS[mass] / "joint_trajectory_log.csv")
        m = get_motion_data(df)
        t_arr = m['rel_time'].to_numpy()
        ee_err_mm = m['ee_error_3d_mm'].to_numpy()

        ax1.plot(
            t_arr,
            ee_err_mm,
            label=f"Payload {mass:.1f} kg (Max: {np.max(ee_err_mm):.2f} mm, RMSE: {np.sqrt(np.mean(ee_err_mm**2)):.2f} mm)",
            color=COLORS[mass],
            linewidth=2.0 if mass == 3.0 else 1.8,
            alpha=0.9,
        )

        max_errors.append(float(np.max(ee_err_mm)))
        rmse_errors.append(float(np.sqrt(np.mean(ee_err_mm**2))))
        
        # Settled error from final 10 samples
        settled_errors.append(float(np.mean(ee_err_mm[-10:])))

    ax1.set_title("Cartesian End-Effector Tracking Error Across Physical Payload Variations (path01 Replay)", fontsize=14, fontweight="bold", pad=12)
    ax1.set_xlabel("Trajectory Relative Time (s)", fontsize=12)
    ax1.set_ylabel("EE 3D Tracking Error (mm)", fontsize=12)
    ax1.grid(True, linestyle="--", alpha=0.6)
    ax1.axhline(1.0, color="green", linestyle=":", linewidth=1.5, label="1.0 mm Accuracy Threshold")
    ax1.axhline(3.0, color="gray", linestyle="--", linewidth=1.2, label="3.0 mm Pass Limit")
    ax1.legend(loc="upper left", fontsize=11, framealpha=0.95)

    # Annotate trajectory segments
    ax1.axvspan(0.0, 3.2, color="gray", alpha=0.08)
    ax1.text(1.6, 6.2, "prepick / pick", ha="center", fontsize=10, style="italic")
    ax1.axvspan(3.2, 11.8, color="blue", alpha=0.05)
    ax1.text(7.5, 6.2, "LOADED TRANSPORT (pick_to_place)", ha="center", fontsize=10, fontweight="bold", color="#1f449b")
    ax1.axvspan(11.8, 16.4, color="gray", alpha=0.08)
    ax1.text(14.1, 6.2, "place / return", ha="center", fontsize=10, style="italic")

    # Subplot 2: Scaling of tracking error metrics vs payload mass
    ax2.plot(masses, max_errors, 'o-', color="#d62728", linewidth=2.5, markersize=8, label="Max EE Tracking Error (mm)")
    ax2.plot(masses, rmse_errors, 's-', color="#1f77b4", linewidth=2.5, markersize=8, label="EE Tracking RMSE (mm)")
    ax2.plot(masses, settled_errors, '^--', color="#2ca02c", linewidth=2.0, markersize=8, label="Settled Placement Error (mm)")

    for m_val, max_e, rmse_e, set_e in zip(masses, max_errors, rmse_errors, settled_errors):
        ax2.annotate(f"{max_e:.2f} mm", (m_val, max_e), textcoords="offset points", xytext=(0, 10), ha="center", fontweight="bold", color="#d62728")
        ax2.annotate(f"{rmse_e:.2f} mm", (m_val, rmse_e), textcoords="offset points", xytext=(0, 10), ha="center", fontweight="bold", color="#1f77b4")
        ax2.annotate(f"{set_e*1000:.0f} µm", (m_val, set_e), textcoords="offset points", xytext=(0, -15), ha="center", color="#2ca02c")

    ax2.axhline(1.0, color="green", linestyle=":", linewidth=1.5, label="1.0 mm Sub-Millimeter Threshold")
    ax2.set_title("Tracking Error Scaling vs Physical Payload Mass", fontsize=13, fontweight="bold", pad=10)
    ax2.set_xlabel("Payload Mass (kg)", fontsize=12)
    ax2.set_ylabel("Error (mm)", fontsize=12)
    ax2.set_xticks(masses)
    ax2.grid(True, linestyle="--", alpha=0.6)
    ax2.legend(loc="upper left", fontsize=11, framealpha=0.95)

    plt.tight_layout()
    out_path = ARTIFACTS_DIR / "ee_tracking_error_phase6_comparison.png"
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"Saved: {out_path}")


def plot_torque_breakdown():
    fig, axes = plt.subplots(3, 2, figsize=(16, 12), sharex=True)
    axes = axes.flatten()

    masses = [0.3, 0.5, 1.0, 3.0]

    for j_idx, joint in enumerate(JOINTS):
        ax = axes[j_idx]
        col = f"{joint}__commanded_effort_Nm"
        for mass in masses:
            df = pd.read_csv(EXP_DIRS[mass] / "joint_trajectory_log.csv")
            m = get_motion_data(df)
            t_arr = m['rel_time'].to_numpy()
            tau_arr = m[col].to_numpy()
            ax.plot(
                t_arr,
                tau_arr,
                label=f"{mass:.1f} kg (Peak: {np.max(np.abs(tau_arr)):.1f} Nm)",
                color=COLORS[mass],
                linewidth=1.6 if mass == 3.0 else 1.3,
            )

        ax.set_title(JOINT_LABELS[j_idx], fontsize=12, fontweight="bold")
        ax.set_ylabel("Torque (Nm)", fontsize=10)
        ax.grid(True, linestyle="--", alpha=0.5)
        ax.legend(loc="upper right", fontsize=9, framealpha=0.9)
        if j_idx >= 4:
            ax.set_xlabel("Trajectory Relative Time (s)", fontsize=10)

    fig.suptitle("Commanded Joint Torque Profiles Across Physical Payload Variations (path01)", fontsize=15, fontweight="bold", y=0.995)
    plt.tight_layout()
    out_path = ARTIFACTS_DIR / "torque_breakdown_phase6.png"
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"Saved: {out_path}")


def plot_gain_adaptation():
    fig, axes = plt.subplots(2, 2, figsize=(15, 10), sharex=True)

    masses = [0.3, 0.5, 1.0, 3.0]
    key_joints = ["shoulder_pan_joint", "shoulder_lift_joint"]

    for i, joint in enumerate(key_joints):
        ax_kp = axes[i, 0]
        ax_kd = axes[i, 1]

        for mass in masses:
            df = pd.read_csv(EXP_DIRS[mass] / "gain_adaptation_log.csv")
            j_df = df[df['joint'] == joint].copy()
            # Find trajectory start
            j_df['rel_time'] = j_df['time_sec'] - j_df['time_sec'].min()
            
            # Subsample for smooth plotting
            j_df_sub = j_df.iloc[::5]

            t_arr = j_df_sub['rel_time'].to_numpy()
            kp_arr = j_df_sub['kp'].to_numpy()
            kd_arr = j_df_sub['kd'].to_numpy()

            ax_kp.plot(t_arr, kp_arr, label=f"{mass:.1f} kg", color=COLORS[mass], linewidth=1.5)
            ax_kd.plot(t_arr, kd_arr, label=f"{mass:.1f} kg", color=COLORS[mass], linewidth=1.5)

        ax_kp.set_title(f"{joint} - Proportional Gain Kp", fontsize=12, fontweight="bold")
        ax_kp.set_ylabel("Kp", fontsize=10)
        ax_kp.grid(True, linestyle="--", alpha=0.5)
        ax_kp.legend(loc="upper right", fontsize=9)

        ax_kd.set_title(f"{joint} - Derivative Gain Kd", fontsize=12, fontweight="bold")
        ax_kd.set_ylabel("Kd", fontsize=10)
        ax_kd.grid(True, linestyle="--", alpha=0.5)
        ax_kd.legend(loc="upper right", fontsize=9)

    axes[1, 0].set_xlabel("Time (s)", fontsize=10)
    axes[1, 1].set_xlabel("Time (s)", fontsize=10)

    fig.suptitle("Adaptive Gain Trajectories (Kp, Kd) Across Physical Payload Variations", fontsize=14, fontweight="bold", y=0.99)
    plt.tight_layout()
    out_path = ARTIFACTS_DIR / "gain_adaptation_phase6_profiles.png"
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    plot_ee_comparison()
    plot_torque_breakdown()
    plot_gain_adaptation()
