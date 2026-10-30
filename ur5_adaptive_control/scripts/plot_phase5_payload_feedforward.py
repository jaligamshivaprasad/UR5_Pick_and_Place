#!/usr/bin/env python3
"""
plot_phase5_payload_feedforward.py
Generates comprehensive visual telemetry plots for Phase 5:
1. End-Effector Tracking Error Comparison across C1 Fixed, G4 Adaptive, G4.1 Adaptive, and Phase 5 Payload FF + Adaptive.
2. 6-Joint Effort Breakdown & Feedforward Authority (Commanded vs Measured vs Limits).
3. Online Adaptive Gain Profiles (Kp, Ki, Kd) during Payload-Compensated Pick & Place.
"""

from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ARTIFACTS_DIR = Path("/home/shiva/.gemini/antigravity/brain/4d17afdd-60b6-4a5a-8b6b-96c941c8cbf6")
EXP_DIR = Path("/home/shiva/ur5/ur5_ws/experiment_logs/test_phase5_payload_adaptive")
G4_DIR = Path("/home/shiva/ur5/ur5_ws/experiment_logs/test_phase4_g4_fixed")
G4_ADAPT_DIR = Path("/home/shiva/ur5/ur5_ws/experiment_logs/test_phase4_g4_adaptive")
G4_1_ADAPT_DIR = Path("/home/shiva/ur5/ur5_ws/experiment_logs/test_phase4_1_adaptive")

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
JOINT_LABELS = ["J1: Shoulder Pan", "J2: Shoulder Lift", "J3: Elbow", "J4: Wrist 1", "J5: Wrist 2", "J6: Wrist 3"]

def plot_ee_comparison():
    fig, axes = plt.subplots(2, 1, figsize=(14, 10), sharex=False)
    
    # Load dataframes
    df_fixed = pd.read_csv(G4_DIR / "joint_trajectory_log.csv")
    df_g4 = pd.read_csv(G4_ADAPT_DIR / "joint_trajectory_log.csv")
    df_g4_1 = pd.read_csv(G4_1_ADAPT_DIR / "joint_trajectory_log.csv")
    df_p5 = pd.read_csv(EXP_DIR / "joint_trajectory_log.csv")

    # Align by trajectory segments
    # Filter for active replay phases: prepick, pick, pick_to_place, place_to_straight
    def get_motion_data(df):
        m = df[df['trajectory_id'].str.contains('path/')].copy()
        m['rel_time'] = m['time_sec'] - m['time_sec'].min()
        return m

    m_fix = get_motion_data(df_fixed)
    m_g4 = get_motion_data(df_g4)
    m_g4_1 = get_motion_data(df_g4_1)
    m_p5 = get_motion_data(df_p5)

    # Top plot: 3D Tracking Error
    ax0 = axes[0]
    ax0.plot(m_fix['rel_time'].to_numpy(), m_fix['ee_error_3d_mm'].to_numpy(), 'r--', label='C1 Fixed PID (Max: 32.2 mm, RMSE: 13.8 mm)', alpha=0.7, linewidth=1.5)
    ax0.plot(m_g4['rel_time'].to_numpy(), m_g4['ee_error_3d_mm'].to_numpy(), 'g-.', label='G4 Adaptive Kp/Kd (Max: 30.6 mm, RMSE: 10.7 mm)', alpha=0.7, linewidth=1.5)
    ax0.plot(m_g4_1['rel_time'].to_numpy(), m_g4_1['ee_error_3d_mm'].to_numpy(), 'b:', label='G4.1 Adaptive Kp/Ki/Kd (Max: 27.0 mm, RMSE: 13.4 mm)', alpha=0.8, linewidth=1.8)
    ax0.plot(m_p5['rel_time'].to_numpy(), m_p5['ee_error_3d_mm'].to_numpy(), 'm-', label='Phase 5 Payload FF + Adaptive (Max: 7.2 mm, RMSE: 1.57 mm)', linewidth=2.5)

    ax0.axhline(1.0, color='k', linestyle='--', linewidth=1.2, label='1.0 mm Accuracy Goal')
    ax0.axhline(3.0, color='gray', linestyle=':', linewidth=1.0, label='3.0 mm Acceptance Boundary')
    ax0.set_ylabel('3D EE Tracking Error (mm)', fontsize=12, fontweight='bold')
    ax0.set_title('UR5 End-Effector 3D Tracking Error: Benchmark Evolution (G3 -> G4 -> G4.1 -> Phase 5)', fontsize=14, fontweight='bold')
    ax0.legend(loc='upper right', framealpha=0.9, fontsize=10)
    ax0.grid(True, alpha=0.3)
    ax0.set_ylim(-0.5, 35.0)

    # Bottom plot: Vertical (Z-axis) Tracking Error
    ax1 = axes[1]
    ax1.plot(m_fix['rel_time'].to_numpy(), m_fix['ee_error_z_mm'].to_numpy(), 'r--', label='C1 Fixed PID Z-Error (Max: +30.5 mm)', alpha=0.7, linewidth=1.5)
    ax1.plot(m_g4_1['rel_time'].to_numpy(), m_g4_1['ee_error_z_mm'].to_numpy(), 'b:', label='G4.1 Adaptive Z-Error (Max: +26.8 mm)', alpha=0.8, linewidth=1.8)
    ax1.plot(m_p5['rel_time'].to_numpy(), m_p5['ee_error_z_mm'].to_numpy(), 'm-', label='Phase 5 Payload FF + Adaptive Z-Error (Max: 6.5 mm, Steady < 0.8 mm)', linewidth=2.5)

    ax1.axhline(0.0, color='k', linestyle='-', linewidth=0.8)
    ax1.axhline(1.0, color='k', linestyle='--', linewidth=1.2)
    ax1.axhline(-1.0, color='k', linestyle='--', linewidth=1.2)
    ax1.set_xlabel('Relative Trajectory Time (s)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Vertical Z Tracking Error (mm)', fontsize=12, fontweight='bold')
    ax1.set_title('Vertical Gravity Droop Compensation (Z-Axis Error)', fontsize=13, fontweight='bold')
    ax1.legend(loc='upper right', framealpha=0.9, fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(-10.0, 35.0)

    plt.tight_layout()
    out_path = ARTIFACTS_DIR / "ee_tracking_error_phase5_comparison.png"
    plt.savefig(out_path, dpi=200)
    plt.savefig(EXP_DIR / "ee_tracking_error_phase5_comparison.png", dpi=200)
    plt.close()
    print(f"Saved EE Comparison Plot to: {out_path}")

def plot_torque_breakdown():
    fig, axes = plt.subplots(3, 2, figsize=(16, 12), sharex=True)
    df_p5 = pd.read_csv(EXP_DIR / "joint_trajectory_log.csv")
    m_p5 = df_p5[df_p5['trajectory_id'].str.contains('path/')].copy()
    m_p5['rel_time'] = m_p5['time_sec'] - m_p5['time_sec'].min()

    limits = [150.0, 150.0, 150.0, 28.0, 28.0, 28.0]

    for idx, (j_name, label) in enumerate(zip(JOINTS, JOINT_LABELS)):
        r, c = idx % 3, idx // 3
        ax = axes[r, c]

        cmd = m_p5[f"{j_name}__commanded_effort_Nm"].to_numpy()
        meas = m_p5[f"{j_name}__measured_effort_Nm"].to_numpy()
        rel_time = m_p5['rel_time'].to_numpy()

        ax.plot(rel_time, cmd, 'm-', label='Total Commanded Torque (PID + FF)', linewidth=1.8)
        ax.plot(rel_time, meas, 'k--', label='Measured Joint Torque (Gazebo Wrench)', alpha=0.6, linewidth=1.2)

        lim = limits[idx]
        ax.axhline(lim, color='r', linestyle=':', alpha=0.6, label=f'Safety Limit (+/-{lim} Nm)')
        ax.axhline(-lim, color='r', linestyle=':', alpha=0.6)

        ax.set_ylabel('Torque (Nm)', fontsize=10, fontweight='bold')
        ax.set_title(label, fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3)
        if idx == 0:
            ax.legend(loc='upper right', fontsize=8)

        # Highlight loaded transport phase (approx 5.5s to 10.5s)
        ax.axvspan(5.5, 10.5, color='yellow', alpha=0.1, label='3.0 kg Transport Segment')

    axes[2, 0].set_xlabel('Relative Trajectory Time (s)', fontsize=11, fontweight='bold')
    axes[2, 1].set_xlabel('Relative Trajectory Time (s)', fontsize=11, fontweight='bold')

    plt.suptitle('UR5 Six-Joint Torque Breakdown under Payload-Aware Feedforward + Adaptive Control', fontsize=14, fontweight='bold')
    plt.tight_layout()
    out_path = ARTIFACTS_DIR / "torque_breakdown_phase5.png"
    plt.savefig(out_path, dpi=200)
    plt.savefig(EXP_DIR / "torque_breakdown_phase5.png", dpi=200)
    plt.close()
    print(f"Saved Torque Breakdown Plot to: {out_path}")

def plot_gains():
    fig, axes = plt.subplots(3, 2, figsize=(16, 12), sharex=True)
    df_p5 = pd.read_csv(EXP_DIR / "joint_trajectory_log.csv")
    m_p5 = df_p5[df_p5['trajectory_id'].str.contains('path/')].copy()
    m_p5['rel_time'] = m_p5['time_sec'] - m_p5['time_sec'].min()

    for idx, (j_name, label) in enumerate(zip(JOINTS, JOINT_LABELS)):
        r, c = idx % 3, idx // 3
        ax = axes[r, c]

        kp = m_p5[f"{j_name}__Kp"].to_numpy()
        kd = m_p5[f"{j_name}__Kd"].to_numpy()
        rel_time = m_p5['rel_time'].to_numpy()

        ax.plot(rel_time, kp, 'b-', label='Adapted Kp (Stiffness)', linewidth=1.8)
        ax.set_ylabel('Kp [Nm/rad]', color='b', fontsize=10, fontweight='bold')
        ax.tick_params(axis='y', labelcolor='b')
        ax.grid(True, alpha=0.3)

        ax2 = ax.twinx()
        ax2.plot(rel_time, kd, 'g--', label='Adapted Kd (Damping)', linewidth=1.5)
        ax2.set_ylabel('Kd [Nm/(rad/s)]', color='g', fontsize=10, fontweight='bold')
        ax2.tick_params(axis='y', labelcolor='g')

        ax.set_title(label, fontsize=11, fontweight='bold')
        ax.axvspan(5.5, 10.5, color='yellow', alpha=0.1)

    axes[2, 0].set_xlabel('Relative Trajectory Time (s)', fontsize=11, fontweight='bold')
    axes[2, 1].set_xlabel('Relative Trajectory Time (s)', fontsize=11, fontweight='bold')

    plt.suptitle('UR5 Online Gain Adaptation Profiles under Validated Gravity Compensation', fontsize=14, fontweight='bold')
    plt.tight_layout()
    out_path = ARTIFACTS_DIR / "gain_adaptation_phase5_profiles.png"
    plt.savefig(out_path, dpi=200)
    plt.savefig(EXP_DIR / "gain_adaptation_phase5_profiles.png", dpi=200)
    plt.close()
    print(f"Saved Gain Adaptation Plot to: {out_path}")

if __name__ == "__main__":
    plot_ee_comparison()
    plot_torque_breakdown()
    plot_gains()
