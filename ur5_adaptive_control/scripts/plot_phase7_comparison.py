#!/usr/bin/env python3
"""
plot_phase7_comparison.py
Comprehensive telemetry visualization for Phase 7 (Dynamic Acceleration Feedforward & Gain Scheduling):
1. Controlled Comparison Suite on 1.0 kg:
   - Variant A: G6 Baseline (Adaptive PID + Gravity FF)
   - Variant B: Adaptive PID + Gravity FF + Dynamic FF
   - Variant C: Full Phase 7 (Adaptive + Gravity + Dynamic FF + Gain Sched)
   - Matched Fixed Control: Fixed C1 PID + Gravity FF + Dynamic FF
2. Multi-Payload Validation Matrix under Full Phase 7:
   - 0.3 kg, 0.5 kg, 0.75 kg (unseen generalization), 1.0 kg, 3.0 kg
3. Commanded Torque & Dynamic Feedforward Breakdown
4. Gain Adaptation & Scheduling Profiles
"""

import sys
import json
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ARTIFACTS_DIR = Path("/home/shiva/.gemini/antigravity/brain/4d17afdd-60b6-4a5a-8b6b-96c941c8cbf6")
BASE_DIR = Path("/home/shiva/ur5/ur5_ws/experiment_logs")

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


def plot_controlled_comparison_1_0kg():
    trials = {
        "Variant A: G6 Baseline (Adaptive + Gravity FF)": {
            "dir": BASE_DIR / "test_phase6_1_0kg",
            "color": "#7f7f7f",
            "style": "--"
        },
        "Matched Fixed Control (Fixed C1 + Grav + Dyn FF)": {
            "dir": BASE_DIR / "test_phase7_1_0kg_fixed_dyn",
            "color": "#1f77b4",
            "style": "-."
        },
        "Variant B: Adaptive + Grav + Dynamic FF": {
            "dir": BASE_DIR / "test_phase7_1_0kg_dyn",
            "color": "#ff7f0e",
            "style": "-"
        },
        "Variant C: Full Phase 7 (+ Gain Scheduling)": {
            "dir": BASE_DIR / "test_phase7_1_0kg_full",
            "color": "#2ca02c",
            "style": "-"
        },
    }

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 11), gridspec_kw={'height_ratios': [1.3, 1.0]})

    metrics_summary = []

    for label, info in trials.items():
        csv_file = info["dir"] / "joint_trajectory_log.csv"
        res_file = info["dir"] / "result.json"
        if not csv_file.is_file() or not res_file.is_file():
            print(f"Skipping {label}: files missing in {info['dir']}")
            continue

        with open(res_file) as f:
            rdata = json.load(f)
        met = rdata.get("metrics", {})
        metrics_summary.append({
            "label": label,
            "max_ee": met.get("ee_max_mm", 0.0),
            "rmse_ee": met.get("ee_rmse_mm", 0.0),
            "settled_ee": met.get("ee_final_settled_mm", 0.0),
            "color": info["color"]
        })

        df = pd.read_csv(csv_file)
        m = get_motion_data(df)
        ax1.plot(m['rel_time'].to_numpy(), m['ee_error_3d_mm'].to_numpy(),
                 label=f"{label} (Max: {met.get('ee_max_mm', 0.0):.3f} mm, RMSE: {met.get('ee_rmse_mm', 0.0):.3f} mm)",
                 color=info["color"], linestyle=info["style"], linewidth=2.0)

    ax1.set_title("Phase 7 Controlled Comparison on 1.0 kg: 3D End-Effector Tracking Error", fontsize=14, fontweight='bold', pad=12)
    ax1.set_xlabel("Relative Motion Time (s)", fontsize=11)
    ax1.set_ylabel("3D EE Tracking Error (mm)", fontsize=11)
    ax1.grid(True, linestyle=":", alpha=0.6)
    ax1.legend(loc="upper right", framealpha=0.95, fontsize=10)
    ax1.axhline(1.0, color='red', linestyle=':', linewidth=1.5, label='< 1.0 mm Accuracy Goal')

    # Subplot 2: Grouped Bar Chart of Error Reduction
    if metrics_summary:
        names = [m["label"].split(":")[0] if ":" in m["label"] else m["label"] for m in metrics_summary]
        max_vals = [m["max_ee"] for m in metrics_summary]
        rmse_vals = [m["rmse_ee"] for m in metrics_summary]
        settle_vals = [m["settled_ee"] for m in metrics_summary]

        x = np.arange(len(names))
        width = 0.25

        rects1 = ax2.bar(x - width, max_vals, width, label='Peak EE Error (mm)', color='#d62728', alpha=0.85)
        rects2 = ax2.bar(x, rmse_vals, width, label='RMSE EE Error (mm)', color='#1f77b4', alpha=0.85)
        rects3 = ax2.bar(x + width, settle_vals, width, label='Settled EE Error (mm)', color='#2ca02c', alpha=0.85)

        ax2.set_title("Controlled Metrics Comparison: Error Reduction Breakdown", fontsize=13, fontweight='bold', pad=10)
        ax2.set_xticks(x)
        ax2.set_xticklabels(names, fontsize=10)
        ax2.set_ylabel("Error (mm)", fontsize=11)
        ax2.grid(True, axis='y', linestyle=':', alpha=0.6)
        ax2.legend(loc="upper right", fontsize=10)
        ax2.axhline(1.0, color='red', linestyle=':', linewidth=1.2)

        def autolabel(rects):
            for rect in rects:
                height = rect.get_height()
                ax2.annotate(f'{height:.3f}',
                            xy=(rect.get_x() + rect.get_width() / 2, height),
                            xytext=(0, 3), textcoords="offset points",
                            ha='center', va='bottom', fontsize=9, fontweight='semibold')

        autolabel(rects1)
        autolabel(rects2)
        autolabel(rects3)

    plt.tight_layout()
    out_path = ARTIFACTS_DIR / "ee_tracking_error_phase7_comparison.png"
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"Saved: {out_path}")


def plot_multipreload_scaling():
    masses = [0.3, 0.5, 0.75, 1.0, 3.0]
    palette = {
        0.3: "#2ca02c",
        0.5: "#1f77b4",
        0.75: "#9467bd",  # Purple for unseen generalization
        1.0: "#ff7f0e",
        3.0: "#d62728",
    }
    dir_map = {
        0.3: BASE_DIR / "test_phase7_0_3kg",
        0.5: BASE_DIR / "test_phase7_0_5kg",
        0.75: BASE_DIR / "test_phase7_0_75kg",
        1.0: BASE_DIR / "test_phase7_1_0kg_full",
        3.0: BASE_DIR / "test_phase7_3_0kg",
    }

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 11), gridspec_kw={'height_ratios': [1.3, 1.0]})

    mass_points = []
    max_err_points = []
    rmse_err_points = []

    for m in masses:
        tdir = dir_map[m]
        csv_file = tdir / "joint_trajectory_log.csv"
        res_file = tdir / "result.json"
        if not csv_file.is_file() or not res_file.is_file():
            print(f"Skipping mass {m} kg: files missing in {tdir}")
            continue

        with open(res_file) as f:
            rdata = json.load(f)
        met = rdata.get("metrics", {})
        max_err = met.get("ee_max_mm", 0.0)
        rmse_err = met.get("ee_rmse_mm", 0.0)

        mass_points.append(m)
        max_err_points.append(max_err)
        rmse_err_points.append(rmse_err)

        df = pd.read_csv(csv_file)
        df_m = get_motion_data(df)
        lbl = f"{m:.2f} kg {'(Unseen)' if m==0.75 else ''} (Max: {max_err:.3f} mm, RMSE: {rmse_err:.3f} mm)"
        ax1.plot(df_m['rel_time'].to_numpy(), df_m['ee_error_3d_mm'].to_numpy(),
                 label=lbl, color=palette[m], linewidth=2.0)

    ax1.set_title("Phase 7 Full System Multi-Payload 3D EE Tracking Error Profiles", fontsize=14, fontweight='bold', pad=12)
    ax1.set_xlabel("Relative Motion Time (s)", fontsize=11)
    ax1.set_ylabel("3D EE Tracking Error (mm)", fontsize=11)
    ax1.grid(True, linestyle=":", alpha=0.6)
    ax1.legend(loc="upper right", framealpha=0.95, fontsize=10)
    ax1.axhline(1.0, color='red', linestyle=':', linewidth=1.5, label='< 1.0 mm Accuracy Goal')

    # Subplot 2: Error Scaling vs Payload Mass
    if mass_points:
        ax2.plot(mass_points, max_err_points, 'o-', color='#d62728', linewidth=2.0, markersize=8, label='Peak EE Error (mm)')
        ax2.plot(mass_points, rmse_err_points, 's-', color='#1f77b4', linewidth=2.0, markersize=8, label='RMSE EE Error (mm)')
        for i, m in enumerate(mass_points):
            ax2.annotate(f"{max_err_points[i]:.3f} mm", (m, max_err_points[i]),
                         textcoords="offset points", xytext=(0, 8), ha='center', fontsize=9, fontweight='semibold')
            ax2.annotate(f"{rmse_err_points[i]:.3f} mm", (m, rmse_err_points[i]),
                         textcoords="offset points", xytext=(0, -14), ha='center', fontsize=9)

        ax2.set_title("Phase 7 Error Scaling vs Active Payload Mass", fontsize=13, fontweight='bold', pad=10)
        ax2.set_xlabel("Payload Mass (kg)", fontsize=11)
        ax2.set_ylabel("Tracking Error (mm)", fontsize=11)
        ax2.grid(True, linestyle=":", alpha=0.6)
        ax2.set_xticks(masses)
        ax2.axhline(1.0, color='red', linestyle=':', linewidth=1.2, label='1.0 mm Target')
        ax2.legend(loc="upper left", fontsize=10)

    plt.tight_layout()
    out_path = ARTIFACTS_DIR / "ee_tracking_error_phase7_scaling.png"
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"Saved: {out_path}")


def plot_torque_breakdown():
    fig, axes = plt.subplots(3, 2, figsize=(16, 12), sharex=True)
    axes = axes.flatten()

    dir_g6 = BASE_DIR / "test_phase6_1_0kg"
    dir_p7 = BASE_DIR / "test_phase7_1_0kg_full"

    if not (dir_g6 / "joint_trajectory_log.csv").is_file() or not (dir_p7 / "joint_trajectory_log.csv").is_file():
        print("Skipping torque breakdown: missing logs")
        return

    df_g6 = get_motion_data(pd.read_csv(dir_g6 / "joint_trajectory_log.csv"))
    df_p7 = get_motion_data(pd.read_csv(dir_p7 / "joint_trajectory_log.csv"))

    for idx, (joint, label) in enumerate(zip(JOINTS, JOINT_LABELS)):
        ax = axes[idx]
        col_cmd = f"{joint}__commanded_effort_Nm"
        if col_cmd in df_g6.columns:
            ax.plot(df_g6['rel_time'].to_numpy(), df_g6[col_cmd].to_numpy(), label="Phase 6 (Grav FF only)", color="#7f7f7f", linestyle="--", alpha=0.8)
        if col_cmd in df_p7.columns:
            ax.plot(df_p7['rel_time'].to_numpy(), df_p7[col_cmd].to_numpy(), label="Phase 7 (Grav + Dyn FF + Sched)", color="#2ca02c", linewidth=1.8)

        ax.set_title(label, fontsize=12, fontweight='bold')
        ax.set_ylabel("Torque (Nm)", fontsize=10)
        ax.grid(True, linestyle=":", alpha=0.6)
        if idx == 0:
            ax.legend(loc="upper right", fontsize=9)
        if idx >= 4:
            ax.set_xlabel("Relative Motion Time (s)", fontsize=10)

    plt.suptitle("Phase 7 Actuator Torque Comparison: Dynamic Feedforward Action (1.0 kg)", fontsize=14, fontweight='bold', y=0.995)
    plt.tight_layout()
    out_path = ARTIFACTS_DIR / "torque_breakdown_phase7.png"
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"Saved: {out_path}")


def plot_gain_adaptation():
    fig, axes = plt.subplots(2, 2, figsize=(15, 10), sharex=True)

    masses = [0.3, 0.5, 0.75, 1.0, 3.0]
    palette = {
        0.3: "#2ca02c",
        0.5: "#1f77b4",
        0.75: "#9467bd",
        1.0: "#ff7f0e",
        3.0: "#d62728",
    }
    dir_map = {
        0.3: BASE_DIR / "test_phase7_0_3kg",
        0.5: BASE_DIR / "test_phase7_0_5kg",
        0.75: BASE_DIR / "test_phase7_0_75kg",
        1.0: BASE_DIR / "test_phase7_1_0kg_full",
        3.0: BASE_DIR / "test_phase7_3_0kg",
    }

    key_joints = ["shoulder_pan_joint", "shoulder_lift_joint"]

    for i, joint in enumerate(key_joints):
        ax_kp = axes[i, 0]
        ax_kd = axes[i, 1]

        for mass in masses:
            log_file = dir_map[mass] / "gain_adaptation_log.csv"
            if not log_file.is_file():
                continue
            df = pd.read_csv(log_file)
            j_df = df[df['joint'] == joint].copy()
            j_df['rel_time'] = j_df['time_sec'] - j_df['time_sec'].min()
            j_df_sub = j_df.iloc[::5]

            t_arr = j_df_sub['rel_time'].to_numpy()
            kp_arr = j_df_sub['kp'].to_numpy()
            kd_arr = j_df_sub['kd'].to_numpy()

            lbl = f"{mass:.2f} kg {'(Unseen)' if mass==0.75 else ''}"
            ax_kp.plot(t_arr, kp_arr, label=lbl, color=palette[mass], linewidth=1.5)
            ax_kd.plot(t_arr, kd_arr, label=lbl, color=palette[mass], linewidth=1.5)

        ax_kp.set_title(f"{joint} - Proportional Gain Kp", fontsize=12, fontweight="bold")
        ax_kp.set_ylabel("Kp (Nm/rad)", fontsize=10)
        ax_kp.grid(True, linestyle="--", alpha=0.5)
        ax_kp.legend(loc="upper right", fontsize=9)

        ax_kd.set_title(f"{joint} - Derivative Gain Kd", fontsize=12, fontweight="bold")
        ax_kd.set_ylabel("Kd (Nm·s/rad)", fontsize=10)
        ax_kd.grid(True, linestyle="--", alpha=0.5)
        ax_kd.legend(loc="upper right", fontsize=9)

    axes[1, 0].set_xlabel("Relative Time (s)", fontsize=10)
    axes[1, 1].set_xlabel("Relative Time (s)", fontsize=10)

    fig.suptitle("Phase 7 Adaptive & Scheduled Gain Trajectories Across Physical Payloads", fontsize=14, fontweight="bold", y=0.99)
    plt.tight_layout()
    out_path = ARTIFACTS_DIR / "gain_adaptation_phase7_profiles.png"
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"Saved: {out_path}")


def main():
    plot_controlled_comparison_1_0kg()
    plot_multipreload_scaling()
    plot_torque_breakdown()
    plot_gain_adaptation()


if __name__ == "__main__":
    main()
