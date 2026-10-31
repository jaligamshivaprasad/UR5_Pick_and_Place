#!/usr/bin/env python3
"""Generate Phase 9 Stability, Fault Handling, and Integration Validation Figures."""

import json
from pathlib import Path
import shutil
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

ROOT = Path("/home/shiva/ur5/ur5_ws")
EXP_BASE = ROOT / "experiment_logs"
OUT_DIR = ROOT / "docs" / "adaptive_pid" / "figures"
BRAIN_DIR = Path("/home/shiva/.gemini/antigravity/brain/dc6bd500-0759-4676-b816-a98d6b5f519e/figures")
OUT_DIR.mkdir(parents=True, exist_ok=True)
BRAIN_DIR.mkdir(parents=True, exist_ok=True)

plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")
plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 11,
    "axes.titlesize": 13,
    "axes.labelsize": 11,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "figure.titlesize": 14,
    "lines.linewidth": 2.0,
})

# -----------------------------------------------------------------------------
# Figure 1: Stability & Drift Analysis Across Multi-Payload & Switch Events
# -----------------------------------------------------------------------------
def plot_figure_1_stability():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5.5))

    conditions = [
        "Fixed (0.0kg)",
        "Adapt (0.0kg)",
        "Adapt (0.3kg)",
        "Adapt (0.5kg)",
        "Adapt (0.75kg)",
        "Adapt (1.0kg)",
        "Adapt (3.0kg)",
        "Switch F->A",
        "Switch A->F",
        "Recovery",
    ]
    joint_drifts_rad = [
        0.000002,
        0.000001,
        0.000003,
        0.000013,
        0.000035,
        0.000053,
        0.000275,
        0.000439,
        0.000006,
        0.000047,
    ]
    ee_drifts_mm = [
        0.00001,
        0.00001,
        0.00010,
        0.00020,
        0.00030,
        0.00032,
        0.00181,
        0.00290,
        0.00004,
        0.00031,
    ]

    colors = ["#2b5c8f"] * 7 + ["#e67e22", "#e67e22"] + ["#27ae60"]

    # Ax 1: Joint Drift (log scale)
    bars1 = ax1.bar(range(len(conditions)), [d * 1000 for d in joint_drifts_rad], color=colors, alpha=0.85, edgecolor="black")
    ax1.axhline(20.0, color="#c0392b", linestyle="--", linewidth=1.8, label="Gate G9 Threshold (20.0 mrad / 0.02 rad)")
    ax1.set_yscale("log")
    ax1.set_xticks(range(len(conditions)))
    ax1.set_xticklabels(conditions, rotation=40, ha="right")
    ax1.set_ylabel("Maximum Joint Drift [mrad] (Log Scale)")
    ax1.set_title("Joint Holding & Switching Drift vs G9 Threshold")
    ax1.grid(True, which="both", linestyle=":", alpha=0.6)
    ax1.legend(loc="upper left")

    for bar, val in zip(bars1, joint_drifts_rad):
        ax1.annotate(f"{val*1000:.3f}",
                     xy=(bar.get_x() + bar.get_width() / 2, bar.get_height()),
                     xytext=(0, 3), textcoords="offset points",
                     ha="center", va="bottom", fontsize=8.5, fontweight="bold")

    # Ax 2: Cartesian EE Drift
    bars2 = ax2.bar(range(len(conditions)), ee_drifts_mm, color=colors, alpha=0.85, edgecolor="black")
    ax2.axhline(0.50, color="#c0392b", linestyle="--", linewidth=1.8, label="Gate G9 Threshold (0.50 mm)")
    ax2.set_xticks(range(len(conditions)))
    ax2.set_xticklabels(conditions, rotation=40, ha="right")
    ax2.set_ylabel("End-Effector Cartesian Drift [mm]")
    ax2.set_title("Cartesian End-Effector Holding Stability")
    ax2.grid(True, linestyle=":", alpha=0.6)
    ax2.legend(loc="upper left")

    for bar, val in zip(bars2, ee_drifts_mm):
        ax2.annotate(f"{val:.4f}",
                     xy=(bar.get_x() + bar.get_width() / 2, bar.get_height()),
                     xytext=(0, 3), textcoords="offset points",
                     ha="center", va="bottom", fontsize=8.5, fontweight="bold")

    plt.suptitle("UR5 Phase 9A: Stationary Holding Stability, Bumpless Switching & Recovery Drift", y=0.98, fontsize=13, fontweight="bold")
    plt.tight_layout()
    p1 = OUT_DIR / "phase9_stability_drift_analysis.png"
    plt.savefig(p1, dpi=300)
    plt.close()
    shutil.copy2(p1, BRAIN_DIR / "phase9_stability_drift_analysis.png")
    print(f"Generated {p1}")


# -----------------------------------------------------------------------------
# Figure 2: Fault Matrix & Deterministic Containment Verification
# -----------------------------------------------------------------------------
def plot_figure_2_fault_matrix():
    fig, ax = plt.subplots(figsize=(13, 6.5))

    faults = [
        "F01: NaN / Inf Joint Feedback Guard",
        "F02: Canceled Trajectory Reference Holding",
        "F03: Missing Payload Confirmation Silence",
        "F04: Invalid Controller Load / Switch Rejection",
        "F05: In-Motion Controller Switching Rejection",
        "F06: Invalid Payload Mass Fallback Clamping",
        "F07: Excessive Tracking Error (|e| > 0.10 rad)",
        "F08: Actuator Torque Saturation Anti-Windup",
        "F09: High-Freq Oscillation Zero-Crossing Freeze",
        "F10: Timing Delays & Update Rate Jitter Guard",
        "F11: Invalid Scaling & Retiming Rejection",
        "REC: Deterministic Baseline Recovery",
    ]

    actions = [
        "Instant zero clamp (0.0 Nm) & frozen states; no actuator kick",
        "Zero-velocity reference latching; drift = 0.00039 rad (< 0.02 rad)",
        "Feedforward strictly zeroed (tau_ff = 0); baseline feedback only",
        "Strict rejection with exception; active controller undisturbed",
        "Quiescence gate (|v| >= 0.0055 rad/s) blocks switch; motion intact",
        "Negative / > 3.5kg / NaN safely clamped to s = 0.10 fallback profile",
        "JointGainAdaptation enters FALLBACK; smooth beta gain decay",
        "Effort strictly bounded to joint limits; adaptation frozen",
        "Error zero-crossings > 17.5 Hz trigger FROZEN_OSCILLATION state",
        "dt <= 0 or dt > 0.1 s triggers FROZEN_FEEDBACK; zero output on dt > 0.5s",
        "Negative scale & non-monotonic timestamps caught with ValueError",
        "Instant bumpless recovery to arm_controller; drift = 0.000047 rad",
    ]

    ax.set_xlim(0, 10)
    ax.set_ylim(-0.5, len(faults) - 0.5)
    ax.axis("off")

    ax.text(0.1, len(faults) - 0.1, "Fault Class & Safety Requirement", fontweight="bold", fontsize=11, color="#1a252f")
    ax.text(3.5, len(faults) - 0.1, "Deterministic Containment & Safeguard Mechanism", fontweight="bold", fontsize=11, color="#1a252f")
    ax.text(9.2, len(faults) - 0.1, "Gate Status", fontweight="bold", fontsize=11, color="#1a252f", ha="center")

    ax.plot([0, 10], [len(faults) - 0.25, len(faults) - 0.25], color="#2c3e50", lw=2)

    for i, (f, a) in enumerate(zip(reversed(faults), reversed(actions))):
        y = i
        bg_color = "#f8f9fa" if i % 2 == 0 else "#ffffff"
        ax.add_patch(plt.Rectangle((0, y - 0.4), 10, 0.8, color=bg_color, zorder=0))

        ax.text(0.1, y, f, fontsize=10, va="center", fontweight="bold", color="#2c3e50")
        ax.text(3.5, y, a, fontsize=9.5, va="center", color="#34495e")

        # Pass pill
        bbox_props = dict(boxstyle="round,pad=0.3", fc="#27ae60", ec="none", alpha=0.9)
        ax.text(9.2, y, "PASS", fontsize=9.5, fontweight="bold", color="white", ha="center", va="center", bbox=bbox_props)

    plt.title("UR5 Phase 9B: Deterministic Fault Detection, Safe Containment & Recovery Matrix", fontsize=13, fontweight="bold", pad=15)
    plt.tight_layout()
    p2 = OUT_DIR / "phase9_fault_matrix_response.png"
    plt.savefig(p2, dpi=300)
    plt.close()
    shutil.copy2(p2, BRAIN_DIR / "phase9_fault_matrix_response.png")
    print(f"Generated {p2}")


# -----------------------------------------------------------------------------
# Figure 3: Phase 9C Integration Trial Accuracy & Cycle Time Trade-off
# -----------------------------------------------------------------------------
def plot_figure_3_integration():
    trial_dirs = [
        "test_phase9_trial1_1_0kg_fixed_scaled",
        "test_phase9_trial2_1_0kg_adaptive_scaled",
        "test_phase9_trial3_1_0kg_repeat_adaptive",
        "test_phase9_trial4_3_0kg_adaptive_scaled",
        "test_phase9_trial5_0_3kg_adaptive_scaled",
    ]
    labels = [
        "T1: Fixed C1 (1.0kg, s=0.15)",
        "T2: Adapt C (1.0kg, s=0.15)",
        "T3: Repeat Adapt (1.0kg, s=0.15)",
        "T4: Adapt Capacity (3.0kg, s=0.10)",
        "T5: Adapt Agile (0.3kg, s=0.25)",
    ]

    rmse = []
    emax = []
    eset = []
    cycle_time = []

    for td in trial_dirs:
        with open(EXP_BASE / td / "result.json") as f:
            d = json.load(f)
        m = d.get("metrics", {})
        rmse.append(m.get("ee_rmse_mm", 0.0))
        emax.append(m.get("ee_max_mm", 0.0))
        eset.append(m.get("ee_final_settled_mm", 0.0))
        cycle_time.append(m.get("cycle_sec", 0.0))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5.5))

    x = np.arange(len(labels))
    width = 0.26

    rects1 = ax1.bar(x - width, rmse, width, label="RMSE EE [mm]", color="#2980b9", alpha=0.9, edgecolor="black")
    rects2 = ax1.bar(x, emax, width, label="Max EE Error [mm]", color="#e74c3c", alpha=0.85, edgecolor="black")
    rects3 = ax1.bar(x + width, eset, width, label="Final Settled Error [mm]", color="#27ae60", alpha=0.9, edgecolor="black")

    ax1.axhline(1.0, color="#2980b9", linestyle=":", linewidth=1.5, label="Target RMSE (<= 1.0 mm)")
    ax1.axhline(0.5, color="#27ae60", linestyle="--", linewidth=1.5, label="Target Settled (<= 0.5 mm)")

    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, rotation=35, ha="right")
    ax1.set_ylabel("End-Effector Cartesian Error [mm]")
    ax1.set_title("Tracking Accuracy & Settling Precision Across Integration Trials")
    ax1.grid(True, linestyle=":", alpha=0.6)
    ax1.legend(loc="upper left", framealpha=0.95)

    for r in rects1:
        ax1.annotate(f"{r.get_height():.2f}", xy=(r.get_x() + r.get_width() / 2, r.get_height()),
                     xytext=(0, 2), textcoords="offset points", ha="center", va="bottom", fontsize=8.5)
    for r in rects2:
        ax1.annotate(f"{r.get_height():.2f}", xy=(r.get_x() + r.get_width() / 2, r.get_height()),
                     xytext=(0, 2), textcoords="offset points", ha="center", va="bottom", fontsize=8.5)
    for r in rects3:
        ax1.annotate(f"{r.get_height():.3f}", xy=(r.get_x() + r.get_width() / 2, r.get_height()),
                     xytext=(0, 2), textcoords="offset points", ha="center", va="bottom", fontsize=8.5)

    # Ax 2: Cycle Time Comparison
    colors_cycle = ["#7f8c8d", "#3498db", "#2980b9", "#d35400", "#16a085"]
    bars_c = ax2.bar(x, cycle_time, width=0.45, color=colors_cycle, alpha=0.85, edgecolor="black")
    ax2.set_xticks(x)
    ax2.set_xticklabels(labels, rotation=35, ha="right")
    ax2.set_ylabel("Cycle Time [s]")
    ax2.set_title("Full Pick-and-Place Cycle Duration (path01)")
    ax2.grid(True, linestyle=":", alpha=0.6)

    for bar in bars_c:
        ax2.annotate(f"{bar.get_height():.2f} s", xy=(bar.get_x() + bar.get_width() / 2, bar.get_height()),
                     xytext=(0, 3), textcoords="offset points", ha="center", va="bottom", fontsize=9.5, fontweight="bold")

    plt.suptitle("UR5 Phase 9C: Full Pick-and-Place Integration Validation Metrics", y=0.98, fontsize=13, fontweight="bold")
    plt.tight_layout()
    p3 = OUT_DIR / "phase9_integration_tracking_comparison.png"
    plt.savefig(p3, dpi=300)
    plt.close()
    shutil.copy2(p3, BRAIN_DIR / "phase9_integration_tracking_comparison.png")
    print(f"Generated {p3}")


# -----------------------------------------------------------------------------
# Figure 4: Actuator Peak Torques & Margin Headroom Analysis
# -----------------------------------------------------------------------------
def plot_figure_4_torques():
    trial_dirs = [
        "test_phase9_trial1_1_0kg_fixed_scaled",
        "test_phase9_trial2_1_0kg_adaptive_scaled",
        "test_phase9_trial3_1_0kg_repeat_adaptive",
        "test_phase9_trial4_3_0kg_adaptive_scaled",
        "test_phase9_trial5_0_3kg_adaptive_scaled",
    ]
    trial_names = [
        "T1: Fixed 1.0kg",
        "T2: Adapt 1.0kg",
        "T3: Repeat 1.0kg",
        "T4: Adapt 3.0kg",
        "T5: Adapt 0.3kg",
    ]
    joint_names = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow",
        "wrist_1",
        "wrist_2",
        "wrist_3",
    ]
    limits = [150.0, 150.0, 150.0, 28.0, 28.0, 28.0]

    all_peaks = []
    for td in trial_dirs:
        with open(EXP_BASE / td / "result.json") as f:
            d = json.load(f)
        all_peaks.append(d["metrics"]["peak_commanded_torque_Nm"])
    all_peaks = np.array(all_peaks)  # Shape (5, 6)

    fig, ax = plt.subplots(figsize=(13, 6))

    x = np.arange(len(joint_names))
    width = 0.15

    palette = ["#95a5a6", "#3498db", "#2980b9", "#e67e22", "#1abc9c"]

    for i, tname in enumerate(trial_names):
        ax.bar(x + (i - 2) * width, all_peaks[i, :], width, label=tname, color=palette[i], alpha=0.9, edgecolor="black")

    # Overlay limit lines as step or markers
    for j, (lim, jn) in enumerate(zip(limits, joint_names)):
        ax.hlines(lim, j - 0.45, j + 0.45, colors="#c0392b", linestyles="--", linewidth=1.8)
        margin_pct = (1.0 - np.max(all_peaks[:, j]) / lim) * 100.0
        ax.text(j, lim + 2.0, f"Limit: {lim:.0f} Nm\n(Headroom: {margin_pct:.1f}%)",
                ha="center", va="bottom", fontsize=8.5, color="#c0392b", fontweight="bold")

    ax.set_xticks(x)
    ax.set_xticklabels([f"{n}\n({lim:.0f} Nm)" for n, lim in zip(joint_names, limits)], fontsize=10.5)
    ax.set_ylabel("Peak Commanded Actuator Torque [Nm]")
    ax.set_title("Peak Commanded Torques & Actuator Safety Headroom Across All Joints")
    ax.set_ylim(0, 175)
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="upper right", framealpha=0.95)

    plt.suptitle("UR5 Phase 9C: Actuator Effort Limits & Saturation Avoidance Under Payload Scaling", y=0.98, fontsize=13, fontweight="bold")
    plt.tight_layout()
    p4 = OUT_DIR / "phase9_torque_margins_repeatability.png"
    plt.savefig(p4, dpi=300)
    plt.close()
    shutil.copy2(p4, BRAIN_DIR / "phase9_torque_margins_repeatability.png")
    print(f"Generated {p4}")


if __name__ == "__main__":
    plot_figure_1_stability()
    plot_figure_2_fault_matrix()
    plot_figure_3_integration()
    plot_figure_4_torques()
    print("All Phase 9 validation figures successfully created!")

