#!/usr/bin/env python3
"""Generate comparative metrics, plots, and performance verification.

Compares baseline effort seed against optimized verified fixed-PID trials.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
]
JOINT_LABELS = ['J1 (Pan)', 'J2 (Lift)', 'J3 (Elbow)', 'J4 (Wrist 1)', 'J5 (Wrist 2)', 'J6 (Wrist 3)']
LIMITS = [150., 150., 150., 28., 28., 28.]


def load_csv(csv_path):
    with open(csv_path, 'r', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    return rows


def compute_metrics(rows):
    path_rows = [r for r in rows if r['trajectory_id'].startswith('path/')]
    hold_rows = [r for r in rows if r['trajectory_id'] == 'hold']
    settle_rows = [r for r in rows if r['trajectory_id'] == 'settle']

    def calc_ee_stats(row_set):
        vals = [float(r['ee_error_3d_mm']) for r in row_set if r['ee_error_3d_mm'] != 'nan']
        if not vals:
            return 0.0, 0.0
        arr = np.array(vals)
        return float(np.sqrt(np.mean(arr**2))), float(np.max(arr))

    all_ee_rmse, all_ee_max = calc_ee_stats(rows)
    path_ee_rmse, path_ee_max = calc_ee_stats(path_rows)
    hold_ee_rmse, hold_ee_max = calc_ee_stats(hold_rows)
    settle_ee_rmse, settle_ee_max = calc_ee_stats(settle_rows)
    final_settled_err = float(rows[-1]['ee_error_3d_mm']) if rows and rows[-1]['ee_error_3d_mm'] != 'nan' else 0.0

    joint_stats = {}
    for idx, j in enumerate(JOINTS):
        path_err = np.array([float(r[j + '__position_error_rad']) for r in path_rows])
        all_err = np.array([float(r[j + '__position_error_rad']) for r in rows])
        torques = np.array([float(r[j + '__commanded_effort_Nm']) for r in path_rows if r[j + '__commanded_effort_Nm'] != 'nan'])
        peak_torque = float(np.max(np.abs(torques))) if len(torques) > 0 else 0.0
        margin_pct = (1.0 - peak_torque / LIMITS[idx]) * 100.0

        joint_stats[j] = {
            'path_rmse_rad': float(np.sqrt(np.mean(path_err**2))),
            'path_rmse_deg': float(np.sqrt(np.mean(path_err**2)) * 180.0 / np.pi),
            'path_max_deg': float(np.max(np.abs(path_err)) * 180.0 / np.pi),
            'all_rmse_deg': float(np.sqrt(np.mean(all_err**2)) * 180.0 / np.pi),
            'peak_torque_Nm': peak_torque,
            'limit_Nm': LIMITS[idx],
            'margin_pct': margin_pct
        }

    return {
        'all_ee_rmse_mm': all_ee_rmse,
        'all_ee_max_mm': all_ee_max,
        'path_ee_rmse_mm': path_ee_rmse,
        'path_ee_max_mm': path_ee_max,
        'hold_ee_rmse_mm': hold_ee_rmse,
        'final_settled_mm': final_settled_err,
        'joints': joint_stats
    }


def plot_comparisons(base_rows, opt_rows, out_dir):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    base_path = [r for r in base_rows if r['trajectory_id'].startswith('path/')]
    opt_path = [r for r in opt_rows if r['trajectory_id'].startswith('path/')]

    t_base = np.array([float(r['time_sec']) for r in base_path]) - float(base_path[0]['time_sec'])
    t_opt = np.array([float(r['time_sec']) for r in opt_path]) - float(opt_path[0]['time_sec'])

    # 1. EE 3D Error and Z Profile Overlay
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    ee_base = [float(r['ee_error_3d_mm']) for r in base_path]
    ee_opt = [float(r['ee_error_3d_mm']) for r in opt_path]
    ax1.plot(t_base, ee_base, 'r--', label='Baseline Seed PID', alpha=0.85, linewidth=1.5)
    ax1.plot(t_opt, ee_opt, 'b-', label='Optimized Fixed PID', linewidth=2.0)
    ax1.set_ylabel('Tool0 3D Error (mm)', fontsize=11, fontweight='bold')
    ax1.set_title('End-Effector Trajectory Tracking Accuracy (Before vs. After Optimization)', fontsize=13, fontweight='bold')
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right', frameon=True)

    z_ref = [float(r['reference_ee_z_m']) * 1000 for r in base_path]
    z_base = [float(r['actual_ee_z_m']) * 1000 for r in base_path]
    z_opt = [float(r['actual_ee_z_m']) * 1000 for r in opt_path]
    ax2.plot(t_base, z_ref, 'k:', label='Reference Z (Set-point)', linewidth=1.5)
    ax2.plot(t_base, z_base, 'r--', label='Baseline Actual Z', alpha=0.85, linewidth=1.5)
    ax2.plot(t_opt, z_opt, 'b-', label='Optimized Actual Z', linewidth=2.0)
    ax2.set_xlabel('Trajectory Time (s)', fontsize=11, fontweight='bold')
    ax2.set_ylabel('Tool0 Z Position (mm)', fontsize=11, fontweight='bold')
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='lower right', frameon=True)
    plt.tight_layout()
    plt.savefig(out_dir / 'before_after_ee_tracking.png', dpi=300)
    plt.close()

    # 2. Joint Errors (6-panel grid)
    fig, axes = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
    axes = axes.flatten()
    for idx, (j, lbl) in enumerate(zip(JOINTS, JOINT_LABELS)):
        ax = axes[idx]
        e_b = np.array([float(r[j + '__position_error_rad']) for r in base_path]) * 180.0 / np.pi
        e_o = np.array([float(r[j + '__position_error_rad']) for r in opt_path]) * 180.0 / np.pi
        ax.plot(t_base, e_b, 'r--', label='Baseline', alpha=0.8, linewidth=1.2)
        ax.plot(t_opt, e_o, 'b-', label='Optimized', linewidth=1.8)
        ax.axhline(0, color='gray', linestyle=':', alpha=0.5)
        ax.set_ylabel(f'{lbl} Error (°)', fontsize=10, fontweight='bold')
        ax.grid(True, linestyle=':', alpha=0.6)
        if idx >= 4:
            ax.set_xlabel('Trajectory Time (s)', fontsize=10, fontweight='bold')
        if idx == 0:
            ax.legend(loc='upper right', frameon=True, fontsize=9)
    plt.suptitle('Six-Joint Position Tracking Errors (Before vs. After Optimization)', fontsize=13, fontweight='bold', y=0.995)
    plt.tight_layout()
    plt.savefig(out_dir / 'before_after_joint_errors.png', dpi=300)
    plt.close()

    # 3. Commanded Torques & Continuous URDF Limits
    fig, axes = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
    axes = axes.flatten()
    for idx, (j, lbl) in enumerate(zip(JOINTS, JOINT_LABELS)):
        ax = axes[idx]
        tau_b = [float(r[j + '__commanded_effort_Nm']) for r in base_path]
        tau_o = [float(r[j + '__commanded_effort_Nm']) for r in opt_path]
        lim = LIMITS[idx]
        ax.plot(t_base, tau_b, 'r--', label='Baseline Torque', alpha=0.7, linewidth=1.2)
        ax.plot(t_opt, tau_o, 'b-', label='Optimized Torque', linewidth=1.8)
        ax.axhline(lim, color='crimson', linestyle='--', linewidth=1.2, label=f'Limit (+{lim:.0f} Nm)')
        ax.axhline(-lim, color='crimson', linestyle='--', linewidth=1.2, label=f'Limit (-{lim:.0f} Nm)')
        ax.set_ylabel(f'{lbl} (Nm)', fontsize=10, fontweight='bold')
        ax.set_ylim(-lim * 1.15, lim * 1.15)
        ax.grid(True, linestyle=':', alpha=0.6)
        if idx >= 4:
            ax.set_xlabel('Trajectory Time (s)', fontsize=10, fontweight='bold')
        if idx == 0:
            ax.legend(loc='upper right', frameon=True, fontsize=8)
    plt.suptitle('Commanded Joint Torques vs. URDF Physical Limits', fontsize=13, fontweight='bold', y=0.995)
    plt.tight_layout()
    plt.savefig(out_dir / 'before_after_torques_margins.png', dpi=300)
    plt.close()

    # 4. 3D Spatial Path Comparison
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    x_ref = [float(r['reference_ee_x_m']) for r in base_path]
    y_ref = [float(r['reference_ee_y_m']) for r in base_path]
    z_ref = [float(r['reference_ee_z_m']) for r in base_path]

    x_base = [float(r['actual_ee_x_m']) for r in base_path]
    y_base = [float(r['actual_ee_y_m']) for r in base_path]
    z_base = [float(r['actual_ee_z_m']) for r in base_path]

    x_opt = [float(r['actual_ee_x_m']) for r in opt_path]
    y_opt = [float(r['actual_ee_y_m']) for r in opt_path]
    z_opt = [float(r['actual_ee_z_m']) for r in opt_path]

    ax.plot(x_ref, y_ref, z_ref, 'k:', label='Reference Path', linewidth=2.0)
    ax.plot(x_base, y_base, z_base, 'r--', label='Baseline Actual Path', linewidth=1.5, alpha=0.85)
    ax.plot(x_opt, y_opt, z_opt, 'b-', label='Optimized Actual Path', linewidth=2.0)
    ax.set_xlabel('X (m)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Y (m)', fontsize=11, fontweight='bold')
    ax.set_zlabel('Z (m)', fontsize=11, fontweight='bold')
    ax.set_title('3D Tool0 Cartesian Path Overlay (Reference vs. Baseline vs. Optimized)', fontsize=12, fontweight='bold')
    ax.legend(loc='upper right', frameon=True)
    plt.tight_layout()
    plt.savefig(out_dir / 'before_after_3d_spatial.png', dpi=300)
    plt.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', required=True, help='Baseline CSV path')
    parser.add_argument('--optimized', required=True, help='Optimized CSV path')
    parser.add_argument('--out-dir', required=True, help='Output directory for plots and JSON')
    args = parser.parse_args()

    base_rows = load_csv(args.baseline)
    opt_rows = load_csv(args.optimized)

    m_base = compute_metrics(base_rows)
    m_opt = compute_metrics(opt_rows)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    summary = {
        'baseline': m_base,
        'optimized': m_opt,
        'improvements': {
            'all_ee_rmse_reduction_mm': m_base['all_ee_rmse_mm'] - m_opt['all_ee_rmse_mm'],
            'all_ee_rmse_reduction_pct': (1.0 - m_opt['all_ee_rmse_mm'] / m_base['all_ee_rmse_mm']) * 100.0,
            'path_ee_rmse_reduction_mm': m_base['path_ee_rmse_mm'] - m_opt['path_ee_rmse_mm'],
            'path_ee_rmse_reduction_pct': (1.0 - m_opt['path_ee_rmse_mm'] / m_base['path_ee_rmse_mm']) * 100.0,
            'max_ee_reduction_mm': m_base['all_ee_max_mm'] - m_opt['all_ee_max_mm'],
            'max_ee_reduction_pct': (1.0 - m_opt['all_ee_max_mm'] / m_base['all_ee_max_mm']) * 100.0,
            'j4_rmse_reduction_deg': m_base['joints']['wrist_1_joint']['all_rmse_deg'] - m_opt['joints']['wrist_1_joint']['all_rmse_deg'],
            'j4_rmse_reduction_pct': (1.0 - m_opt['joints']['wrist_1_joint']['all_rmse_deg'] / m_base['joints']['wrist_1_joint']['all_rmse_deg']) * 100.0,
        }
    }

    with open(out_dir / 'comparison_metrics.json', 'w') as f:
        json.dump(summary, f, indent=2)

    plot_comparisons(base_rows, opt_rows, out_dir)
    print("Comparison plots and metrics generated successfully at", out_dir)


if __name__ == '__main__':
    main()

