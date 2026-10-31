#!/usr/bin/env python3
"""Summarize and plot recorded release isolation trials on simulation time."""

import csv
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ur5_data_logger'))
from ur5_data_logger.fk import VerifiedFK

JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')
EVENTS = {'R1_empty_gripper': 'gripper_command_0.0',
          'R1_empty_gripper_place': 'gripper_command_0.0',
          'R2_feedforward_only': 'synthetic_payload_detach',
          'R2_feedforward_place': 'synthetic_payload_detach',
          'R3_detach_closed': 'state_detached',
          'R4_complete_release': 'state_detached'}
BASE = ROOT / 'experiment_logs' / 'release_diagnosis'
URDF = ROOT / 'experiment_logs' / 'test_phase9_trial4_3_0kg_adaptive_scaled' / 'robot.urdf'
fk = VerifiedFK(URDF.read_text(), 'base_link', 'tool0', JOINTS)


def number(row, key):
    try:
        return float(row[key])
    except (KeyError, ValueError, TypeError):
        return float('nan')


summary = {}
for case, event in EVENTS.items():
    folder = BASE / case
    if not (folder / 'samples.csv').exists():
        continue
    rows = list(csv.DictReader((folder / 'samples.csv').open()))
    events = json.loads((folder / 'events.json').read_text())
    event_times = [e['time_sec'] for e in events if e['event'] == event]
    t0 = event_times[-1]
    selected = [r for r in rows if -1 <= number(r, 'time_sec') - t0 <= 2]
    pre = [r for r in selected if number(r, 'time_sec') < t0]
    post = [r for r in selected if number(r, 'time_sec') >= t0]
    if not pre or not post:
        continue
    times = np.array([number(r, 'time_sec') - t0 for r in selected])
    ref = np.array([[number(r, j + '_ref_rad') for j in JOINTS] for r in selected])
    pos = np.array([[number(r, j + '_pos_rad') for j in JOINTS] for r in selected])
    vel = np.array([[number(r, j + '_vel_rad_s') for j in JOINTS] for r in selected])
    cmd = np.array([[number(r, j + '_cmd_Nm') for j in JOINTS] for r in selected])
    ff = np.array([[number(r, j + '_ff_Nm') for j in JOINTS] for r in selected])
    tcp = np.array([1000 * np.linalg.norm(fk.xyz(qr) - fk.xyz(qa))
                    for qr, qa in zip(ref, pos)])
    z = np.array([number(r, 'block_z_m') for r in selected])
    alpha = np.array([number(r, 'payload_alpha') for r in selected])
    grip = np.array([number(r, 'gripper_rad') for r in selected])
    i0 = len(pre)
    contact = np.where((times >= 0) & (z <= 0.405))[0]
    metrics = {
        'event_sim_sec': t0,
        'pre_samples': len(pre), 'post_samples': len(post),
        'post_window_sim_sec': float(times[-1]),
        'post_max_tcp_tracking_error_mm': float(np.nanmax(tcp[i0:])),
        'pre_max_tcp_tracking_error_mm': float(np.nanmax(tcp[:i0])),
        'post_peak_abs_joint_velocity_rad_s': float(np.nanmax(np.abs(vel[i0:]))),
        'pre_peak_abs_joint_velocity_rad_s': float(np.nanmax(np.abs(vel[:i0]))),
        'post_max_joint_position_excursion_rad': float(np.nanmax(np.abs(pos[i0:] - pos[i0 - 1]))),
        'post_max_reference_excursion_rad': float(np.nanmax(np.abs(ref[i0:] - ref[i0 - 1]))),
        'max_command_step_Nm': float(np.nanmax(np.abs(np.diff(cmd, axis=0)))),
        'block_z_before_m': float(z[i0 - 1]),
        'block_z_after_m': float(z[-1]),
        'first_block_tray_height_crossing_sec': float(times[contact[0]]) if len(contact) else None,
        'gripper_before_rad': float(grip[i0 - 1]),
        'gripper_after_rad': float(grip[-1]),
        'alpha_before': float(alpha[i0 - 1]),
        'alpha_after': float(alpha[-1]),
    }
    summary[case] = metrics

    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(times, tcp, label='3D tool0 reference error (mm)')
    axes[0].plot(times, 1000 * np.linalg.norm(np.array([fk.xyz(q) for q in pos]) -
                                           fk.xyz(pos[i0 - 1]), axis=1),
                 label='3D tool0 movement from release (mm)', alpha=0.65)
    axes[0].set_ylabel('TCP (mm)')
    axes[0].legend(loc='upper right', fontsize=8)
    axes[1].plot(times, np.max(np.abs(vel), axis=1))
    axes[1].set_ylabel('Peak |joint velocity| (rad/s)')
    axes[2].plot(times, cmd[:, 1], label='shoulder lift total command')
    if np.isfinite(ff[:, 1]).any():
        axes[2].plot(times, ff[:, 1], label='payload feedforward')
    axes[2].set_ylabel('Torque (Nm)')
    axes[2].legend(loc='upper right', fontsize=8)
    axes[3].plot(times, z, label='block center z (m)')
    axes[3].axhline(0.405, color='gray', linestyle=':', label='tray top + block half height')
    axes[3].plot(times, grip, label='gripper angle (rad)')
    if np.isfinite(alpha).any():
        axes[3].plot(times, alpha, label='payload alpha')
    axes[3].set_ylabel('Scene / state')
    axes[3].set_xlabel('Simulation seconds from release event')
    axes[3].legend(loc='upper right', fontsize=8)
    for axis in axes:
        axis.axvline(0, color='red', linewidth=0.8)
        axis.grid(alpha=0.2)
    fig.suptitle(case + ': event aligned on simulation clock')
    fig.tight_layout()
    fig.savefig(folder / 'event_timing.png', dpi=160)
    plt.close(fig)

(BASE / 'summary.json').write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
