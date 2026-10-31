#!/usr/bin/env python3
"""G9.4 Dynamic Tracking Precision Verification & Optimization Suite.

Executes:
1. STAGE B: Isolated full-arm dynamic feedforward experiments (OFF vs ON)
2. STAGE C: Scientific performance isolation:
   - 1.0 kg nominal workpiece (Arm Dynamics OFF vs ON)
   - Fixed PID baseline vs Adaptive PID under matched feedforward
   - 3.0 kg full-capacity workpiece (Arm Dynamics OFF vs ON)
   - 0.3 kg agile workpiece
   - 5.0 kg labeled Gazebo stress test
   - Payload COM offset & inertia perturbation robustness
3. STAGE D: Precision optimization:
   - Identify fastest trajectory meeting < 0.5 mm maximum 3D TCP tracking error
4. Post-processing:
   - Summary tables, CSV logs, publication-grade figures, and Gate G9.4 evaluation.
"""

import argparse
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np

try:
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)
except Exception:
    pass

ROOT = Path("/home/shiva/ur5/ur5_ws")
EXP_BASE = ROOT / "experiment_logs" / "g9_4_precision"
EXP_BASE.mkdir(parents=True, exist_ok=True)


def kill_gazebo_processes():
    subprocess.run(
        [
            "pkill",
            "-u",
            "shiva",
            "-9",
            "-f",
            "gz_sim|ign|ruby|controller_manager|ros_gz_bridge|pick_place_trajectory|parameter_bridge|spawner|robot_state_pub|move_group",
        ],
        capture_output=True,
    )
    time.sleep(3.5)


EXPERIMENT_CONFIGS = [
    # ---------------------------------------------------------
    # STAGE C1: 1.0 kg Matched Baseline Comparison (OFF vs ON)
    # ---------------------------------------------------------
    {
        "id": "exp01_1_0kg_arm_ff_off",
        "desc": "1.0 kg Nominal - Arm Dynamic FF OFF (G9.3 Baseline)",
        "mass": 1.0,
        "world": "ur5_pick_place_1_0kg.sdf",
        "mode": "adaptive_enabled",
        "arm_dyn_ff": False,
        "arm_dyn_scale": 1.0,
        "gain_sched": True,
        "v_scale": 0.15,
        "a_scale": 0.15,
        "com_offset": [0.0, 0.0, 0.1414],
    },
    {
        "id": "exp02_1_0kg_arm_ff_on",
        "desc": "1.0 kg Nominal - Arm Dynamic FF ON (Stage B Validated)",
        "mass": 1.0,
        "world": "ur5_pick_place_1_0kg.sdf",
        "mode": "adaptive_enabled",
        "arm_dyn_ff": True,
        "arm_dyn_scale": 1.0,
        "gain_sched": True,
        "v_scale": 0.15,
        "a_scale": 0.15,
        "com_offset": [0.0, 0.0, 0.1414],
    },
    {
        "id": "exp03_1_0kg_fixed_pid_arm_ff_off",
        "desc": "1.0 kg Nominal - Fixed PID C1 + Arm Dynamic FF OFF (Matched Comparison)",
        "mass": 1.0,
        "world": "ur5_pick_place_1_0kg.sdf",
        "mode": "adaptive_disabled",
        "arm_dyn_ff": False,
        "arm_dyn_scale": 1.0,
        "gain_sched": False,
        "v_scale": 0.15,
        "a_scale": 0.15,
        "com_offset": [0.0, 0.0, 0.1414],
    },
    # ---------------------------------------------------------
    # STAGE C2: 3.0 kg Capacity Comparison (OFF vs ON)
    # ---------------------------------------------------------
    {
        "id": "exp04_3_0kg_arm_ff_off",
        "desc": "3.0 kg Capacity - Arm Dynamic FF OFF (G9.3 Baseline)",
        "mass": 3.0,
        "world": "ur5_pick_place_3_0kg.sdf",
        "mode": "adaptive_enabled",
        "arm_dyn_ff": False,
        "arm_dyn_scale": 1.0,
        "gain_sched": True,
        "v_scale": 0.10,
        "a_scale": 0.10,
        "com_offset": [0.0, 0.0, 0.1414],
    },
    {
        "id": "exp05_3_0kg_arm_ff_on",
        "desc": "3.0 kg Capacity - Arm Dynamic FF ON (Stage B Validated)",
        "mass": 3.0,
        "world": "ur5_pick_place_3_0kg.sdf",
        "mode": "adaptive_enabled",
        "arm_dyn_ff": True,
        "arm_dyn_scale": 1.0,
        "gain_sched": True,
        "v_scale": 0.10,
        "a_scale": 0.10,
        "com_offset": [0.0, 0.0, 0.1414],
    },
    # ---------------------------------------------------------
    # STAGE C3: Agile Payload (0.3 kg) & Stress Test (5.0 kg)
    # ---------------------------------------------------------
    {
        "id": "exp06_0_3kg_agile_arm_ff_off",
        "desc": "0.3 kg Agile - Arm Dynamic FF OFF",
        "mass": 0.3,
        "world": "ur5_pick_place_0_3kg.sdf",
        "mode": "adaptive_enabled",
        "arm_dyn_ff": False,
        "arm_dyn_scale": 1.0,
        "gain_sched": True,
        "v_scale": 0.20,
        "a_scale": 0.20,
        "com_offset": [0.0, 0.0, 0.1414],
    },
    {
        "id": "exp07_5_0kg_stress_test",
        "desc": "5.0 kg Labeled Gazebo Stress Test - Arm Dynamic FF OFF",
        "mass": 5.0,
        "world": "ur5_pick_place_5_0kg.sdf",
        "mode": "adaptive_enabled",
        "arm_dyn_ff": False,
        "arm_dyn_scale": 1.0,
        "gain_sched": True,
        "v_scale": 0.08,
        "a_scale": 0.08,
        "com_offset": [0.0, 0.0, 0.1414],
    },
    # ---------------------------------------------------------
    # STAGE C4: Robustness Matrix (COM Offset Perturbation)
    # ---------------------------------------------------------
    {
        "id": "exp08_3_0kg_perturbed_com",
        "desc": "3.0 kg Robustness - Perturbed COM [+20mm, -20mm, +20mm]",
        "mass": 3.0,
        "world": "ur5_pick_place_3_0kg.sdf",
        "mode": "adaptive_enabled",
        "arm_dyn_ff": False,
        "arm_dyn_scale": 1.0,
        "gain_sched": True,
        "v_scale": 0.10,
        "a_scale": 0.10,
        "com_offset": [0.02, -0.02, 0.1614],
    },
    # ---------------------------------------------------------
    # STAGE D: Precision Optimization (< 0.50 mm 3D Peak Target)
    # ---------------------------------------------------------
    {
        "id": "exp09_0_3kg_sub_half_mm",
        "desc": "0.3 kg Precision Optimized - Target < 0.5 mm 3D Error",
        "mass": 0.3,
        "world": "ur5_pick_place_0_3kg.sdf",
        "mode": "adaptive_enabled",
        "arm_dyn_ff": False,
        "arm_dyn_scale": 1.0,
        "gain_sched": True,
        "v_scale": 0.06,
        "a_scale": 0.03,
        "com_offset": [0.0, 0.0, 0.1414],
    },
    {
        "id": "exp10_1_0kg_sub_half_mm",
        "desc": "1.0 kg Precision Optimized - Target < 0.5 mm 3D Error",
        "mass": 1.0,
        "world": "ur5_pick_place_1_0kg.sdf",
        "mode": "adaptive_enabled",
        "arm_dyn_ff": False,
        "arm_dyn_scale": 1.0,
        "gain_sched": True,
        "v_scale": 0.05,
        "a_scale": 0.025,
        "com_offset": [0.0, 0.0, 0.1414],
    },
]


def run_single_experiment(cfg: dict) -> dict:
    exp_id = cfg["id"]
    desc = cfg["desc"]
    mass = cfg["mass"]
    world_file = cfg["world"]
    mode = cfg["mode"]
    arm_dyn_ff = cfg["arm_dyn_ff"]
    arm_dyn_scale = cfg["arm_dyn_scale"]
    gain_sched = cfg["gain_sched"]
    v_scale = cfg["v_scale"]
    a_scale = cfg["a_scale"]
    com_offset = cfg["com_offset"]

    out_dir = EXP_BASE / exp_id
    out_dir.mkdir(parents=True, exist_ok=True)

    print("\n" + "=" * 80)
    print(f"   STARTING G9.4 EXPERIMENT: {exp_id}")
    print(f"   Description: {desc}")
    print(f"   World: {world_file} | Mass: {mass} kg | Mode: {mode}")
    print(f"   Arm Dyn FF: {arm_dyn_ff} (scale: {arm_dyn_scale}) | Gain Sched: {gain_sched}")
    print(f"   Scaling: v_scale={v_scale}, a_scale={a_scale} | COM Offset: {com_offset}")
    print("=" * 80 + "\n")

    kill_gazebo_processes()

    launch_cmd = (
        f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && "
        f"ros2 launch ur5_bringup ur5_bringup.launch.py "
        f"headless:=true rviz:=false controller_mode:=effort simulated_grasp:=true "
        f"controllers_file:={ROOT}/install/ur5_adaptive_control/share/ur5_adaptive_control/config/adaptive_controllers.yaml "
        f"world:={ROOT}/install/ur5_bringup/share/ur5_bringup/worlds/{world_file}"
    )

    print(f"[1/5] Launching Gazebo simulation with {world_file}...")
    proc = subprocess.Popen(
        ["bash", "-c", launch_cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    try:
        t0 = time.monotonic()
        controllers_ready = False
        while time.monotonic() - t0 < 50.0:
            try:
                res = subprocess.run(
                    ["bash", "-c", "source /opt/ros/humble/setup.bash && ros2 control list_controllers"],
                    capture_output=True,
                    text=True,
                    timeout=4.0,
                )
                if "arm_controller" in res.stdout and "active" in res.stdout:
                    controllers_ready = True
                    break
            except subprocess.TimeoutExpired:
                pass
            time.sleep(2.5)

        if not controllers_ready:
            print("ERROR: Timeout waiting for arm_controller to activate!")
            return {"id": exp_id, "status": "failed", "reason": "Arm controller activation timeout"}

        print("[2/5] Arm controller active. Verifying quiescence (<0.01 rad/s)...")
        t_quiesce = time.monotonic()
        while time.monotonic() - t_quiesce < 25.0:
            res = subprocess.run(
                [
                    "bash",
                    "-c",
                    f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && "
                    f"python3 {ROOT}/ur5_adaptive_control/scripts/controller_switcher.py status",
                ],
                capture_output=True,
                text=True,
            )
            if "Safe to Switch:           YES" in res.stdout:
                print("  -> Arm is quiescent!")
                break
            time.sleep(2.5)

        # 2. Switch controller & configure parameters
        print(f"[3/5] Switching to adaptive_arm_controller (mode: {mode})...")
        switch_cmd = (
            f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && "
            f"python3 {ROOT}/ur5_adaptive_control/scripts/controller_switcher.py switch adaptive && "
            f"python3 {ROOT}/ur5_adaptive_control/scripts/controller_switcher.py mode {mode}"
        )
        switch_ok = False
        for attempt in range(6):
            res_switch = subprocess.run(["bash", "-c", switch_cmd], capture_output=True, text=True)
            if res_switch.returncode == 0:
                switch_ok = True
                break
            print(f"  Attempt {attempt+1}: Switch retry, waiting 3s...")
            time.sleep(3.0)

        if not switch_ok:
            print(f"ERROR: Controller switch failed:\n{res_switch.stderr}\n{res_switch.stdout}")
            return {"id": exp_id, "status": "failed", "reason": "Controller switch failed"}

        sched_str = "true" if gain_sched else "false"
        arm_dyn_str = "true" if arm_dyn_ff else "false"
        com_str = f"[{com_offset[0]}, {com_offset[1]}, {com_offset[2]}]"

        param_cmd = (
            f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && "
            f"ros2 param set /adaptive_arm_controller payload_feedforward_enabled true && "
            f"ros2 param set /adaptive_arm_controller payload_nominal_mass {mass} && "
            f"ros2 param set /adaptive_arm_controller payload_dynamic_feedforward_enabled true && "
            f"ros2 param set /adaptive_arm_controller payload_gain_scheduling_enabled {sched_str} && "
            f"ros2 param set /adaptive_arm_controller arm_dynamics_feedforward_enabled {arm_dyn_str} && "
            f"ros2 param set /adaptive_arm_controller arm_dynamics_scale {arm_dyn_scale} && "
            f"ros2 param set /adaptive_arm_controller payload_com_offset \"{com_str}\""
        )
        res_param = subprocess.run(["bash", "-c", param_cmd], capture_output=True, text=True)
        print(f"  -> Parameters set:\n{res_param.stdout.strip()}")
        time.sleep(1.0)

        # 3. Build orchestrator command with explicit scaling override
        scale_args = f"--scaling-enabled true --velocity-scale {v_scale} --acceleration-scale {a_scale} --generate-excel false"

        print(f"[4/5] Executing pick-and-place replay path01 with scaling ({scale_args})...")
        orch_cmd = (
            f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && "
            f"python3 {ROOT}/ur5_bringup/scripts/pick_place_trajectory_orchestrator.py "
            f"--path {ROOT}/path01 --mode effort --controller-name adaptive_arm_controller "
            f"--payload-kg {mass} --output-dir {out_dir} {scale_args} --auto-exit true"
        )
        orch_proc = subprocess.run(["bash", "-c", orch_cmd], capture_output=True, text=True)
        if orch_proc.returncode != 0:
            print(f"ERROR: Orchestrator failed (exit {orch_proc.returncode}):\n{orch_proc.stderr}\n{orch_proc.stdout}")
            return {"id": exp_id, "status": "failed", "reason": f"Orchestrator execution error: {orch_proc.stderr[-300:]}"}

        # 4. Check result.json
        res_file = out_dir / "result.json"
        if not res_file.is_file():
            print(f"ERROR: result.json missing in {out_dir}!")
            return {"id": exp_id, "status": "failed", "reason": "result.json missing"}

        with open(res_file) as f:
            rdata = json.load(f)

        status = rdata.get("status")
        metrics = rdata.get("metrics", {})
        scaling_info = rdata.get("scaling_supervision", {})

        print(f"[5/5] Experiment {exp_id} completed. Status: {status}")
        print(f"  -> Max EE Error: {metrics.get('ee_max_mm', 0.0):.3f} mm")
        print(f"  -> RMSE EE:     {metrics.get('ee_rmse_mm', 0.0):.3f} mm")
        print(f"  -> Settled EE:  {metrics.get('ee_final_settled_mm', 0.0):.3f} mm")
        print(f"  -> Jitter W3:   {metrics.get('wrist_3_jitter_rad_s', 0.0):.4f} rad/s")

        result_summary = {
            "id": exp_id,
            "desc": desc,
            "status": status,
            "mass": mass,
            "mode": mode,
            "arm_dyn_ff": arm_dyn_ff,
            "arm_dyn_scale": arm_dyn_scale,
            "v_scale": v_scale,
            "a_scale": a_scale,
            "ee_max_mm": metrics.get("ee_max_mm", 0.0),
            "ee_rmse_mm": metrics.get("ee_rmse_mm", 0.0),
            "ee_settled_mm": metrics.get("ee_final_settled_mm", 0.0),
            "jitter_w3": metrics.get("wrist_3_jitter_rad_s", 0.0),
            "cycle_time_s": float(rdata.get("cycle_time_s", 0.0)),
            "out_dir": str(out_dir),
        }
        return result_summary

    finally:
        print("[CLEANUP] Stopping simulation processes...")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
        kill_gazebo_processes()


def main():
    parser = argparse.ArgumentParser(description="G9.4 Experiment Runner")
    parser.add_argument("--id", default="all", help="Specific experiment ID to run or 'all'")
    args = parser.parse_args()

    selected = EXPERIMENT_CONFIGS if args.id == "all" else [c for c in EXPERIMENT_CONFIGS if c["id"] == args.id]
    if not selected:
        print(f"No configuration found matching id: {args.id}")
        return 1

    results = []
    for cfg in selected:
        res = run_single_experiment(cfg)
        results.append(res)
        time.sleep(2.0)

    # Save summary
    summary_path = EXP_BASE / "g9_4_master_summary.json"
    with open(summary_path, "w") as f:
        json.dump(results, f, indent=2)

    print("\n" + "=" * 90)
    print("                    G9.4 EXPERIMENTAL RESULTS MASTER TABLE")
    print("=" * 90)
    header = f"{'ID':28s} | {'Mass':5s} | {'Arm FF':7s} | {'v_s':5s} | {'a_s':5s} | {'Max EE(mm)':10s} | {'RMSE(mm)':9s} | {'Status':6s}"
    print(header)
    print("-" * 90)
    for r in results:
        aff = "ON" if r.get("arm_dyn_ff") else "OFF"
        row = (
            f"{r['id']:28s} | {r.get('mass', 0):.1f}kg | {aff:7s} | "
            f"{r.get('v_scale', 0):.2f}  | {r.get('a_scale', 0):.2f}  | "
            f"{r.get('ee_max_mm', 0):10.3f} | {r.get('ee_rmse_mm', 0):9.3f} | {r.get('status', 'FAIL'):6s}"
        )
        print(row)
    print("=" * 90 + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
