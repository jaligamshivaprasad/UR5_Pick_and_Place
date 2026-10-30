#!/usr/bin/env python3
"""
run_phase7_experiments.py
Automated runner for UR5 Phase 7: Dynamic Acceleration Feedforward & Gain Scheduling.

1. Executes 7 progressive safety tests via test_progressive_phase7.py.
2. Controlled Comparison Suite on 1.0 kg:
   - Variant A: G6 Baseline (Adaptive PID + Gravity FF) [Preserved from Phase 6]
   - Variant B: Adaptive PID + Gravity FF + Dynamic FF (test_phase7_1_0kg_dyn)
   - Variant C: Full Phase 7: Adaptive PID + Gravity FF + Dynamic FF + Gain Sched (test_phase7_1_0kg_full)
   - Fixed PID Control: Fixed C1 PID + Gravity FF + Dynamic FF (test_phase7_1_0kg_fixed_dyn)
3. Multi-Payload Validation Matrix under Full Phase 7 (Variant C):
   - 0.3 kg:  test_phase7_0_3kg
   - 0.5 kg:  test_phase7_0_5kg
   - 0.75 kg: test_phase7_0_75kg (Unseen payload generalization)
   - 3.0 kg:  test_phase7_3_0kg  (Reference baseline)
"""

import os
import sys
import time
import subprocess
import signal
import json
from pathlib import Path

ROOT = Path("/home/shiva/ur5/ur5_ws")
EXP_BASE = ROOT / "experiment_logs"

TRIALS = [
    # --- Controlled Comparisons on 1.0 kg (Task 9) ---
    {
        "name": "test_phase7_1_0kg_dyn",
        "mass": 1.0,
        "world_file": "ur5_pick_place_1_0kg.sdf",
        "mode": "adaptive_enabled",
        "dynamic_ff": True,
        "gain_sched": False,
        "desc": "Variant B: Adaptive PID + Gravity FF + Dynamic FF (1.0 kg)"
    },
    {
        "name": "test_phase7_1_0kg_full",
        "mass": 1.0,
        "world_file": "ur5_pick_place_1_0kg.sdf",
        "mode": "adaptive_enabled",
        "dynamic_ff": True,
        "gain_sched": True,
        "desc": "Variant C: Full Phase 7 (Adaptive + Gravity + Dynamic FF + Gain Sched) (1.0 kg)"
    },
    {
        "name": "test_phase7_1_0kg_fixed_dyn",
        "mass": 1.0,
        "world_file": "ur5_pick_place_1_0kg.sdf",
        "mode": "adaptive_disabled",
        "dynamic_ff": True,
        "gain_sched": False,
        "desc": "Matched Control: Fixed C1 PID + Gravity FF + Dynamic FF (1.0 kg)"
    },

    # --- Multi-Payload Scaling (Task 10) under Full Phase 7 ---
    {
        "name": "test_phase7_0_3kg",
        "mass": 0.3,
        "world_file": "ur5_pick_place_0_3kg.sdf",
        "mode": "adaptive_enabled",
        "dynamic_ff": True,
        "gain_sched": True,
        "desc": "Full Phase 7: 0.3 kg payload"
    },
    {
        "name": "test_phase7_0_5kg",
        "mass": 0.5,
        "world_file": "ur5_pick_place_0_5kg.sdf",
        "mode": "adaptive_enabled",
        "dynamic_ff": True,
        "gain_sched": True,
        "desc": "Full Phase 7: 0.5 kg payload"
    },
    {
        "name": "test_phase7_0_75kg",
        "mass": 0.75,
        "world_file": "ur5_pick_place_0_75kg.sdf",
        "mode": "adaptive_enabled",
        "dynamic_ff": True,
        "gain_sched": True,
        "desc": "Full Phase 7: 0.75 kg UNSEEN generalization payload"
    },
    {
        "name": "test_phase7_3_0kg",
        "mass": 3.0,
        "world_file": "ur5_pick_place_3_0kg.sdf",
        "mode": "adaptive_enabled",
        "dynamic_ff": True,
        "gain_sched": True,
        "desc": "Full Phase 7: 3.0 kg reference baseline payload"
    }
]


def kill_gazebo_processes():
    subprocess.run(["pkill", "-u", "shiva", "-9", "-f", "gz_sim|ign_gazebo|controller_manager|ros_gz_bridge|pick_place_trajectory"], capture_output=True)
    time.sleep(2.0)


def run_progressive_suite():
    print("\n" + "=" * 80)
    print("      STAGE 1: EXECUTING 7 PROGRESSIVE SAFETY & COMPENSATOR TESTS")
    print("=" * 80 + "\n")
    kill_gazebo_processes()

    launch_cmd = (
        f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && "
        f"ros2 launch ur5_bringup ur5_bringup.launch.py "
        f"headless:=true rviz:=false controller_mode:=effort simulated_grasp:=true "
        f"controllers_file:={ROOT}/install/ur5_adaptive_control/share/ur5_adaptive_control/config/adaptive_controllers.yaml "
        f"world:={ROOT}/install/ur5_bringup/share/ur5_bringup/worlds/ur5_pick_place_1_0kg.sdf"
    )
    proc = subprocess.Popen(["bash", "-c", launch_cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, preexec_fn=os.setsid)

    try:
        t0 = time.monotonic()
        ready = False
        while time.monotonic() - t0 < 45.0:
            res = subprocess.run(["bash", "-c", "source /opt/ros/humble/setup.bash && ros2 control list_controllers"], capture_output=True, text=True)
            if "arm_controller" in res.stdout and "active" in res.stdout:
                ready = True
                break
            time.sleep(2.0)
        if not ready:
            print("ERROR: arm_controller failed to activate for progressive suite!")
            return False

        # Run progressive script
        prog_cmd = (
            f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && "
            f"python3 {ROOT}/ur5_adaptive_control/scripts/test_progressive_phase7.py"
        )
        prog_res = subprocess.run(["bash", "-c", prog_cmd], capture_output=True, text=True)
        print(prog_res.stdout)
        if prog_res.returncode != 0:
            print(f"ERROR: Progressive suite failed:\n{prog_res.stderr}")
            return False
        return True
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
        kill_gazebo_processes()


def run_single_trial(cfg):
    trial_name = cfg["name"]
    mass = cfg["mass"]
    world_file = cfg["world_file"]
    mode = cfg["mode"]
    dyn_ff = cfg["dynamic_ff"]
    gain_sched = cfg["gain_sched"]
    desc = cfg["desc"]

    out_dir = EXP_BASE / trial_name
    out_dir.mkdir(parents=True, exist_ok=True)

    print("\n" + "=" * 80)
    print(f"   STARTING PHASE 7 TRIAL: {trial_name}")
    print(f"   Description: {desc}")
    print(f"   World: {world_file} | Mass: {mass} kg | Mode: {mode}")
    print(f"   Dynamic FF: {dyn_ff} | Gain Sched: {gain_sched} | Out: {out_dir}")
    print("=" * 80 + "\n")

    kill_gazebo_processes()

    # 1. Launch Gazebo bringup
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
        preexec_fn=os.setsid
    )

    try:
        t0 = time.monotonic()
        controllers_ready = False
        while time.monotonic() - t0 < 45.0:
            res = subprocess.run(
                ["bash", "-c", f"source /opt/ros/humble/setup.bash && ros2 control list_controllers"],
                capture_output=True, text=True
            )
            if "arm_controller" in res.stdout and "active" in res.stdout:
                controllers_ready = True
                break
            time.sleep(2.0)

        if not controllers_ready:
            print("ERROR: Timeout waiting for arm_controller to activate!")
            return False

        print("[2/5] Controllers active. Waiting for robot quiescence (< 0.01 rad/s)...")
        t_quiesce = time.monotonic()
        while time.monotonic() - t_quiesce < 25.0:
            res = subprocess.run(
                ["bash", "-c", f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && python3 {ROOT}/ur5_adaptive_control/scripts/controller_switcher.py status"],
                capture_output=True, text=True
            )
            if "Safe to Switch:           YES" in res.stdout:
                print("  -> Robot quiescent!")
                break
            time.sleep(2.0)

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
            return False

        # Configure parameters online
        dyn_str = "true" if dyn_ff else "false"
        sched_str = "true" if gain_sched else "false"
        param_cmd = (
            f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && "
            f"ros2 param set /adaptive_arm_controller payload_feedforward_enabled true && "
            f"ros2 param set /adaptive_arm_controller payload_nominal_mass {mass} && "
            f"ros2 param set /adaptive_arm_controller payload_dynamic_feedforward_enabled {dyn_str} && "
            f"ros2 param set /adaptive_arm_controller payload_gain_scheduling_enabled {sched_str}"
        )
        res_param = subprocess.run(["bash", "-c", param_cmd], capture_output=True, text=True)
        print(f"  -> Parameters set:\n{res_param.stdout.strip()}")
        time.sleep(1.0)

        # 3. Run trajectory orchestrator
        print(f"[4/5] Executing pick-and-place replay path01...")
        orch_cmd = (
            f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && "
            f"python3 {ROOT}/ur5_bringup/scripts/pick_place_trajectory_orchestrator.py "
            f"--path {ROOT}/path01 --mode effort --controller-name adaptive_arm_controller "
            f"--payload-kg {mass} --output-dir {out_dir} --auto-exit false"
        )
        orch_proc = subprocess.run(["bash", "-c", orch_cmd], capture_output=True, text=True)
        print(orch_proc.stdout)
        if orch_proc.returncode != 0:
            print(f"ERROR: Orchestrator failed:\n{orch_proc.stderr}")
            return False

        # 4. Check result.json
        res_file = out_dir / "result.json"
        if not res_file.is_file():
            print(f"ERROR: result.json missing in {out_dir}!")
            return False

        with open(res_file) as f:
            rdata = json.load(f)

        status = rdata.get("status")
        metrics = rdata.get("metrics", {})
        print(f"[5/5] Trial {trial_name} completed. Status: {status}")
        print(f"  -> Max EE Error: {metrics.get('ee_max_mm', 0.0):.3f} mm")
        print(f"  -> RMSE EE:     {metrics.get('ee_rmse_mm', 0.0):.3f} mm")
        print(f"  -> Settled EE:  {metrics.get('ee_final_settled_mm', 0.0):.3f} mm")
        print(f"  -> Jitter W3:   {metrics.get('wrist_3_jitter_rad_s', 0.0):.4f} rad/s")

        return status == "passed"
    finally:
        print("[CLEANUP] Stopping simulation processes...")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
        kill_gazebo_processes()


def main():
    # 1. Run progressive safety test suite
    if not run_progressive_suite():
        print("HALTING: Progressive safety suite failed!")
        return 1

    # 2. Run trials
    results = {}
    for cfg in TRIALS:
        res_file = EXP_BASE / cfg["name"] / "result.json"
        if res_file.is_file():
            try:
                with open(res_file) as f:
                    rdata = json.load(f)
                if rdata.get("status") == "passed":
                    print(f"Skipping already passed trial: {cfg['name']}")
                    results[cfg["name"]] = True
                    continue
            except Exception:
                pass

        ok = run_single_trial(cfg)
        results[cfg["name"]] = ok
        if not ok:
            print(f"HALTING: Trial {cfg['name']} failed!")
            break

    print("\n" + "=" * 80)
    print("                PHASE 7 EXPERIMENTAL EXECUTION SUMMARY")
    print("=" * 80)
    for name, ok in results.items():
        print(f"  - {name:32s}: [{'PASS' if ok else 'FAIL'}]")
    print("=" * 80 + "\n")

    return 0 if all(results.values()) and len(results) == len(TRIALS) else 1


if __name__ == "__main__":
    sys.exit(main())
