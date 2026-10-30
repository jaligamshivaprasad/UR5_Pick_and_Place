#!/usr/bin/env python3
"""
run_phase6_experiments.py
Automated runner for UR5 Phase 6 Physical Payload Variations (1.0 kg, 0.5 kg, 0.3 kg).
Sequentially launches Gazebo Sim with each dedicated world, executes path01, logs results,
and validates performance.
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

MASS_CONFIGS = [
    {
        "mass": 1.0,
        "name": "test_phase6_1_0kg",
        "world_file": "ur5_pick_place_1_0kg.sdf"
    },
    {
        "mass": 0.5,
        "name": "test_phase6_0_5kg",
        "world_file": "ur5_pick_place_0_5kg.sdf"
    },
    {
        "mass": 0.3,
        "name": "test_phase6_0_3kg",
        "world_file": "ur5_pick_place_0_3kg.sdf"
    }
]

def kill_gazebo_processes():
    subprocess.run(["pkill", "-9", "-f", "ign|gz|controller_manager|ros_gz_bridge"], capture_output=True)
    time.sleep(2.0)

def run_single_mass_trial(cfg):
    mass = cfg["mass"]
    trial_name = cfg["name"]
    world_file = cfg["world_file"]
    out_dir = EXP_BASE / trial_name
    out_dir.mkdir(parents=True, exist_ok=True)

    print("\n" + "=" * 80)
    print(f"   STARTING PHASE 6 TRIAL: PAYLOAD {mass:.1f} kg ({trial_name})")
    print(f"   World: {world_file} | Output: {out_dir}")
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
        # Wait for controller_manager and arm_controller
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

        print("[2/5] Controllers active. Waiting for robot to settle into quiescence (< 0.01 rad/s)...")
        t_quiesce = time.monotonic()
        quiescent = False
        while time.monotonic() - t_quiesce < 30.0:
            res = subprocess.run(
                ["bash", "-c", f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && python3 {ROOT}/ur5_adaptive_control/scripts/controller_switcher.py status"],
                capture_output=True, text=True
            )
            if "Safe to Switch:           YES" in res.stdout:
                quiescent = True
                print("  -> Robot confirmed quiescent and safe to switch!")
                break
            time.sleep(2.0)

        if not quiescent:
            print("WARNING: Quiescence timeout reached. Proceeding to switch with internal supervisor stabilization...")

        # 2. Switch to adaptive_arm_controller and enable payload feedforward
        print(f"[3/5] Switching to adaptive_arm_controller (mode: adaptive_enabled, FF: true, mass: {mass} kg)...")
        switch_cmd = (
            f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && "
            f"python3 {ROOT}/ur5_adaptive_control/scripts/controller_switcher.py switch adaptive && "
            f"python3 {ROOT}/ur5_adaptive_control/scripts/controller_switcher.py mode adaptive_enabled"
        )
        switch_ok = False
        for attempt in range(6):
            res_switch = subprocess.run(["bash", "-c", switch_cmd], capture_output=True, text=True)
            if res_switch.returncode == 0:
                switch_ok = True
                break
            print(f"  Attempt {attempt+1}: Switch rejected due to motion, waiting 3s for quiescence...")
            time.sleep(3.0)

        if not switch_ok:
            print(f"ERROR: Controller switch failed after retries:\n{res_switch.stderr}\n{res_switch.stdout}")
            return False

        # Set payload parameters
        param_cmd = (
            f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && "
            f"ros2 param set /adaptive_arm_controller payload_feedforward_enabled true && "
            f"ros2 param set /adaptive_arm_controller payload_nominal_mass {mass}"
        )
        res_param = subprocess.run(["bash", "-c", param_cmd], capture_output=True, text=True)
        print(f"  -> Parameter response:\n{res_param.stdout.strip()}")
        time.sleep(1.0)

        # 3. Run trajectory orchestrator
        print(f"[4/5] Executing pick-and-place replay path01 for payload {mass} kg...")
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
    results = {}
    for cfg in MASS_CONFIGS:
        # Check if already passed
        res_file = EXP_BASE / cfg["name"] / "result.json"
        if res_file.is_file():
            try:
                with open(res_file) as f:
                    rdata = json.load(f)
                if rdata.get("status") == "passed":
                    print(f"Skipping already passed trial: {cfg['name']} ({cfg['mass']} kg)")
                    results[cfg["mass"]] = True
                    continue
            except Exception:
                pass

        ok = run_single_mass_trial(cfg)
        results[cfg["mass"]] = ok
        if not ok:
            print(f"HALTING: Trial failed for mass {cfg['mass']} kg!")
            break

    print("\n" + "=" * 80)
    print("           PHASE 6 PAYLOAD VARIATION EXECUTION SUMMARY")
    print("=" * 80)
    for mass, ok in results.items():
        print(f"  - Payload {mass:.1f} kg: [{'PASS' if ok else 'FAIL'}]")
    print("=" * 80 + "\n")

    return 0 if all(results.values()) and len(results) == len(MASS_CONFIGS) else 1

if __name__ == "__main__":
    sys.exit(main())
