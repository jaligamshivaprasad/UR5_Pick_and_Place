#!/usr/bin/env python3
"""
run_phase8_experiments.py
Automated Experiment Runner for UR5 Phase 8: Automatic Payload-Aware Trajectory Scaling.

Executes:
1. STAGE 1: 8 Progressive Safety & Retiming Verification Tests (test_progressive_phase8.py).
2. STAGE 2: Controlled Timing Isolation on 1.0 kg:
   - Variant A Scaled (test_phase8_1_0kg_variant_a_scaled) vs Variant A Unscaled (Phase 6/7 baseline)
   - Variant B Scaled (test_phase8_1_0kg_variant_b_scaled)
   - Variant C Scaled (test_phase8_1_0kg_variant_c_scaled)
   - Matched Fixed Control Scaled (test_phase8_1_0kg_fixed_scaled)
3. STAGE 3: Scaling Factor Sweep on 1.0 kg (Pareto Frontier Evaluation):
   - Nominal (s=0.20): test_phase7_1_0kg_full (from Phase 7)
   - Optimized Pareto (s=0.15): test_phase8_1_0kg_variant_c_scaled
   - High Precision (s=0.10): test_phase8_1_0kg_scale_0_10
4. STAGE 4: Multi-Payload Scaling Matrix under Automatic Profile Selection:
   - 0.30 kg: test_phase8_0_3kg_scaled (s=0.25)
   - 0.50 kg: test_phase8_0_5kg_scaled (s=0.22)
   - 0.75 kg: test_phase8_0_75kg_scaled (s=0.18, UNSEEN generalization mass)
   - 3.00 kg: test_phase8_3_0kg_scaled (s=0.10, reference capacity)
5. STAGE 5: Conservative Fallback Verification:
   - test_phase8_fallback_unverified: Unverified payload triggering fallback (s=0.10)
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path("/home/shiva/ur5/ur5_ws")
EXP_BASE = ROOT / "experiment_logs"

TRIALS = [
    # --- STAGE 2: Controlled Isolation & Mode Comparisons on 1.0 kg ---
    {
        "name": "test_phase8_1_0kg_variant_a_scaled",
        "mass": 1.0,
        "world_file": "ur5_pick_place_1_0kg.sdf",
        "mode": "adaptive_enabled",
        "dynamic_ff": False,
        "gain_sched": False,
        "v_scale": 0.15,
        "a_scale": 0.15,
        "fallback": False,
        "desc": "Timing Isolation: Variant A (Adaptive + Pure Gravity FF) with retimed s=0.15",
    },
    {
        "name": "test_phase8_1_0kg_fixed_scaled",
        "mass": 1.0,
        "world_file": "ur5_pick_place_1_0kg.sdf",
        "mode": "adaptive_disabled",
        "dynamic_ff": True,
        "gain_sched": False,
        "v_scale": 0.15,
        "a_scale": 0.15,
        "fallback": False,
        "desc": "Matched Control: Fixed C1 PID + Gravity FF + Dynamic FF with retimed s=0.15",
    },
    {
        "name": "test_phase8_1_0kg_variant_b_scaled",
        "mass": 1.0,
        "world_file": "ur5_pick_place_1_0kg.sdf",
        "mode": "adaptive_enabled",
        "dynamic_ff": True,
        "gain_sched": False,
        "v_scale": 0.15,
        "a_scale": 0.15,
        "fallback": False,
        "desc": "Variant B: Adaptive PID + Gravity FF + Dynamic FF with retimed s=0.15",
    },
    {
        "name": "test_phase8_1_0kg_variant_c_scaled",
        "mass": 1.0,
        "world_file": "ur5_pick_place_1_0kg.sdf",
        "mode": "adaptive_enabled",
        "dynamic_ff": True,
        "gain_sched": True,
        "v_scale": 0.15,
        "a_scale": 0.15,
        "fallback": False,
        "desc": "Full Phase 8 (Variant C): Adaptive + Gravity + Dynamic FF + Gain Sched with retimed s=0.15",
    },

    # --- STAGE 3: Scale Sweep / High-Precision Profile on 1.0 kg ---
    {
        "name": "test_phase8_1_0kg_scale_0_10",
        "mass": 1.0,
        "world_file": "ur5_pick_place_1_0kg.sdf",
        "mode": "adaptive_enabled",
        "dynamic_ff": True,
        "gain_sched": True,
        "v_scale": 0.10,
        "a_scale": 0.10,
        "fallback": False,
        "desc": "High-Precision 1.0 kg Sweep: s=0.10 for near-millimeter tracking",
    },

    # --- STAGE 4: Multi-Payload Scaling Matrix under Automatic Selection ---
    {
        "name": "test_phase8_0_3kg_scaled",
        "mass": 0.3,
        "world_file": "ur5_pick_place_0_3kg.sdf",
        "mode": "adaptive_enabled",
        "dynamic_ff": True,
        "gain_sched": True,
        "v_scale": 0.25,
        "a_scale": 0.25,
        "fallback": False,
        "desc": "Optimal Profile 0.3 kg: Agility scaling s=0.25",
    },
    {
        "name": "test_phase8_0_5kg_scaled",
        "mass": 0.5,
        "world_file": "ur5_pick_place_0_5kg.sdf",
        "mode": "adaptive_enabled",
        "dynamic_ff": True,
        "gain_sched": True,
        "v_scale": 0.22,
        "a_scale": 0.22,
        "fallback": False,
        "desc": "Optimal Profile 0.5 kg: Balanced scaling s=0.22",
    },
    {
        "name": "test_phase8_0_75kg_scaled",
        "mass": 0.75,
        "world_file": "ur5_pick_place_0_75kg.sdf",
        "mode": "adaptive_enabled",
        "dynamic_ff": True,
        "gain_sched": True,
        "v_scale": 0.18,
        "a_scale": 0.18,
        "fallback": False,
        "desc": "Optimal Profile 0.75 kg (UNSEEN generalization payload): Scaling s=0.18",
    },
    {
        "name": "test_phase8_3_0kg_scaled",
        "mass": 3.0,
        "world_file": "ur5_pick_place_3_0kg.sdf",
        "mode": "adaptive_enabled",
        "dynamic_ff": True,
        "gain_sched": True,
        "v_scale": 0.10,
        "a_scale": 0.10,
        "fallback": False,
        "desc": "Optimal Profile 3.0 kg Reference: Heavy capacity scaling s=0.10",
    },

    # --- STAGE 5: Conservative Fallback Verification ---
    {
        "name": "test_phase8_fallback_unverified",
        "mass": 1.0,
        "world_file": "ur5_pick_place_1_0kg.sdf",
        "mode": "adaptive_enabled",
        "dynamic_ff": True,
        "gain_sched": True,
        "v_scale": None,
        "a_scale": None,
        "fallback": True,
        "desc": "Conservative Fallback: Unverified payload triggering fallback s=0.10",
    },
]


def kill_gazebo_processes():
    subprocess.run(
        ["pkill", "-u", "shiva", "-9", "-f", "gz_sim|ign_gazebo|controller_manager|ros_gz_bridge|pick_place_trajectory"],
        capture_output=True,
    )
    time.sleep(3.5)


def run_progressive_suite() -> bool:
    print("\n" + "=" * 80)
    print("      STAGE 1: EXECUTING 8 PROGRESSIVE SAFETY & RETIMING TESTS")
    print("=" * 80 + "\n")
    kill_gazebo_processes()

    launch_cmd = (
        f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && "
        f"ros2 launch ur5_bringup ur5_bringup.launch.py "
        f"headless:=true rviz:=false controller_mode:=effort simulated_grasp:=true "
        f"controllers_file:={ROOT}/install/ur5_adaptive_control/share/ur5_adaptive_control/config/adaptive_controllers.yaml "
        f"world:={ROOT}/install/ur5_bringup/share/ur5_bringup/worlds/ur5_pick_place_1_0kg.sdf"
    )
    proc = subprocess.Popen(
        ["bash", "-c", launch_cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    try:
        t0 = time.monotonic()
        ready = False
        while time.monotonic() - t0 < 45.0:
            res = subprocess.run(
                ["bash", "-c", "source /opt/ros/humble/setup.bash && ros2 control list_controllers"],
                capture_output=True,
                text=True,
            )
            if "arm_controller" in res.stdout and "active" in res.stdout:
                ready = True
                break
            time.sleep(3.5)
        if not ready:
            print("ERROR: arm_controller failed to activate for progressive suite!")
            return False

        prog_cmd = (
            f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && "
            f"python3 {ROOT}/ur5_adaptive_control/scripts/test_progressive_phase8.py"
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


def run_single_trial(cfg: dict) -> bool:
    trial_name = cfg["name"]
    mass = cfg["mass"]
    world_file = cfg["world_file"]
    mode = cfg["mode"]
    dyn_ff = cfg["dynamic_ff"]
    gain_sched = cfg["gain_sched"]
    v_scale = cfg["v_scale"]
    a_scale = cfg["a_scale"]
    fallback = cfg["fallback"]
    desc = cfg["desc"]

    out_dir = EXP_BASE / trial_name
    out_dir.mkdir(parents=True, exist_ok=True)

    print("\n" + "=" * 80)
    print(f"   STARTING PHASE 8 TRIAL: {trial_name}")
    print(f"   Description: {desc}")
    print(f"   World: {world_file} | Mass: {mass} kg | Mode: {mode}")
    print(f"   Dynamic FF: {dyn_ff} | Gain Sched: {gain_sched}")
    print(f"   Scaling: v={v_scale}, a={a_scale}, fallback={fallback}")
    print(f"   Output Directory: {out_dir}")
    print("=" * 80 + "\n")

    kill_gazebo_processes()

    # 1. Launch Gazebo simulation
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
        while time.monotonic() - t0 < 45.0:
            res = subprocess.run(
                ["bash", "-c", "source /opt/ros/humble/setup.bash && ros2 control list_controllers"],
                capture_output=True,
                text=True,
            )
            if "arm_controller" in res.stdout and "active" in res.stdout:
                controllers_ready = True
                break
            time.sleep(3.5)

        if not controllers_ready:
            print("ERROR: Timeout waiting for arm_controller to activate!")
            return False

        print("[2/5] Controllers active. Waiting for robot quiescence (< 0.01 rad/s)...")
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
                print("  -> Robot quiescent!")
                break
            time.sleep(3.5)

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

        # 3. Build orchestrator command with Phase 8 supervisor options
        scale_args = "--scaling-enabled true"
        if fallback:
            scale_args += " --force-fallback true"
        elif v_scale is not None and a_scale is not None:
            scale_args += f" --velocity-scale {v_scale} --acceleration-scale {a_scale}"

        print(f"[4/5] Executing pick-and-place replay path01 with supervisor scaling ({scale_args})...")
        orch_cmd = (
            f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && "
            f"python3 {ROOT}/ur5_bringup/scripts/pick_place_trajectory_orchestrator.py "
            f"--path {ROOT}/path01 --mode effort --controller-name adaptive_arm_controller "
            f"--payload-kg {mass} --output-dir {out_dir} {scale_args} --auto-exit true"
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
        scaling_info = rdata.get("scaling_supervision", {})
        print(f"[5/5] Trial {trial_name} completed. Status: {status}")
        print(f"  -> Max EE Error: {metrics.get('ee_max_mm', 0.0):.3f} mm")
        print(f"  -> RMSE EE:     {metrics.get('ee_rmse_mm', 0.0):.3f} mm")
        print(f"  -> Settled EE:  {metrics.get('ee_final_settled_mm', 0.0):.3f} mm")
        print(f"  -> Jitter W3:   {metrics.get('wrist_3_jitter_rad_s', 0.0):.4f} rad/s")
        if "pick_to_place" in scaling_info:
            p2p_info = scaling_info["pick_to_place"]
            print(f"  -> Scaling Info (pick_to_place): v={p2p_info['velocity_scale']}, a={p2p_info['acceleration_scale']}, dur={p2p_info['duration_s']:.2f}s")
            print(f"  -> Scaling Reason: {p2p_info['reason']}")

        return status == "passed"
    finally:
        print("[CLEANUP] Stopping simulation processes...")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
        kill_gazebo_processes()


def main():
    parser = argparse.ArgumentParser(description="UR5 Phase 8 Experiment Runner")
    parser.add_argument("--skip-progressive", action="store_true", help="Skip Stage 1 progressive tests")
    parser.add_argument("--progressive-only", action="store_true", help="Run only Stage 1 progressive tests")
    parser.add_argument("--trial", default=None, help="Run only a specific trial by name")
    args = parser.parse_args()

    # 1. Run progressive safety test suite
    if args.progressive_only:
        ok = run_progressive_suite()
        return 0 if ok else 1

    if not args.skip_progressive and args.trial is None:
        if not run_progressive_suite():
            print("HALTING: Progressive safety suite failed!")
            return 1

    # 2. Run trials
    results = {}
    trials_to_run = [t for t in TRIALS if args.trial is None or t["name"] == args.trial]
    for cfg in trials_to_run:
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
    print("           UR5 PHASE 8 EXPERIMENTAL SUITE SUMMARY")
    print("=" * 80)
    all_ok = True
    for name, ok in results.items():
        status_str = "PASS" if ok else "FAIL"
        print(f"  {name:38s}: {status_str}")
        if not ok:
            all_ok = False
    print("=" * 80)
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())

