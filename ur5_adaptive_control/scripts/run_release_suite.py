#!/usr/bin/env python3
"""Automated runner for UR5 Release Isolation Tests R1, R2, R3, R4."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path("/home/shiva/ur5/ur5_ws")
EXP_BASE = ROOT / "experiment_logs" / "release_diagnosis"


def kill_gazebo_processes():
    subprocess.run(
        ["pkill", "-u", "shiva", "-9", "-f", "gz_sim|ign_gazebo|controller_manager|ros_gz_bridge|pick_place_trajectory|diagnose_release"],
        capture_output=True,
    )
    time.sleep(3.5)


def run_single_case(case: str, at_place: bool, folder_name: str, mass: float = 3.0) -> bool:
    print("\n" + "=" * 80)
    print(f"      EXECUTING RELEASE ISOLATION TRIAL: {case} -> {folder_name}")
    print(f"      Payload: {mass} kg | at_place: {at_place}")
    print("=" * 80 + "\n")

    kill_gazebo_processes()
    out_dir = EXP_BASE / folder_name
    out_dir.mkdir(parents=True, exist_ok=True)

    world_file = "ur5_pick_place_3_0kg.sdf" if mass >= 3.0 else "ur5_pick_place.sdf"
    launch_cmd = (
        f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && "
        f"ros2 launch ur5_bringup ur5_bringup.launch.py "
        f"headless:=true rviz:=false controller_mode:=effort simulated_grasp:=true "
        f"controllers_file:={ROOT}/install/ur5_adaptive_control/share/ur5_adaptive_control/config/adaptive_controllers.yaml "
        f"world:={ROOT}/install/ur5_bringup/share/ur5_bringup/worlds/{world_file}"
    )

    print(f"[1/4] Launching Gazebo simulation with {world_file}...")
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
            time.sleep(3.0)

        if not controllers_ready:
            print("ERROR: Timeout waiting for arm_controller to activate!")
            return False

        print("[2/4] Arm controller active. Verifying quiescence (<0.01 rad/s)...")
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

        print("[3/4] Switching to adaptive_arm_controller...")
        switch_cmd = (
            f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && "
            f"python3 {ROOT}/ur5_adaptive_control/scripts/controller_switcher.py switch adaptive && "
            f"python3 {ROOT}/ur5_adaptive_control/scripts/controller_switcher.py mode adaptive_enabled && "
            f"ros2 param set /adaptive_arm_controller payload_feedforward_enabled true && "
            f"ros2 param set /adaptive_arm_controller payload_nominal_mass {mass}"
        )
        switch_ok = False
        for attempt in range(8):
            res_switch = subprocess.run(["bash", "-c", switch_cmd], capture_output=True, text=True)
            if res_switch.returncode == 0:
                switch_ok = True
                print(f"  -> Controller switched to adaptive_arm_controller successfully!")
                break
            print(f"  Attempt {attempt + 1}: Switch retry, waiting 3.0s...")
            time.sleep(3.0)

        if not switch_ok:
            print(f"ERROR: Switch to adaptive controller failed:\n{res_switch.stderr}")
            return False
        time.sleep(1.0)

        print(f"[4/4] Executing diagnose_release_isolates {case}...")
        diag_args = f"{case} --out {out_dir}"
        if at_place:
            diag_args += " --at-place"
        diag_cmd = (
            f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && "
            f"python3 {ROOT}/ur5_adaptive_control/scripts/diagnose_release_isolates.py {diag_args}"
        )
        diag_proc = subprocess.run(["bash", "-c", diag_cmd], capture_output=True, text=True)
        print(diag_proc.stdout)
        if diag_proc.returncode != 0:
            print(f"ERROR: diagnose_release_isolates failed for {case}:\n{diag_proc.stderr}")
            return False

        return True
    finally:
        print("[CLEANUP] Stopping simulation processes...")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
        kill_gazebo_processes()


def main():
    parser = argparse.ArgumentParser(description="UR5 Release Suite Runner")
    parser.add_argument("--case", choices=["R1", "R2", "R3", "R4", "all"], default="all")
    args = parser.parse_args()

    suite = [
        ("R1", True, "R1_empty_gripper_place"),
        ("R2", True, "R2_feedforward_place"),
        ("R3", False, "R3_detach_closed"),
        ("R4", False, "R4_complete_release"),
    ]

    selected = suite if args.case == "all" else [s for s in suite if s[0] == args.case]
    success = {}
    for case, at_place, folder in selected:
        ok = run_single_case(case, at_place, folder)
        success[case] = ok
        if not ok:
            print(f"ABORTING: Case {case} failed!")
            break

    # Generate summary report and plots
    print("\n" + "=" * 80)
    print("      GENERATING RELEASE DIAGNOSIS REPORT AND TIMING PLOTS")
    print("=" * 80 + "\n")
    rep_cmd = (
        f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && "
        f"python3 {ROOT}/ur5_adaptive_control/scripts/report_release_isolates.py"
    )
    subprocess.run(["bash", "-c", rep_cmd])

    print("\nRelease Suite Results:")
    for c, ok in success.items():
        print(f"  {c}: {'PASS' if ok else 'FAIL'}")

    return 0 if all(success.values()) else 1


if __name__ == "__main__":
    sys.exit(main())

