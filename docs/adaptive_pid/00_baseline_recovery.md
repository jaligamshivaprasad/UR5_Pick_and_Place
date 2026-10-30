# UR5 Baseline Recovery Instructions

**Reference Commit:** `2bea65474119cc802260655b2d431e8d2ee7530b`  
**Reference Tag:** `v1.4.0-fixed-pid` (`34c82e6ba2e12cd94326f293551396ede16387c0`)  
**Workspace:** `/home/shiva/ur5/ur5_ws`

---

## 1. Safety Rules During Research

1. **Do Not Overwrite Baseline Configuration:**
   - Keep `ur5_moveit_config/config/ros2_controllers_position.yaml` unchanged.
   - Keep `ur5_moveit_config/config/ros2_controllers_effort.yaml` unchanged (this serves as the calibrated fixed-PID baseline C1).
   - Any experimental adaptive controller YAML should be named `ros2_controllers_adaptive.yaml` or placed inside a dedicated package.
2. **Do Not Reset or Discard Untracked Git Files Without Confirmation:**
   - The user has untracked xlsx backups and working SDF changes in `ur5_bringup/worlds/ur5_pick_place.sdf`.
3. **Restoring the Clean Fixed-PID Baseline:**
   If at any point simulation behavior diverges or controllers fail, restore the verified baseline world and controllers.

---

## 2. Baseline Launch Commands

### Verification of Fixed-PID Position Baseline (C0)
```bash
source /opt/ros/humble/setup.bash
source /home/shiva/ur5/ur5_ws/install/setup.bash
ros2 launch ur5_bringup ur5_bringup.launch.py controller_mode:=position
```

### Verification of Calibrated Fixed-PID Effort Baseline (C1)
```bash
source /opt/ros/humble/setup.bash
source /home/shiva/ur5/ur5_ws/install/setup.bash
# Full pick-and-place replay with automated logging:
ros2 launch ur5_bringup ur5_pick_place_optimized.launch.py
```

### Clean Gazebo & Process Teardown
If any hung simulation processes remain:
```bash
pkill -9 -f "ign gazebo"
pkill -9 -f "gz_sim"
pkill -9 -f "ros2"
pkill -9 -f "robot_state_publisher"
pkill -9 -f "move_group"
pkill -9 -f "pick_place"
```

---

## 3. Configuration Checksums (Baseline Verification)

- `ur5_moveit_config/config/ros2_controllers_effort.yaml`:
  `sha256`: `ad7ba55a159938b8ca81313eb287e0b51cb59e5572688f6e80b2a59a9366df0e`
- `ur5_moveit_config/config/ros2_controllers_position.yaml`:
  `sha256`: `d2a7bc726ff5857217db551ce27a8efd38867cce6945805bf54a558dc0bf9fe4`
- `path01`:
  `sha256`: `667d4142153beacacd946612571b9cad54c6c060f1630b32ac6873255ed50f1e`
- `ur5_bringup/launch/ur5_pick_place_optimized.launch.py`:
  `sha256`: `434fec5bcf6bcda73fe91aeead09f987e9e7b2339d67562092dd9d18e5b60287`

