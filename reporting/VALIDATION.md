# Validation summary — `pick_place_001`

Source: `experiment_logs/20261028_065619_pick_place_001/joint_trajectory_log.csv`

- Source size: 8,446,822 bytes
- Source SHA-256: `03f0848e5970e6ca18655a11e3e2ec15cc98ee89543b0c344cd3da4aaffc8efa`
- Source before/after checksum and size: unchanged
- Rows / columns: 9,256 / 74
- Timestamps: monotonic; logged span 194.380 s; median positive interval 0.020000 s
- EE 3D error: RMSE 8.783108 mm; maximum 61.683473 mm; final logged sample 0.000000 mm
- EE 3D mean / population standard deviation: 1.788598 / 8.599064 mm
- Joint RMSE / maximum absolute error (deg):
  - J1: 0.317681 / 2.073921
  - J2: 0.305939 / 1.928168
  - J3: 0.406177 / 2.533763
  - J4: 0.127546 / 1.684609
  - J5: 0.539633 / 3.603018
  - J6: 0.297264 / 1.706654
- Payload, commanded effort, measured effort, and Kp/Ki/Kd: no valid observations; represented as N/A / Not recorded
- Workbook ZIP integrity: passed
- Required sheets: passed (`Dashboard`, `Joint_Tracking`, `Joint_Errors`, `EE_Analysis`, `Velocity_Analysis`, `Raw_Data`, `Calculated_Data`, `Metrics`, `Run_Info`, `Comparisons`)
- Dashboard: all nine labeled sections; 17 native charts plus one embedded static 3D plot
- Enlarged analysis tabs: 6 joint tracking charts, 6 joint error charts, 4 EE charts, 6 velocity/error charts
- LibreOffice compatibility: workbook opened and converted successfully in a temporary directory
- Existing-output safety: generation refused without explicit `--overwrite`
- Backups created before replacement: timestamped `joint_trajectory_analysis.backup_<UTC timestamp>.xlsx` files (including `joint_trajectory_analysis.backup_20261028T022200Z.xlsx`)

All values above were recomputed independently from the CSV with Python's
standard `csv`, `math`, and `statistics` modules. No ROS process, controller,
configuration, logger source, or source CSV was modified.
