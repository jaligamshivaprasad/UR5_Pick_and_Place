# UR5 trajectory Excel analysis

This folder contains an offline, read-only CSV-to-XLSX reporting tool. It does
not import ROS or modify nodes, controllers, configurations, logs, or the input
CSV.

## Generate a report

```bash
python3 reporting/generate_trajectory_excel.py \
  --input experiment_logs/20261028_065619_pick_place_001/joint_trajectory_log.csv \
  --output experiment_logs/20261028_065619_pick_place_001/joint_trajectory_analysis.xlsx
```

An existing report is never replaced unless `--overwrite` is supplied
explicitly. With `--overwrite`, the current workbook is copied to a timestamped
`joint_trajectory_analysis.backup_<UTC timestamp>.xlsx` file before replacement.
Output is written to a temporary workbook, checked, and atomically moved into
place. The source CSV's byte size and SHA-256 checksum are checked before and
after generation.

The workbook contains `Dashboard`, enlarged `Joint_Tracking`, `Joint_Errors`,
`EE_Analysis`, and `Velocity_Analysis` sheets, plus the preserved
`Raw_Data`, `Calculated_Data`, `Metrics`, `Run_Info`, and `Comparisons` sheets.
The dashboard is an overview; the dedicated tabs provide larger research-grade
charts with readable time axes, units, peak timestamps, and separate velocity
and error axes. Native Excel/Calc charts are used except for the clearly labeled
static 3D trajectory views. Metrics use full-resolution data; missing payload,
effort, or gain observations remain blank/N/A.

## Rollback

Delete only these reporting artifacts if they are no longer wanted:

```text
reporting/generate_trajectory_excel.py
reporting/README.md
experiment_logs/<run>/joint_trajectory_analysis.xlsx
```

No ROS package or runtime configuration is part of this tool.
