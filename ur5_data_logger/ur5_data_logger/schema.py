JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

METADATA_COLUMNS = (
    "time_sec",
    "trajectory_id",
    "controller_mode",
    "payload_kg",
)

JOINT_FIELDS = (
    "reference_position_rad",
    "actual_position_rad",
    "position_error_rad",
    "reference_velocity_rad_s",
    "actual_velocity_rad_s",
    "commanded_effort_Nm",
    "measured_effort_Nm",
    "Kp",
    "Ki",
    "Kd",
)

EE_COLUMNS = (
    "reference_ee_x_m",
    "reference_ee_y_m",
    "reference_ee_z_m",
    "actual_ee_x_m",
    "actual_ee_y_m",
    "actual_ee_z_m",
    "ee_error_x_mm",
    "ee_error_y_mm",
    "ee_error_z_mm",
    "ee_error_3d_mm",
)

CSV_COLUMNS = METADATA_COLUMNS + tuple(
    f"{joint}__{field}" for field in JOINT_FIELDS for joint in JOINTS
) + EE_COLUMNS

METRIC_COLUMNS = (
    "joint",
    "sample_count",
    "position_rmse_rad",
    "max_abs_position_error_rad",
    "iae_rad_s",
    "ise_rad2_s",
    "cycle_duration_sec",
)
