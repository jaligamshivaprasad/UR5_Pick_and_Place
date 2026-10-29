"""Dash application callbacks connecting ROS monitoring, historical CSVs, and UI rendering."""

import math
from pathlib import Path
from typing import Any, Dict, List, Optional

from dash import Input, Output, State, callback_context, dcc, html
import numpy as np

from .components import (
    build_3d_path_figure,
    build_ee_cartesian_figure,
    build_effort_figure,
    build_joint_tracking_figure,
    build_joint_velocity_figure,
    create_kpi_card,
    create_status_pill,
)
from .config import (
    COLORS,
    JOINT_LABELS,
    JOINT_SHORT_LABELS,
    JOINTS,
    load_dashboard_settings,
    save_dashboard_settings,
)
from .data_manager import DataManager
from .ros_monitor import ROSMonitor


def register_callbacks(app, ros_monitor: ROSMonitor, data_manager: DataManager):
    """Register all Dash callbacks on the Dash application instance."""

    # 1. Interval rate callback
    @app.callback(
        [Output("live-interval", "interval"), Output("live-interval", "disabled")],
        [Input("dropdown-refresh-rate", "value")],
    )
    def update_interval_rate(val):
        if val is None or val == 0:
            return 1000, True
        return val, False

    # 2. Toggle all joints callback
    @app.callback(
        Output("checklist-joints", "value"),
        [Input("btn-select-all-joints", "n_clicks")],
        [State("checklist-joints", "value")],
    )
    def toggle_all_joints(n_clicks, current_val):
        if n_clicks is None:
            return current_val or JOINTS
        if len(current_val or []) == len(JOINTS):
            return [JOINTS[0]]
        return JOINTS

    # Cache for sidebar run info text to avoid redundant CSV reads
    _sidebar_info_cache = {}

    # 3. Header Status Badges Callback
    @app.callback(
        [
            Output("badge-ros-status", "children"),
            Output("badge-sim-time", "children"),
            Output("badge-controller-rate", "children"),
            Output("badge-fk-status", "children"),
            Output("sidebar-run-info", "children"),
        ],
        [
            Input("live-interval", "n_intervals"),
            Input("radio-data-source", "value"),
            Input("dropdown-run-selector", "value"),
        ],
    )
    def update_header_badges(n_intervals, data_source, selected_run_path):
        snapshot = ros_monitor.get_snapshot()
        diag = snapshot["diagnostics"]

        # ROS status badge
        if not diag["is_ros_available"]:
            ros_pill = create_status_pill("ROS 2", "Not Installed", "danger")
        elif not diag["is_ros_initialized"]:
            ros_pill = create_status_pill("ROS 2", "Offline", "warn")
        elif diag["last_controller_seen_ago"] < 3.0:
            ros_pill = create_status_pill("ROS 2", "Active (Streaming)", "ok")
        else:
            ros_pill = create_status_pill("ROS 2", "Idle / Waiting", "warn")

        # Sim time badge
        sim_time = diag["sim_time"]
        sim_pill = create_status_pill("Sim Time", f"{sim_time:.2f} s", "info")

        # Controller rate badge
        rate_hz = diag["rates_hz"].get("controller_state", 0.0)
        rate_state = "ok" if rate_hz > 10.0 else ("warn" if rate_hz > 0.0 else "muted")
        rate_pill = create_status_pill("Controller", f"{rate_hz:.1f} Hz", rate_state)

        # FK status badge
        fk_status = diag["fk_status"]
        fk_state = "ok" if "Verified" in fk_status else "warn"
        fk_pill = create_status_pill("Kinematics", fk_status.split(":")[0], fk_state)

        # Sidebar run info text (cached for fast response)
        run_info_text = ""
        if data_source == "historical" and selected_run_path:
            if selected_run_path in _sidebar_info_cache:
                run_info_text = _sidebar_info_cache[selected_run_path]
            else:
                try:
                    run_data = data_manager.load_run(selected_run_path)
                    m = run_data["metrics"]["cycle"]
                    run_info_text = (
                        f"Run ID: {run_data['run_id']}\n"
                        f"Samples: {m['total_samples']:,} rows\n"
                        f"Duration: {m['logged_duration_sec']:.1f} s (Active: {m['active_duration_sec']:.1f} s)\n"
                        f"Mode: {run_data['info'].get('controller_mode', 'baseline_position')}"
                    )
                    _sidebar_info_cache[selected_run_path] = run_info_text
                except Exception:
                    run_info_text = "Error reading run metadata"
        else:
            run_info_text = (
                f"Live Buffer: {diag['buffer_count']} / {diag['buffer_capacity']} samples\n"
                f"Controller: {diag['controller_mode']}\n"
                f"Effort: {diag['effort_status_note'][:32]}..."
            )

        return ros_pill, sim_pill, rate_pill, fk_pill, run_info_text

    # 4. Main Tab Content & KPI Rendering Callback
    @app.callback(
        [Output("kpi-summary-grid", "children"), Output("tab-content", "children")],
        [
            Input("live-interval", "n_intervals"),
            Input("dashboard-tabs", "value"),
            Input("radio-data-source", "value"),
            Input("dropdown-run-selector", "value"),
            Input("dropdown-overlay-selector", "value"),
            Input("checklist-joints", "value"),
        ],
    )
    def update_main_content(n_intervals, active_tab, data_source, run_path, overlay_path, selected_joints):
        import dash

        # Safely determine trigger without raising MissingCallbackContextException
        triggered_id = None
        try:
            if callback_context and callback_context.triggered:
                prop_id = callback_context.triggered[0].get("prop_id", "")
                triggered_id = prop_id.split(".")[0]
        except Exception:
            triggered_id = None

        # CRITICAL PERFORMANCE GUARD:
        # When viewing historical data, ignore periodic timer ticks!
        # Historical data does not change dynamically, so re-rendering it every 500ms
        # needlessly floods the browser and blocks the event loop.
        if data_source == "historical" and triggered_id == "live-interval":
            return dash.no_update, dash.no_update

        selected_joints = selected_joints or []

        # -------------------------------------------------------------
        # Extract primary data (Live or Historical)
        # -------------------------------------------------------------
        is_live = (data_source == "live")
        time_series = np.array([])
        ref_positions = {j: np.array([]) for j in JOINTS}
        act_positions = {j: np.array([]) for j in JOINTS}
        err_positions = {j: np.array([]) for j in JOINTS}
        ref_velocities = {j: np.array([]) for j in JOINTS}
        act_velocities = {j: np.array([]) for j in JOINTS}
        effort_values = {j: np.array([]) for j in JOINTS}
        ref_xyz = np.empty((0, 3))
        act_xyz = np.empty((0, 3))
        ee_3d_err_mm = np.array([])
        ee_available = False
        effort_available = False
        run_metrics = None
        run_info = {}

        if is_live:
            snapshot = ros_monitor.get_snapshot()
            records = snapshot["data"]
            effort_available = snapshot["diagnostics"]["effort_available"]
            if records:
                time_series = np.array([r["time_sec"] for r in records])
                for j in JOINTS:
                    ref_positions[j] = np.array([r["ref_pos"].get(j, np.nan) for r in records])
                    act_positions[j] = np.array([r["act_pos"].get(j, np.nan) for r in records])
                    err_positions[j] = np.array([r["err_pos"].get(j, np.nan) for r in records])
                    ref_velocities[j] = np.array([r["ref_vel"].get(j, np.nan) for r in records])
                    act_velocities[j] = np.array([r["act_vel"].get(j, np.nan) for r in records])
                    effort_values[j] = np.array([r["meas_eff"].get(j, np.nan) for r in records])

                # End effector live
                r_xyz = [r["ee_ref_xyz"] for r in records if math.isfinite(r["ee_ref_xyz"][0])]
                a_xyz = [r["ee_act_xyz"] for r in records if math.isfinite(r["ee_act_xyz"][0])]
                if r_xyz and len(r_xyz) == len(records):
                    ref_xyz = np.array([r["ee_ref_xyz"] for r in records])
                    act_xyz = np.array([r["ee_act_xyz"] for r in records])
                    ee_3d_err_mm = np.array([r["ee_3d_err_mm"] for r in records])
                    ee_available = True

                # Fast vectorized KPI calculation for live buffer (<1ms, no heavy pandas)
                err_abs_all = [np.abs(err_positions[j]) for j in JOINTS if len(err_positions[j]) > 0]
                worst_max_rad = float(np.nanmax([np.nanmax(ea) for ea in err_abs_all])) if err_abs_all else 0.0
                worst_rmse_rad = float(np.nanmax([np.sqrt(np.nanmean(ea**2)) for ea in err_abs_all])) if err_abs_all else 0.0

                iae_vals = []
                for j in JOINTS:
                    if len(err_positions[j]) > 1:
                        iae_vals.append(float(np.trapz(np.abs(err_positions[j]), time_series)))
                worst_iae = max(iae_vals) if iae_vals else 0.0

                t_duration = float(time_series[-1] - time_series[0]) if len(time_series) > 1 else 0.0
                act_vel_mag = np.zeros(len(time_series))
                for j in JOINTS:
                    if len(act_velocities[j]) == len(time_series):
                        act_vel_mag += np.nan_to_num(np.abs(act_velocities[j]))
                active_pts = np.sum(act_vel_mag > 0.02)
                dt_avg = t_duration / max(1, len(time_series) - 1)
                active_sec = float(active_pts * dt_avg)

                ee_max = float(np.nanmax(ee_3d_err_mm)) if ee_available and len(ee_3d_err_mm) > 0 else 0.0
                ee_rmse = float(np.sqrt(np.nanmean(ee_3d_err_mm**2))) if ee_available and len(ee_3d_err_mm) > 0 else 0.0

                run_metrics = {
                    "summary": {
                        "worst_max_err_rad": worst_max_rad,
                        "worst_max_err_deg": float(np.degrees(worst_max_rad)),
                        "worst_rmse_rad": worst_rmse_rad,
                        "worst_rmse_deg": float(np.degrees(worst_rmse_rad)),
                        "worst_iae_rad_s": worst_iae,
                    },
                    "cycle": {
                        "total_samples": len(time_series),
                        "logged_duration_sec": t_duration,
                        "active_duration_sec": active_sec,
                    },
                    "ee": {
                        "available": ee_available,
                        "max_3d_mm": ee_max,
                        "rmse_3d_mm": ee_rmse,
                    },
                }
        else:
            if run_path:
                try:
                    loaded = data_manager.load_run(run_path)
                    df = loaded["df"]
                    run_metrics = loaded["metrics"]
                    run_info = loaded["info"]
                    time_series = df["time_sec"].to_numpy()
                    for j in JOINTS:
                        ref_positions[j] = df[f"{j}__reference_position_rad"].to_numpy()
                        act_positions[j] = df[f"{j}__actual_position_rad"].to_numpy()
                        err_positions[j] = df[f"{j}__position_error_rad"].to_numpy()
                        if f"{j}__reference_velocity_rad_s" in df.columns:
                            ref_velocities[j] = df[f"{j}__reference_velocity_rad_s"].to_numpy()
                        if f"{j}__actual_velocity_rad_s" in df.columns:
                            act_velocities[j] = df[f"{j}__actual_velocity_rad_s"].to_numpy()
                        if f"{j}__measured_effort_Nm" in df.columns:
                            effort_values[j] = df[f"{j}__measured_effort_Nm"].to_numpy()

                    # Check effort availability
                    effort_available = any(np.any(np.isfinite(effort_values[j])) for j in JOINTS)

                    # EE columns
                    if "reference_ee_x_m" in df.columns and "actual_ee_x_m" in df.columns:
                        ref_xyz = df[["reference_ee_x_m", "reference_ee_y_m", "reference_ee_z_m"]].to_numpy()
                        act_xyz = df[["actual_ee_x_m", "actual_ee_y_m", "actual_ee_z_m"]].to_numpy()
                        ee_3d_err_mm = df["ee_error_3d_mm"].to_numpy()
                        ee_available = np.any(np.isfinite(ee_3d_err_mm))
                except Exception:
                    pass

        # -------------------------------------------------------------
        # Extract overlay data if requested
        # -------------------------------------------------------------
        overlay_time = None
        overlay_act = {j: None for j in JOINTS}
        overlay_name = None
        if overlay_path and overlay_path != "none":
            try:
                overlay_data = data_manager.load_run(overlay_path)
                df_ov = overlay_data["df"]
                overlay_time = df_ov["time_sec"].to_numpy()
                overlay_name = overlay_data["run_id"]
                for j in JOINTS:
                    overlay_act[j] = df_ov[f"{j}__actual_position_rad"].to_numpy()
            except Exception:
                pass

        # -------------------------------------------------------------
        # Render Executive KPI Summary Cards
        # -------------------------------------------------------------
        kpi_cards = []
        if run_metrics:
            summary = run_metrics["summary"]
            cycle = run_metrics["cycle"]
            ee_m = run_metrics["ee"]

            # Max Error
            max_err_deg = summary["worst_max_err_deg"]
            max_err_rad = summary["worst_max_err_rad"]
            is_err_alert = max_err_deg > 3.0
            kpi_cards.append(
                create_kpi_card(
                    "Worst-Case Max Error",
                    f"{max_err_deg:.2f}°",
                    f"{max_err_rad:.4f} rad",
                    COLORS["error"],
                    alert=is_err_alert,
                    card_id="kpi-card-max-error",
                )
            )

            # Worst RMSE
            rmse_deg = summary["worst_rmse_deg"]
            rmse_rad = summary["worst_rmse_rad"]
            kpi_cards.append(
                create_kpi_card(
                    "Worst-Case RMSE",
                    f"{rmse_deg:.2f}°",
                    f"{rmse_rad:.4f} rad",
                    COLORS["amber"],
                    card_id="kpi-card-rmse",
                )
            )

            # Worst IAE
            iae_val = summary["worst_iae_rad_s"]
            kpi_cards.append(
                create_kpi_card(
                    "Integral Abs Error (IAE)",
                    f"{iae_val:.3f}",
                    "rad · s",
                    COLORS["accent"],
                    card_id="kpi-card-iae",
                )
            )

            # End-Effector 3D Error
            if ee_m["available"]:
                kpi_cards.append(
                    create_kpi_card(
                        "Max EE 3D Error",
                        f"{ee_m['max_3d_mm']:.2f} mm",
                        f"RMSE: {ee_m['rmse_3d_mm']:.2f} mm",
                        COLORS["cyan_ref"],
                        card_id="kpi-card-ee",
                    )
                )
            else:
                kpi_cards.append(
                    create_kpi_card(
                        "EE 3D Accuracy",
                        "N/A",
                        "Awaiting Verified URDF",
                        COLORS["text_muted"],
                        card_id="kpi-card-ee",
                    )
                )

            # Cycle Durations
            kpi_cards.append(
                create_kpi_card(
                    "Cycle Duration",
                    f"{cycle['logged_duration_sec']:.1f} s",
                    f"Active Motion: {cycle['active_duration_sec']:.1f} s",
                    COLORS["success"],
                    card_id="kpi-card-cycle",
                )
            )
        else:
            kpi_cards = [
                create_kpi_card("Worst-Case Max Error", "--", "Waiting for data", card_id="kpi-card-max-error"),
                create_kpi_card("Worst-Case RMSE", "--", "Waiting for data", card_id="kpi-card-rmse"),
                create_kpi_card("IAE Metric", "--", "Waiting for data", card_id="kpi-card-iae"),
                create_kpi_card("EE 3D Error", "--", "Waiting for data", card_id="kpi-card-ee"),
                create_kpi_card("Cycle Duration", "--", "Waiting for data", card_id="kpi-card-cycle"),
            ]

        # -------------------------------------------------------------
        # Decimate series specifically for visual charts (<350 pts live, <1500 historical)
        # Keeps network payload ultra-light (<70 KB) and rendering silky-smooth
        # -------------------------------------------------------------
        max_plot_points = 350 if is_live else 1500
        if len(time_series) > max_plot_points:
            c_idx = np.linspace(0, len(time_series) - 1, max_plot_points, dtype=int)
            time_plot = time_series[c_idx]
            ref_pos_plot = {j: ref_positions[j][c_idx] for j in JOINTS}
            act_pos_plot = {j: act_positions[j][c_idx] for j in JOINTS}
            err_pos_plot = {j: err_positions[j][c_idx] for j in JOINTS}
            ref_vel_plot = {j: ref_velocities[j][c_idx] for j in JOINTS}
            act_vel_plot = {j: act_velocities[j][c_idx] for j in JOINTS}
            effort_plot = {j: effort_values[j][c_idx] for j in JOINTS}
            ref_xyz_plot = ref_xyz[c_idx] if len(ref_xyz) == len(time_series) else ref_xyz
            act_xyz_plot = act_xyz[c_idx] if len(act_xyz) == len(time_series) else act_xyz
            ee_3d_err_plot = ee_3d_err_mm[c_idx] if len(ee_3d_err_mm) == len(time_series) else ee_3d_err_mm
        else:
            time_plot = time_series
            ref_pos_plot = ref_positions
            act_pos_plot = act_positions
            err_pos_plot = err_positions
            ref_vel_plot = ref_velocities
            act_vel_plot = act_velocities
            effort_plot = effort_values
            ref_xyz_plot = ref_xyz
            act_xyz_plot = act_xyz
            ee_3d_err_plot = ee_3d_err_mm

        if overlay_time is not None and len(overlay_time) > max_plot_points:
            ov_idx = np.linspace(0, len(overlay_time) - 1, max_plot_points, dtype=int)
            overlay_time_plot = overlay_time[ov_idx]
            overlay_act_plot = {j: overlay_act[j][ov_idx] if overlay_act[j] is not None else None for j in JOINTS}
        else:
            overlay_time_plot = overlay_time
            overlay_act_plot = overlay_act

        # -------------------------------------------------------------
        # Render Active Tab Content
        # -------------------------------------------------------------
        content = html.Div()

        if active_tab == "tab-tracking":
            # Tab 1: Joint Tracking J1-J6
            if len(time_plot) == 0:
                content = html.Div(
                    id="tracking-placeholder",
                    style={"padding": "40px", "textAlign": "center", "color": COLORS["text_secondary"], "minHeight": "400px"},
                    children="Waiting for data stream or historical log selection...",
                )
            else:
                charts = []
                for j in selected_joints:
                    fig = build_joint_tracking_figure(
                        joint=j,
                        time_series=time_plot,
                        ref_pos=ref_pos_plot[j],
                        act_pos=act_pos_plot[j],
                        err_pos=err_pos_plot[j],
                        overlay_time=overlay_time_plot,
                        overlay_act=overlay_act_plot.get(j),
                        overlay_name=overlay_name,
                    )
                    charts.append(
                        html.Div(
                            id=f"card-tracking-{j}",
                            className="dashboard-card",
                            style={"marginBottom": "16px", "minHeight": "380px"},
                            children=[
                                dcc.Graph(
                                    id=f"graph-tracking-{j}",
                                    figure=fig,
                                    config={"responsive": True, "displayModeBar": True},
                                )
                            ],
                        )
                    )
                content = html.Div(id="tracking-charts-container", children=charts if charts else "No joints selected. Check at least one joint in the sidebar.")

        elif active_tab == "tab-velocities":
            # Tab 2: Velocities and Efforts
            if len(time_plot) == 0:
                content = html.Div(
                    id="velocities-placeholder",
                    style={"padding": "40px", "textAlign": "center", "color": COLORS["text_secondary"], "minHeight": "400px"},
                    children="Waiting for data stream or historical log selection...",
                )
            else:
                charts = []
                for j in selected_joints:
                    fig_vel = build_joint_velocity_figure(
                        joint=j,
                        time_series=time_plot,
                        ref_vel=ref_vel_plot[j],
                        act_vel=act_vel_plot[j],
                    )
                    charts.append(
                        html.Div(
                            id=f"card-velocity-{j}",
                            className="dashboard-card",
                            style={"marginBottom": "16px", "minHeight": "320px"},
                            children=[
                                dcc.Graph(
                                    id=f"graph-velocity-{j}",
                                    figure=fig_vel,
                                    config={"responsive": True},
                                )
                            ],
                        )
                    )

                # Add Effort Figure
                fig_eff = build_effort_figure(time_plot, effort_plot, effort_available)
                charts.append(
                    html.Div(
                        id="card-effort",
                        className="dashboard-card",
                        style={"marginTop": "20px", "minHeight": "340px"},
                        children=[
                            dcc.Graph(
                                id="graph-effort",
                                figure=fig_eff,
                                config={"responsive": True},
                            )
                        ],
                    )
                )
                content = html.Div(id="velocities-charts-container", children=charts)

        elif active_tab == "tab-cartesian":
            # Tab 3: Cartesian & 3D EE Tracking
            if len(time_plot) == 0:
                content = html.Div(
                    id="cartesian-placeholder",
                    style={"padding": "40px", "textAlign": "center", "color": COLORS["text_secondary"], "minHeight": "400px"},
                    children="Waiting for data stream or historical log selection...",
                )
            else:
                fig_cart = build_ee_cartesian_figure(time_plot, ref_xyz_plot, act_xyz_plot, ee_3d_err_plot, ee_available)
                fig_3d = build_3d_path_figure(ref_xyz_plot, act_xyz_plot)

                content = html.Div(
                    id="cartesian-charts-container",
                    children=[
                        html.Div(
                            id="card-ee-cartesian",
                            className="dashboard-card",
                            style={"marginBottom": "20px", "minHeight": "360px"},
                            children=[
                                html.Div(
                                    style={"display": "flex", "justifyContent": "space-between", "alignItems": "center", "marginBottom": "12px"},
                                    children=[
                                        html.Span("Cartesian Coordinate Frame: base_link → tool0 (Validated)", style={"fontWeight": "600", "fontSize": "13px"}),
                                        html.Span("Translational Error Only (mm)", style={"fontSize": "11px", "color": COLORS["text_secondary"]}),
                                    ],
                                ),
                                dcc.Graph(id="graph-ee-cartesian", figure=fig_cart, config={"responsive": True}),
                            ],
                        ),
                        html.Div(
                            id="card-ee-3d",
                            className="dashboard-card",
                            style={"minHeight": "480px"},
                            children=[
                                html.Div("Interactive 3D Spatial Trajectory (Pick & Place)", className="card-title"),
                                dcc.Graph(id="graph-ee-3d", figure=fig_3d, config={"responsive": True}),
                            ],
                        ),
                    ],
                )

        elif active_tab == "tab-metrics":
            # Tab 4: Performance Metrics Table & Analytics
            if run_metrics:
                rows = []
                for j in JOINTS:
                    jm = run_metrics["joints"][j]
                    rows.append(
                        html.Tr(
                            children=[
                                html.Td(JOINT_LABELS[j], style={"fontWeight": "600"}),
                                html.Td(f"{jm['sample_count']:,}"),
                                html.Td(f"{jm['rmse_rad']:.6f} rad ({jm['rmse_deg']:.3f}°)"),
                                html.Td(
                                    f"{jm['max_err_rad']:.6f} rad ({jm['max_err_deg']:.3f}°)",
                                    style={"color": COLORS["danger"] if jm["max_err_deg"] > 3.0 else COLORS["text_primary"]},
                                ),
                                html.Td(f"{jm['iae_rad_s']:.5f}"),
                                html.Td(f"{jm['ise_rad2_s']:.6f}"),
                                html.Td(f"{jm['peak_time_sec']:.2f} s"),
                            ]
                        )
                    )

                cycle_m = run_metrics["cycle"]
                content = html.Div(
                    children=[
                        html.Div(
                            className="dashboard-card",
                            children=[
                                html.Div(
                                    style={"display": "flex", "justifyContent": "space-between", "alignItems": "center", "marginBottom": "16px"},
                                    children=[
                                        html.Div("Joint Tracking Performance Summary (Full Resolution)", className="card-title"),
                                        html.Div(
                                            f"Logged Duration: {cycle_m['logged_duration_sec']:.2f} s | Active Motion: {cycle_m['active_duration_sec']:.2f} s",
                                            style={"fontSize": "12px", "color": COLORS["text_secondary"]},
                                        ),
                                    ],
                                ),
                                html.Table(
                                    className="dark-table",
                                    children=[
                                        html.Thead(
                                            html.Tr(
                                                children=[
                                                    html.Th("Joint"),
                                                    html.Th("Samples"),
                                                    html.Th("RMSE (rad / deg)"),
                                                    html.Th("Max Error (rad / deg)"),
                                                    html.Th("IAE (rad·s)"),
                                                    html.Th("ISE (rad²·s)"),
                                                    html.Th("Peak Error Stamp"),
                                                ]
                                            )
                                        ),
                                        html.Tbody(rows),
                                    ],
                                ),
                            ],
                        ),
                    ]
                )
            else:
                content = html.Div("No performance metrics available for the active selection.")

        elif active_tab == "tab-pid":
            # Tab 5: PID Control (Read-Only / Draft Mode in Phase 1)
            content = html.Div(
                children=[
                    # Safety Banner
                    html.Div(
                        className="safety-banner",
                        children=[
                            html.Div(
                                style={"display": "flex", "alignItems": "center", "gap": "10px", "marginBottom": "6px"},
                                children=[
                                    html.Span("🔒", style={"fontSize": "16px"}),
                                    html.Span("SAFETY POLICY ENFORCEMENT — PHASE 1 READ-ONLY MODE", style={"fontWeight": "700", "color": COLORS["amber"], "fontSize": "13px"}),
                                ],
                            ),
                            html.Div(
                                style={"fontSize": "12px", "color": COLORS["text_primary"], "lineHeight": "1.5"},
                                children=[
                                    "Runtime parameter modification and controller switching are strictly disabled in Phase 1. "
                                    "The verified UR5 simulation controller currently operates in ",
                                    html.B("open-loop position mode (open_loop_control: true)"),
                                    " via the GazeboSimSystem hardware interface. No commanded torque or dynamic PID feedback loops are active. "
                                    "You may inspect nominal parameters and draft proposed gains below for offline evaluation.",
                                ],
                            ),
                        ],
                    ),

                    # Gain Draft Table
                    html.Div(
                        className="dashboard-card",
                        children=[
                            html.Div("Proposed Fixed PID Gains (Draft Only — Read-Only Mode)", className="card-title"),
                            html.Table(
                                className="dark-table",
                                style={"marginTop": "14px"},
                                children=[
                                    html.Thead(
                                        html.Tr(
                                            children=[
                                                html.Th("Joint"),
                                                html.Th("Active Mode"),
                                                html.Th("Proportional Gain (Kp)"),
                                                html.Th("Integral Gain (Ki)"),
                                                html.Th("Derivative Gain (Kd)"),
                                                html.Th("Draft Status"),
                                            ]
                                        )
                                    ),
                                    html.Tbody(
                                        [
                                            html.Tr(
                                                children=[
                                                    html.Td(JOINT_LABELS[j], style={"fontWeight": "600"}),
                                                    html.Td("Open-Loop Position", style={"color": COLORS["text_secondary"]}),
                                                    html.Td(dcc.Input(type="number", value=100.0 if idx < 3 else 40.0, disabled=False, className="styled-input", style={"width": "100px"})),
                                                    html.Td(dcc.Input(type="number", value=0.0, disabled=False, className="styled-input", style={"width": "100px"})),
                                                    html.Td(dcc.Input(type="number", value=5.0 if idx < 3 else 1.5, disabled=False, className="styled-input", style={"width": "100px"})),
                                                    html.Td(html.Span("Draft (Offline)", style={"color": COLORS["amber"], "fontSize": "11px", "fontWeight": "600"})),
                                                ]
                                            )
                                            for idx, j in enumerate(JOINTS)
                                        ]
                                    ),
                                ],
                            ),
                            html.Div(
                                style={"marginTop": "16px", "display": "flex", "gap": "10px", "alignItems": "center"},
                                children=[
                                    html.Button("Save Draft Configuration", id="btn-save-draft-gains", className="btn-secondary"),
                                    html.Span("Note: Gain updates will NOT be published to ROS 2 in Phase 1.", style={"fontSize": "11px", "color": COLORS["text_muted"]}),
                                ],
                            ),
                        ],
                    ),

                    # Controller Architecture Details
                    html.Div(
                        className="dashboard-card",
                        style={"marginTop": "20px"},
                        children=[
                            html.Div("Controller Configuration Provenance", className="card-title"),
                            html.Div(
                                style={"fontSize": "12px", "color": COLORS["text_secondary"], "lineHeight": "1.6", "marginTop": "10px"},
                                children=[
                                    html.Div("• Controller: joint_trajectory_controller/JointTrajectoryController"),
                                    html.Div("• Command Interfaces: [position] (ros2_controllers.yaml)"),
                                    html.Div("• State Interfaces: [position]"),
                                    html.Div("• Update Rate: 100 Hz (Controller Manager) / 1000 Hz (Gazebo Physics)"),
                                    html.Div("• Phase 2 Proposal: Closed-loop PID gain tuning will be enabled in Phase 2 only after complete validation of Phase 1 read-only monitoring."),
                                ],
                            ),
                        ],
                    ),
                ]
            )

        elif active_tab == "tab-settings":
            # Tab 6: Settings and Run Annotations
            run_id = Path(run_path).parent.name if run_path else "current_run"
            notes_data = data_manager.load_annotation(run_id)

            content = html.Div(
                className="dashboard-card",
                children=[
                    html.Div(f"Experiment Notes & Annotation: {run_id}", className="card-title"),
                    html.Div(
                        style={"display": "flex", "flexDirection": "column", "gap": "12px", "marginTop": "14px"},
                        children=[
                            html.Div(
                                children=[
                                    html.Label("Operator / Engineer", style={"fontSize": "12px", "color": COLORS["text_secondary"]}),
                                    dcc.Input(id="input-operator", type="text", value=notes_data.get("operator", ""), placeholder="e.g. ROS Engineer", className="styled-input"),
                                ]
                            ),
                            html.Div(
                                children=[
                                    html.Label("Run Notes / Observations", style={"fontSize": "12px", "color": COLORS["text_secondary"]}),
                                    dcc.Textarea(id="input-notes", value=notes_data.get("notes", ""), placeholder="Add observations on trajectory tracking, overshoot, or vibrations...", className="styled-input", style={"height": "100px"}),
                                ]
                            ),
                            html.Button("Save Annotations (Separately from Logs)", id="btn-save-notes", className="btn-primary", style={"alignSelf": "flex-start"}),
                            html.Div(id="notes-save-status", style={"fontSize": "12px", "color": COLORS["success"]}),
                        ],
                    ),
                ]
            )

        return kpi_cards, content

    # 5. Save Annotation Callback
    @app.callback(
        Output("notes-save-status", "children"),
        [Input("btn-save-notes", "n_clicks")],
        [
            State("radio-data-source", "value"),
            State("dropdown-run-selector", "value"),
            State("input-operator", "value"),
            State("input-notes", "value"),
        ],
    )
    def save_run_notes(n_clicks, data_source, run_path, operator, notes):
        if not n_clicks:
            return ""
        run_id = Path(run_path).parent.name if run_path else "live_session"
        data_manager.save_annotation(run_id, {
            "operator": operator or "",
            "notes": notes or "",
            "run_id": run_id,
        })
        return "✓ Annotations successfully saved to separate dashboard metadata file."

