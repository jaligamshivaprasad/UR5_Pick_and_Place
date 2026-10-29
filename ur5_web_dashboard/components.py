"""Reusable visual UI components, KPI cards, and Plotly graph generators for UR5 Dashboard."""

import math
from typing import Any, Dict, List, Optional, Tuple

from dash import dcc, html
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from .config import (
    COLORS,
    JOINT_LABELS,
    JOINT_SHORT_LABELS,
    JOINTS,
    PLOT_LAYOUT_DEFAULTS,
)


def create_kpi_card(
    title: str,
    value: str,
    subtitle: str,
    badge_color: str = COLORS["accent"],
    alert: bool = False,
    card_id: Optional[str] = None,
):
    """Render an industrial metric KPI card with stable DOM ID."""
    card_border = COLORS["danger"] if alert else COLORS["border"]
    return html.Div(
        id=card_id,
        className="kpi-card",
        style={
            "backgroundColor": COLORS["surface"],
            "border": f"1px solid {card_border}",
            "borderRadius": "8px",
            "padding": "16px 20px",
            "position": "relative",
            "boxShadow": "0 4px 6px -1px rgba(0, 0, 0, 0.3)",
        },
        children=[
            html.Div(
                style={"display": "flex", "justifyContent": "space-between", "alignItems": "center", "marginBottom": "6px"},
                children=[
                    html.Span(title.upper(), style={"fontSize": "11px", "fontWeight": "600", "color": COLORS["text_secondary"], "letterSpacing": "0.08em"}),
                    html.Span(
                        "ALERT" if alert else "NORMAL",
                        style={
                            "fontSize": "10px",
                            "padding": "2px 8px",
                            "borderRadius": "4px",
                            "backgroundColor": f"{card_border}25",
                            "color": card_border,
                            "fontWeight": "600",
                        },
                    ),
                ],
            ),
            html.Div(value, style={"fontSize": "26px", "fontWeight": "700", "color": COLORS["text_primary"], "marginBottom": "4px", "fontFamily": "monospace"}),
            html.Div(subtitle, style={"fontSize": "12px", "color": COLORS["text_muted"]}),
        ],
    )


def create_status_pill(label: str, status_text: str, state: str = "ok"):
    """Create a header status indicator with optional pulse animation."""
    state_colors = {
        "ok": COLORS["success"],
        "warn": COLORS["warning"],
        "danger": COLORS["danger"],
        "info": COLORS["accent"],
        "muted": COLORS["text_muted"],
    }
    color = state_colors.get(state, COLORS["text_muted"])
    return html.Div(
        style={
            "display": "inline-flex",
            "alignItems": "center",
            "gap": "8px",
            "backgroundColor": COLORS["surface"],
            "padding": "6px 14px",
            "borderRadius": "6px",
            "border": f"1px solid {COLORS['border']}",
            "fontSize": "12px",
        },
        children=[
            html.Span(
                style={
                    "width": "8px",
                    "height": "8px",
                    "borderRadius": "50%",
                    "backgroundColor": color,
                    "boxShadow": f"0 0 8px {color}" if state == "ok" else "none",
                }
            ),
            html.Span(f"{label}:", style={"color": COLORS["text_secondary"], "fontWeight": "500"}),
            html.Span(status_text, style={"color": COLORS["text_primary"], "fontWeight": "600"}),
        ],
    )


def build_joint_tracking_figure(
    joint: str,
    time_series: np.ndarray,
    ref_pos: np.ndarray,
    act_pos: np.ndarray,
    err_pos: np.ndarray,
    overlay_time: Optional[np.ndarray] = None,
    overlay_act: Optional[np.ndarray] = None,
    overlay_name: Optional[str] = None,
    height: int = 340,
) -> go.Figure:
    """Create a synchronized 2-row subplot for a single joint: Position (top) and Error (bottom)."""
    traces = [
        go.Scatter(
            x=time_series,
            y=ref_pos,
            mode="lines",
            name="Reference (Cyan)",
            line=dict(color=COLORS["reference"], width=2.2),
            hovertemplate="Ref: %{y:.4f} rad<extra></extra>",
            yaxis="y1",
            xaxis="x1",
        ),
        go.Scatter(
            x=time_series,
            y=act_pos,
            mode="lines",
            name="Actual (Amber)",
            line=dict(color=COLORS["actual"], width=2.0),
            hovertemplate="Act: %{y:.4f} rad<extra></extra>",
            yaxis="y1",
            xaxis="x1",
        ),
    ]

    # Optional Overlay Trace
    if overlay_act is not None and overlay_time is not None and len(overlay_time) > 0:
        traces.append(
            go.Scatter(
                x=overlay_time,
                y=overlay_act,
                mode="lines",
                name=f"Overlay ({overlay_name or 'Ref Run'})",
                line=dict(color="#a855f7", width=1.8, dash="dot"),
                hovertemplate="Overlay Act: %{y:.4f} rad<extra></extra>",
                yaxis="y1",
                xaxis="x1",
            )
        )

    # Bottom: Tracking Error (Reference minus Actual)
    traces.append(
        go.Scatter(
            x=time_series,
            y=err_pos,
            mode="lines",
            name="Error (Red)",
            line=dict(color=COLORS["error"], width=1.8),
            fill="tozeroy",
            fillcolor="rgba(248, 113, 113, 0.15)",
            hovertemplate="Err: %{y:.5f} rad (%{customdata:.3f}°)<extra></extra>",
            customdata=np.degrees(err_pos),
            yaxis="y2",
            xaxis="x1",
        )
    )

    title_text = f"<b>{JOINT_LABELS.get(joint, joint)}</b> — Position Tracking & Error"
    layout = dict(PLOT_LAYOUT_DEFAULTS)
    layout.update({
        "title": {"text": title_text, "font": {"size": 13, "color": COLORS["text_primary"]}, "x": 0.01, "y": 0.98},
        "height": height,
        "showlegend": True,
        "legend": dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="right",
            x=1.0,
            bgcolor="rgba(0,0,0,0)",
            font=dict(size=10),
        ),
        "yaxis": {
            "title": "Position (rad)",
            "domain": [0.38, 1.0],
            "gridcolor": COLORS["grid"],
            "zerolinecolor": COLORS["grid"],
        },
        "yaxis2": {
            "title": "Error (rad)",
            "domain": [0.0, 0.28],
            "gridcolor": COLORS["grid"],
            "zerolinecolor": COLORS["border"],
        },
        "xaxis": {
            "title": "Simulation Time (s)",
            "gridcolor": COLORS["grid"],
            "zerolinecolor": COLORS["grid"],
        },
    })
    return go.Figure(data=traces, layout=layout)


def build_joint_velocity_figure(
    joint: str,
    time_series: np.ndarray,
    ref_vel: np.ndarray,
    act_vel: np.ndarray,
    height: int = 280,
) -> go.Figure:
    """Create velocity plot for a joint."""
    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=time_series,
            y=ref_vel,
            mode="lines",
            name="Ref Velocity",
            line=dict(color=COLORS["reference"], width=1.8, dash="dash"),
            hovertemplate="Ref Vel: %{y:.4f} rad/s<extra></extra>",
        )
    )
    fig.add_trace(
        go.Scatter(
            x=time_series,
            y=act_vel,
            mode="lines",
            name="Act Velocity",
            line=dict(color=COLORS["velocity"], width=2.0),
            hovertemplate="Act Vel: %{y:.4f} rad/s<extra></extra>",
        )
    )

    title_text = f"<b>{JOINT_LABELS.get(joint, joint)}</b> — Velocity Profile"
    layout = dict(PLOT_LAYOUT_DEFAULTS)
    layout.update({
        "title": {"text": title_text, "font": {"size": 12, "color": COLORS["text_primary"]}, "x": 0.01, "y": 0.96},
        "height": height,
        "yaxis": {**PLOT_LAYOUT_DEFAULTS["yaxis"], "title": "Velocity (rad/s)"},
        "xaxis": {**PLOT_LAYOUT_DEFAULTS["xaxis"], "title": "Time (s)"},
        "legend": dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1.0, bgcolor="rgba(0,0,0,0)"),
    })
    fig.update_layout(**layout)
    return fig


def build_effort_figure(
    time_series: np.ndarray,
    effort_data: Dict[str, np.ndarray],
    is_available: bool,
    height: int = 300,
) -> go.Figure:
    """Create effort plot or clearly display authenticity notice if unavailable."""
    fig = go.Figure()

    if is_available and any(len(v) > 0 and np.any(np.isfinite(v)) for v in effort_data.values()):
        palette = ["#38bdf8", "#818cf8", "#c084fc", "#f472b6", "#fb923c", "#4ade80"]
        for idx, (j, eff_vals) in enumerate(effort_data.items()):
            if len(eff_vals) > 0 and np.any(np.isfinite(eff_vals)):
                fig.add_trace(
                    go.Scatter(
                        x=time_series,
                        y=eff_vals,
                        mode="lines",
                        name=JOINT_SHORT_LABELS.get(j, j),
                        line=dict(color=palette[idx % len(palette)], width=1.8),
                        hovertemplate=f"{JOINT_SHORT_LABELS.get(j, j)}: %{{y:.2f}} N·m<extra></extra>",
                    )
                )
        fig.update_yaxes(title_text="Effort / Torque (N·m)")
    else:
        # Display authentic notification: Never fabricate effort!
        if len(time_series) > 0:
            fig.add_trace(
                go.Scatter(
                    x=time_series,
                    y=np.zeros_like(time_series),
                    mode="lines",
                    line=dict(color=COLORS["border"], width=1, dash="dot"),
                    name="Zero Baseline",
                    hoverinfo="none",
                )
            )
        fig.add_annotation(
            text=(
                "<b>MEASURED EFFORT UNAVAILABLE IN HARDWARE CONFIGURATION</b><br>"
                "The active UR5 controller uses open-loop position command interfaces.<br>"
                "Gazebo ros2_control does not export joint effort in this configuration.<br>"
                "<i>Data integrity verified: Zero or simulated torque is never fabricated.</i>"
            ),
            xref="paper",
            yref="paper",
            x=0.5,
            y=0.5,
            showarrow=False,
            font=dict(size=12, color=COLORS["text_secondary"]),
            align="center",
            bordercolor=COLORS["border"],
            borderwidth=1,
            borderpad=12,
            bgcolor="rgba(18, 24, 36, 0.9)",
        )
        fig.update_yaxes(title_text="Effort (N·m) [N/A]", range=[-1, 1])

    layout = dict(PLOT_LAYOUT_DEFAULTS)
    layout.update({
        "title": {"text": "<b>Joint Effort & Commanded Torque Status</b>", "font": {"size": 13, "color": COLORS["text_primary"]}, "x": 0.01, "y": 0.98},
        "height": height,
        "xaxis": {**PLOT_LAYOUT_DEFAULTS["xaxis"], "title": "Simulation Time (s)"},
        "legend": dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1.0, bgcolor="rgba(0,0,0,0)"),
    })
    fig.update_layout(**layout)
    return fig


def build_ee_cartesian_figure(
    time_series: np.ndarray,
    ref_xyz: np.ndarray,
    act_xyz: np.ndarray,
    err_3d_mm: np.ndarray,
    is_available: bool,
    height: int = 360,
) -> go.Figure:
    """Create End-Effector XYZ planned vs actual trajectories and 3D Euclidean error."""
    fig = make_subplots(
        rows=2,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.09,
        row_heights=[0.65, 0.35],
    )

    if is_available and len(time_series) > 0 and ref_xyz.ndim == 2 and ref_xyz.shape[1] >= 3:
        axes = [("X", "#38bdf8"), ("Y", "#34d399"), ("Z", "#fbbf24")]
        for idx, (axis_name, col) in enumerate(axes):
            # Planned
            fig.add_trace(
                go.Scatter(
                    x=time_series,
                    y=ref_xyz[:, idx],
                    mode="lines",
                    name=f"Ref {axis_name}",
                    line=dict(color=col, width=1.8, dash="dash"),
                    hovertemplate=f"Ref {axis_name}: %{{y:.4f}} m<extra></extra>",
                ),
                row=1,
                col=1,
            )
            # Actual
            fig.add_trace(
                go.Scatter(
                    x=time_series,
                    y=act_xyz[:, idx],
                    mode="lines",
                    name=f"Act {axis_name}",
                    line=dict(color=col, width=2.0),
                    hovertemplate=f"Act {axis_name}: %{{y:.4f}} m<extra></extra>",
                ),
                row=1,
                col=1,
            )

        # 3D Error in mm
        fig.add_trace(
            go.Scatter(
                x=time_series,
                y=err_3d_mm,
                mode="lines",
                name="3D Error (mm)",
                line=dict(color=COLORS["error"], width=2.0),
                fill="tozeroy",
                fillcolor="rgba(248, 113, 113, 0.15)",
                hovertemplate="3D Error: %{y:.3f} mm<extra></extra>",
            ),
            row=2,
            col=1,
        )
    else:
        fig.add_annotation(
            text="End-Effector Cartesian data unavailable (Awaiting verified /robot_description URDF)",
            xref="paper",
            yref="paper",
            x=0.5,
            y=0.5,
            showarrow=False,
            font=dict(size=12, color=COLORS["text_secondary"]),
            bgcolor="rgba(18, 24, 36, 0.9)",
            bordercolor=COLORS["border"],
        )

    layout = dict(PLOT_LAYOUT_DEFAULTS)
    layout.update({
        "title": {"text": "<b>End-Effector Tracking (base_link → tool0)</b>", "font": {"size": 13, "color": COLORS["text_primary"]}, "x": 0.01, "y": 0.98},
        "height": height,
        "showlegend": True,
        "legend": dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1.0, bgcolor="rgba(0,0,0,0)", font=dict(size=9)),
    })
    fig.update_layout(**layout)
    fig.update_yaxes(title_text="Cartesian Position (m)", row=1, col=1, gridcolor=COLORS["grid"])
    fig.update_yaxes(title_text="3D Error (mm)", row=2, col=1, gridcolor=COLORS["grid"])
    fig.update_xaxes(title_text="Simulation Time (s)", row=2, col=1, gridcolor=COLORS["grid"])
    return fig


def build_3d_path_figure(ref_xyz: np.ndarray, act_xyz: np.ndarray, height: int = 440) -> go.Figure:
    """Render interactive 3D spatial trajectory for pick-and-place visualization."""
    fig = go.Figure()

    if len(ref_xyz) > 0 and ref_xyz.ndim == 2 and ref_xyz.shape[1] >= 3:
        fig.add_trace(
            go.Scatter3d(
                x=ref_xyz[:, 0],
                y=ref_xyz[:, 1],
                z=ref_xyz[:, 2],
                mode="lines",
                name="Planned Path (Cyan)",
                line=dict(color=COLORS["reference"], width=4),
                hoverinfo="skip",
            )
        )
        fig.add_trace(
            go.Scatter3d(
                x=act_xyz[:, 0],
                y=act_xyz[:, 1],
                z=act_xyz[:, 2],
                mode="lines",
                name="Executed Path (Amber)",
                line=dict(color=COLORS["actual"], width=4),
                hoverinfo="skip",
            )
        )

        # Start and Finish markers
        fig.add_trace(
            go.Scatter3d(
                x=[act_xyz[0, 0]],
                y=[act_xyz[0, 1]],
                z=[act_xyz[0, 2]],
                mode="markers",
                name="Start Point",
                marker=dict(size=6, color=COLORS["success"], symbol="circle"),
            )
        )
        fig.add_trace(
            go.Scatter3d(
                x=[act_xyz[-1, 0]],
                y=[act_xyz[-1, 1]],
                z=[act_xyz[-1, 2]],
                mode="markers",
                name="End Point",
                marker=dict(size=6, color=COLORS["danger"], symbol="diamond"),
            )
        )

    fig.update_layout(
        paper_bgcolor=COLORS["surface"],
        font=dict(color=COLORS["text_secondary"]),
        margin=dict(l=10, r=10, t=30, b=10),
        height=height,
        scene=dict(
            xaxis=dict(title="X (m)", backgroundcolor=COLORS["bg_dark"], gridcolor=COLORS["grid"], showbackground=True),
            yaxis=dict(title="Y (m)", backgroundcolor=COLORS["bg_dark"], gridcolor=COLORS["grid"], showbackground=True),
            zaxis=dict(title="Z (m)", backgroundcolor=COLORS["bg_dark"], gridcolor=COLORS["grid"], showbackground=True),
            camera=dict(eye=dict(x=1.5, y=-1.5, z=1.2)),
        ),
        legend=dict(x=0.02, y=0.98, bgcolor="rgba(18, 24, 36, 0.8)"),
    )
    return fig

