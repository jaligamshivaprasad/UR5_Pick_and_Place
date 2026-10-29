"""Main dashboard layout assembling header, responsive sidebar, tabs, and chart containers."""

from dash import dcc, html

from .config import (
    COLORS,
    DEFAULT_PORT,
    DEFAULT_REFRESH_INTERVAL_MS,
    JOINT_LABELS,
    JOINTS,
)


def create_layout(available_runs: list) -> html.Div:
    """Build the complete Dash single-page layout."""
    run_options = [{"label": f"📁 {r['id']} ({r['controller_mode']})", "value": r["path"]} for r in available_runs]
    default_run = available_runs[0]["path"] if available_runs else None

    overlay_options = [{"label": "-- None --", "value": "none"}] + [
        {"label": f"📊 {r['id']}", "value": r["path"]} for r in available_runs
    ]

    return html.Div(
        style={"backgroundColor": COLORS["bg_dark"], "minHeight": "100vh"},
        children=[
            # Heartbeat / polling interval timer for live ROS 2 stream
            dcc.Interval(
                id="live-interval",
                interval=DEFAULT_REFRESH_INTERVAL_MS,
                n_intervals=0,
                disabled=False,
            ),
            # In-browser client-side stores for active state
            dcc.Store(id="store-active-mode", data="live"),
            dcc.Store(id="store-selected-run", data=default_run),
            dcc.Store(id="store-settings", data={}),
            dcc.Store(id="store-notification", data=None),

            # Top Application Header
            html.Header(
                className="app-header",
                children=[
                    html.Div(
                        style={"display": "flex", "alignItems": "center", "gap": "16px"},
                        children=[
                            html.Div(
                                className="brand-title",
                                children=[
                                    html.Span("🦾 UR5 ROBOTICS", style={"color": COLORS["text_primary"]}),
                                    html.Span("PID DASHBOARD", style={"color": COLORS["reference"], "fontWeight": "300"}),
                                    html.Span("PHASE 1", className="brand-badge"),
                                ],
                            ),
                        ],
                    ),
                    # Live Status Badges Bar
                    html.Div(
                        className="status-bar",
                        id="header-status-bar",
                        children=[
                            html.Div(id="badge-ros-status", children="Initializing ROS..."),
                            html.Div(id="badge-sim-time", children="Sim Time: --"),
                            html.Div(id="badge-controller-rate", children="Rate: -- Hz"),
                            html.Div(id="badge-fk-status", children="FK: Checking"),
                        ],
                    ),
                ],
            ),

            # Main Body Container: Sidebar + Content
            html.Div(
                className="dashboard-container",
                children=[
                    # Left Sidebar
                    html.Aside(
                        className="sidebar",
                        children=[
                            # Data Source Selector
                            html.Div(
                                className="sidebar-section",
                                children=[
                                    html.Div("Data Source", className="sidebar-title"),
                                    dcc.RadioItems(
                                        id="radio-data-source",
                                        options=[
                                            {"label": " 🔴 Live ROS 2 Stream (Active / Gazebo)", "value": "live"},
                                            {"label": " 📁 Historical Run Explorer", "value": "historical"},
                                        ],
                                        value="live",
                                        labelStyle={"display": "block", "marginBottom": "6px", "fontSize": "13px", "cursor": "pointer"},
                                        style={"color": COLORS["text_primary"]},
                                    ),
                                ],
                            ),

                            # Run Selector
                            html.Div(
                                className="sidebar-section",
                                id="section-run-selector",
                                children=[
                                    html.Div("Select Experiment Run", className="sidebar-title"),
                                    dcc.Dropdown(
                                        id="dropdown-run-selector",
                                        options=run_options,
                                        value=default_run,
                                        clearable=False,
                                        style={"fontSize": "12px", "color": "#000"},
                                    ),
                                    html.Div(
                                        id="sidebar-run-info",
                                        style={"fontSize": "11px", "color": COLORS["text_secondary"], "marginTop": "6px", "lineHeight": "1.4"},
                                    ),
                                ],
                            ),

                            # Run Overlay
                            html.Div(
                                className="sidebar-section",
                                children=[
                                    html.Div("Overlay Reference Run", className="sidebar-title"),
                                    dcc.Dropdown(
                                        id="dropdown-overlay-selector",
                                        options=overlay_options,
                                        value="none",
                                        clearable=False,
                                        style={"fontSize": "12px", "color": "#000"},
                                    ),
                                ],
                            ),

                            # Joint Filter Toggles
                            html.Div(
                                className="sidebar-section",
                                children=[
                                    html.Div(
                                        style={"display": "flex", "justifyContent": "space-between", "alignItems": "center"},
                                        children=[
                                            html.Div("Joint Filter (J1–J6)", className="sidebar-title"),
                                            html.Button(
                                                "All",
                                                id="btn-select-all-joints",
                                                className="btn-secondary",
                                                style={"padding": "2px 6px", "fontSize": "10px"},
                                            ),
                                        ],
                                    ),
                                    dcc.Checklist(
                                        id="checklist-joints",
                                        options=[{"label": f" {JOINT_LABELS[j]}", "value": j} for j in JOINTS],
                                        value=JOINTS,
                                        labelStyle={"display": "block", "marginBottom": "4px", "fontSize": "12px", "cursor": "pointer"},
                                        style={"color": COLORS["text_primary"]},
                                    ),
                                ],
                            ),

                            # Refresh Rate Selector
                            html.Div(
                                className="sidebar-section",
                                children=[
                                    html.Div("Live Refresh Interval", className="sidebar-title"),
                                    dcc.Dropdown(
                                        id="dropdown-refresh-rate",
                                        options=[
                                            {"label": "Paused", "value": 0},
                                            {"label": "250 ms (4 Hz)", "value": 250},
                                            {"label": "500 ms (2 Hz - Default)", "value": 500},
                                            {"label": "1000 ms (1 Hz)", "value": 1000},
                                            {"label": "2000 ms (0.5 Hz)", "value": 2000},
                                        ],
                                        value=DEFAULT_REFRESH_INTERVAL_MS,
                                        clearable=False,
                                        style={"fontSize": "12px", "color": "#000"},
                                    ),
                                ],
                            ),

                            # Safety Notice Box in Sidebar
                            html.Div(
                                style={
                                    "marginTop": "auto",
                                    "padding": "12px",
                                    "borderRadius": "6px",
                                    "backgroundColor": COLORS["surface_light"],
                                    "border": f"1px solid {COLORS['border']}",
                                    "fontSize": "11px",
                                    "color": COLORS["text_secondary"],
                                },
                                children=[
                                    html.Div("🛡️ SAFETY CONSTRAINTS", style={"fontWeight": "700", "color": COLORS["amber"], "marginBottom": "4px"}),
                                    html.Div("• Read-only topic monitor"),
                                    html.Div("• No joint command publishers"),
                                    html.Div("• Fixed controller & logger safe"),
                                    html.Div("• No fabricated torques or gains"),
                                ],
                            ),
                        ],
                    ),

                    # Main Content Tabbed Area
                    html.Main(
                        className="main-content",
                        children=[
                            # Executive KPI Summary Grid
                            html.Div(id="kpi-summary-grid", className="kpi-grid"),

                            # Navigation Tabs
                            dcc.Tabs(
                                id="dashboard-tabs",
                                value="tab-tracking",
                                className="nav-tabs-container",
                                children=[
                                    dcc.Tab(label="Joint Tracking (J1–J6)", value="tab-tracking", className="custom-tab", selected_className="custom-tab--selected"),
                                    dcc.Tab(label="Velocities & Effort", value="tab-velocities", className="custom-tab", selected_className="custom-tab--selected"),
                                    dcc.Tab(label="Cartesian & 3D EE", value="tab-cartesian", className="custom-tab", selected_className="custom-tab--selected"),
                                    dcc.Tab(label="Performance Metrics", value="tab-metrics", className="custom-tab", selected_className="custom-tab--selected"),
                                    dcc.Tab(label="PID Control (Draft / Read-Only)", value="tab-pid", className="custom-tab", selected_className="custom-tab--selected"),
                                    dcc.Tab(label="Settings & Notes", value="tab-settings", className="custom-tab", selected_className="custom-tab--selected"),
                                ],
                            ),

                            # Dynamic Tab Content Area
                            html.Div(id="tab-content"),
                        ],
                    ),
                ],
            ),
        ],
    )

