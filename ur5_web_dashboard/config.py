"""Configuration constants, theme palettes, and persistent settings for UR5 Web Dashboard."""

import json
from pathlib import Path

# Network and Server Configuration
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8050
DEFAULT_BUFFER_SIZE = 2500
DEFAULT_REFRESH_INTERVAL_MS = 500  # 2 Hz live updates

# Robot Kinematic Joint Definitions
JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

JOINT_LABELS = {
    "shoulder_pan_joint": "J1: Shoulder Pan",
    "shoulder_lift_joint": "J2: Shoulder Lift",
    "elbow_joint": "J3: Elbow",
    "wrist_1_joint": "J4: Wrist 1",
    "wrist_2_joint": "J5: Wrist 2",
    "wrist_3_joint": "J6: Wrist 3",
}

JOINT_SHORT_LABELS = {
    "shoulder_pan_joint": "J1",
    "shoulder_lift_joint": "J2",
    "elbow_joint": "J3",
    "wrist_1_joint": "J4",
    "wrist_2_joint": "J5",
    "wrist_3_joint": "J6",
}

# Coordinate Frames
BASE_LINK = "base_link"
TCP_LINK = "tool0"

# ROS 2 Topics
TOPIC_CONTROLLER_STATE = "/arm_controller/controller_state"
TOPIC_JOINT_STATES = "/joint_states"
TOPIC_DYNAMIC_JOINT_STATES = "/dynamic_joint_states"
TOPIC_CLOCK = "/clock"
TOPIC_ROBOT_DESCRIPTION = "/robot_description"

# Theme Color Palette - Industrial Dark Charcoal & Navy
COLORS = {
    "bg_dark": "#0a0e17",         # Deep charcoal canvas
    "surface": "#121824",         # Card background
    "surface_light": "#1b2333",   # Elevated elements / headers
    "border": "#283548",          # Subtle borders
    "border_focus": "#38bdf8",    # Focus ring
    "text_primary": "#f1f5f9",    # Primary white-slate text
    "text_secondary": "#94a3b8",  # Muted secondary labels
    "text_muted": "#64748b",      # De-emphasized notes
    "reference": "#06b6d4",       # Cyan for Reference / Target
    "cyan_ref": "#06b6d4",
    "actual": "#f59e0b",          # Amber for Actual / Feedback
    "amber": "#f59e0b",
    "error": "#f87171",           # Restrained Red for Tracking Error
    "red_err": "#f87171",
    "velocity": "#38bdf8",        # Sky Blue for Velocity
    "success": "#10b981",         # Emerald for OK/Online
    "warning": "#eab308",         # Yellow for Warning/Pending
    "danger": "#ef4444",          # Bright Red for Critical
    "accent": "#6366f1",          # Indigo for Navigation/Actions
    "grid": "#1e293b",            # Plotly grid lines
}

# Default Plotly Dark Layout Template
PLOT_LAYOUT_DEFAULTS = {
    "paper_bgcolor": COLORS["surface"],
    "plot_bgcolor": COLORS["bg_dark"],
    "font": {"color": COLORS["text_secondary"], "family": "Inter, Roboto, sans-serif", "size": 11},
    "margin": {"l": 55, "r": 25, "t": 38, "b": 40},
    "uirevision": "constant",  # Critical: preserves user pan, zoom, and scroll across live stream updates
    "xaxis": {
        "gridcolor": COLORS["grid"],
        "zerolinecolor": COLORS["border"],
        "showline": True,
        "linecolor": COLORS["border"],
        "tickfont": {"size": 10, "color": COLORS["text_secondary"]},
    },
    "yaxis": {
        "gridcolor": COLORS["grid"],
        "zerolinecolor": COLORS["border"],
        "showline": True,
        "linecolor": COLORS["border"],
        "tickfont": {"size": 10, "color": COLORS["text_secondary"]},
    },
    "legend": {
        "bgcolor": "rgba(18, 24, 36, 0.85)",
        "bordercolor": COLORS["border"],
        "borderwidth": 1,
        "font": {"size": 10, "color": COLORS["text_primary"]},
    },
    "hovermode": "x unified",
}

# Settings file locations
CONFIG_DIR = Path(__file__).resolve().parent
SETTINGS_FILE = CONFIG_DIR / "dashboard_settings.json"
ANNOTATIONS_DIR = CONFIG_DIR / "experiment_notes"
ANNOTATIONS_DIR.mkdir(parents=True, exist_ok=True)


def load_dashboard_settings() -> dict:
    """Load persisted dashboard settings, returning defaults if not found."""
    defaults = {
        "refresh_interval_ms": DEFAULT_REFRESH_INTERVAL_MS,
        "buffer_size": DEFAULT_BUFFER_SIZE,
        "selected_joints": JOINTS,
        "show_error_band": True,
        "error_threshold_rad": 0.035,  # ~2 degrees alert threshold
        "dark_mode": True,
        "auto_scroll_live": True,
    }
    if SETTINGS_FILE.exists():
        try:
            with open(SETTINGS_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
                defaults.update(data)
        except Exception:
            pass
    return defaults


def save_dashboard_settings(settings: dict) -> bool:
    """Save user dashboard preferences separately from experiment data."""
    try:
        with open(SETTINGS_FILE, "w", encoding="utf-8") as f:
            json.dump(settings, f, indent=2)
        return True
    except Exception:
        return False
