"""UR5 Web Dashboard application initialization and factory."""

from pathlib import Path
import sys

from dash import Dash

# Ensure package is on path
WS_ROOT = Path(__file__).resolve().parent.parent
if str(WS_ROOT) not in sys.path:
    sys.path.insert(0, str(WS_ROOT))

from ur5_web_dashboard.callbacks import register_callbacks
from ur5_web_dashboard.config import DEFAULT_BUFFER_SIZE
from ur5_web_dashboard.data_manager import DataManager
from ur5_web_dashboard.layout import create_layout
from ur5_web_dashboard.ros_monitor import ROSMonitor


def create_app(buffer_size: int = DEFAULT_BUFFER_SIZE) -> Dash:
    """Instantiate and configure the Dash application."""
    assets_folder = str(Path(__file__).resolve().parent / "assets")
    app = Dash(
        __name__,
        title="UR5 Robot PID Performance Dashboard",
        assets_folder=assets_folder,
        update_title=None,  # Clean UI without 'Updating...' title flicker
        suppress_callback_exceptions=True,
    )

    # Initialize data layer and ROS monitoring
    data_manager = DataManager(WS_ROOT)
    available_runs = data_manager.list_available_runs()

    ros_monitor = ROSMonitor(buffer_size=buffer_size)
    ros_monitor.start()

    # Build Layout and connect callbacks
    app.layout = create_layout(available_runs)
    register_callbacks(app, ros_monitor, data_manager)

    # Attach monitor reference to app for clean teardown
    app.ros_monitor = ros_monitor
    return app

