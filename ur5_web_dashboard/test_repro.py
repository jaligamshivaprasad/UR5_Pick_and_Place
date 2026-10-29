import sys
from pathlib import Path

WS_ROOT = Path(__file__).resolve().parent.parent
if str(WS_ROOT) not in sys.path:
    sys.path.insert(0, str(WS_ROOT))

import traceback
from ur5_web_dashboard.app import create_app
from ur5_web_dashboard.data_manager import DataManager
from ur5_web_dashboard.config import JOINTS

app = create_app()
dm = DataManager()
runs = dm.list_available_runs()
print("Available runs:", [r["id"] for r in runs])

cb_func = None
for cid, cb in app.callback_map.items():
    if "kpi-summary-grid.children" in cid:
        cb_func = getattr(cb["callback"], "__wrapped__", cb["callback"])
        break

print("Testing callback with all runs and tabs...")
for run in runs:
    run_path = run["path"]
    for overlay in ["none", runs[0]["path"]]:
        for tab in ["tab-tracking", "tab-velocities", "tab-cartesian", "tab-metrics", "tab-pid", "tab-settings"]:
            try:
                kpis, content = cb_func(1, tab, "historical", run_path, overlay, JOINTS)
                print(f"Run {run['id']} | Overlay {overlay != 'none'} | Tab {tab}: OK")
            except Exception as e:
                print(f"ERROR on Run {run['id']} | Tab {tab}: {e}")
                traceback.print_exc()

app.ros_monitor.stop()
