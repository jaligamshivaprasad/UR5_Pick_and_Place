"""Test all Dash HTTP and POST callback endpoints."""

import json
from pathlib import Path
import sys

WS_ROOT = Path(__file__).resolve().parent.parent
if str(WS_ROOT) not in sys.path:
    sys.path.insert(0, str(WS_ROOT))

from ur5_web_dashboard.app import create_app
from ur5_web_dashboard.config import JOINTS
from ur5_web_dashboard.data_manager import DataManager


def test_endpoints():
    app = create_app()
    client = app.server.test_client()
    dm = DataManager()
    runs = dm.list_available_runs()
    assert len(runs) > 0, "No runs found"
    run_path = runs[0]["path"]

    tabs = [
        "tab-tracking",
        "tab-velocities",
        "tab-cartesian",
        "tab-metrics",
        "tab-pid",
        "tab-settings",
    ]

    print("=" * 60)
    print("Testing Dash HTTP Endpoints and Callbacks")
    print("=" * 60)

    # 1. Root and Layout
    r_root = client.get("/")
    assert r_root.status_code == 200
    print("1. [PASS] Root / endpoint: HTTP 200")

    r_layout = client.get("/_dash-layout")
    assert r_layout.status_code == 200
    print("2. [PASS] /_dash-layout: HTTP 200")

    r_deps = client.get("/_dash-dependencies")
    assert r_deps.status_code == 200
    print("3. [PASS] /_dash-dependencies: HTTP 200")

    # 2. Status Badges update callback
    payload_badges = {
        "output": "..badge-ros-status.children...badge-sim-time.children...badge-controller-rate.children...badge-fk-status.children...sidebar-run-info.children..",
        "outputs": [
            {"id": "badge-ros-status", "property": "children"},
            {"id": "badge-sim-time", "property": "children"},
            {"id": "badge-controller-rate", "property": "children"},
            {"id": "badge-fk-status", "property": "children"},
            {"id": "sidebar-run-info", "property": "children"},
        ],
        "inputs": [
            {"id": "live-interval", "property": "n_intervals", "value": 1},
            {"id": "radio-data-source", "property": "value", "value": "historical"},
            {"id": "dropdown-run-selector", "property": "value", "value": run_path},
        ],
        "changedPropIds": ["live-interval.n_intervals"],
    }
    r = client.post("/_dash-update-component", json=payload_badges)
    assert r.status_code == 200, f"Status badges error: {r.status_code}"
    print("4. [PASS] Status Badges callback: HTTP 200")

    # 3. Test every Tab view callback
    for tab in tabs:
        payload_tab = {
            "output": "..kpi-summary-grid.children...tab-content.children..",
            "outputs": [
                {"id": "kpi-summary-grid", "property": "children"},
                {"id": "tab-content", "property": "children"},
            ],
            "inputs": [
                {"id": "live-interval", "property": "n_intervals", "value": 1},
                {"id": "dashboard-tabs", "property": "value", "value": tab},
                {"id": "radio-data-source", "property": "value", "value": "historical"},
                {"id": "dropdown-run-selector", "property": "value", "value": run_path},
                {"id": "dropdown-overlay-selector", "property": "value", "value": "none"},
                {"id": "checklist-joints", "property": "value", "value": JOINTS},
            ],
            "changedPropIds": ["dashboard-tabs.value"],
        }
        r = client.post("/_dash-update-component", json=payload_tab)
        assert r.status_code == 200, f"Tab {tab} error: {r.status_code}"
        data = json.loads(r.data)
        assert "response" in data
        print(f"5. [PASS] Tab [{tab}]: HTTP 200 (KPI + Content generated)")

    # 4. Test Live stream mode callback
    payload_live = {
        "output": "..kpi-summary-grid.children...tab-content.children..",
        "outputs": [
            {"id": "kpi-summary-grid", "property": "children"},
            {"id": "tab-content", "property": "children"},
        ],
        "inputs": [
            {"id": "live-interval", "property": "n_intervals", "value": 2},
            {"id": "dashboard-tabs", "property": "value", "value": "tab-tracking"},
            {"id": "radio-data-source", "property": "value", "value": "live"},
            {"id": "dropdown-run-selector", "property": "value", "value": run_path},
            {"id": "dropdown-overlay-selector", "property": "value", "value": "none"},
            {"id": "checklist-joints", "property": "value", "value": JOINTS},
        ],
        "changedPropIds": ["live-interval.n_intervals"],
    }
    r = client.post("/_dash-update-component", json=payload_live)
    assert r.status_code == 200
    print("6. [PASS] Live stream mode callback: HTTP 200")

    # 5. Clean teardown
    app.ros_monitor.stop()
    print("=" * 60)
    print("ALL 6 VERIFICATION CHECKS PASSED PERFECTLY!")
    print("=" * 60)


if __name__ == "__main__":
    test_endpoints()

