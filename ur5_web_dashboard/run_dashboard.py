#!/usr/bin/env python3
"""Standalone launch script for UR5 Web-Based PID Performance Dashboard."""

import argparse
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys

# Ensure workspace root and virtualenv site-packages are accessible
WS_ROOT = Path(__file__).resolve().parent.parent
if str(WS_ROOT) not in sys.path:
    sys.path.insert(0, str(WS_ROOT))

# Also ensure .venv site-packages is in sys.path if running under system python
venv_site = WS_ROOT / ".venv" / "lib" / f"python{sys.version_info.major}.{sys.version_info.minor}" / "site-packages"
if venv_site.exists() and str(venv_site) not in sys.path:
    sys.path.insert(0, str(venv_site))

from ur5_web_dashboard.app import create_app
from ur5_web_dashboard.config import DEFAULT_BUFFER_SIZE, DEFAULT_HOST, DEFAULT_PORT


def is_port_in_use(host: str, port: int) -> bool:
    """Check if the given host and port are already bound."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((host, port))
            return False
        except OSError:
            return True


def kill_process_on_port(port: int):
    """Attempt to terminate any stale process holding the port."""
    try:
        subprocess.run(["fuser", "-k", f"{port}/tcp"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


def parse_args():
    parser = argparse.ArgumentParser(description="UR5 Web-Based PID Performance Dashboard (Phase 1)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"Web server port (default: {DEFAULT_PORT})")
    parser.add_argument("--host", type=str, default=DEFAULT_HOST, help=f"Bind host address (default: {DEFAULT_HOST})")
    parser.add_argument("--buffer-size", type=int, default=DEFAULT_BUFFER_SIZE, help="Ring buffer capacity for live ROS messages")
    parser.add_argument("--kill-existing", action="store_true", help="Automatically kill any existing process holding the port")
    parser.add_argument("--debug", action="store_true", help="Enable Dash debug mode")
    return parser.parse_args()


def main():
    args = parse_args()
    print("=" * 72)
    print("   UR5 WEB-BASED PID PERFORMANCE DASHBOARD — PHASE 1")
    print("=" * 72)
    print(f" • Server Host: http://{args.host}:{args.port}")
    print(f" • Ring Buffer Capacity: {args.buffer_size} samples")
    print(f" • Safety Policy: Read-Only (no joint commands, no controller switching)")
    print(f" • Mode: Passive ROS 2 subscriber + Historical CSV explorer")
    print("=" * 72)

    # Check if target port is already in use
    if is_port_in_use(args.host, args.port):
        if args.kill_existing:
            print(f"⚠️ Port {args.port} in use. Terminating stale process (--kill-existing)...")
            kill_process_on_port(args.port)
            import time
            time.sleep(1.0)
        else:
            print(f"\n❌ Port {args.port} is already in use by another program.")
            print(f"   • To free port {args.port}: fuser -k {args.port}/tcp")
            print(f"   • Or re-run with: python3 ur5_web_dashboard/run_dashboard.py --port {args.port} --kill-existing")
            print(f"   • Or use another port: python3 ur5_web_dashboard/run_dashboard.py --port {args.port + 1}\n")
            sys.exit(1)

    app = create_app(buffer_size=args.buffer_size)

    def shutdown_handler(signum, frame):
        print("\n[Dashboard] Gracefully shutting down ROS monitor and web server...")
        if hasattr(app, "ros_monitor") and app.ros_monitor:
            app.ros_monitor.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    print(f"🚀 Dashboard running at: http://{args.host}:{args.port}/")
    print("Press Ctrl+C to terminate cleanly.")
    try:
        app.run(host=args.host, port=args.port, debug=args.debug)
    except OSError as e:
        if "Address already in use" in str(e) or e.errno == 98:
            print(f"\n❌ Error: Port {args.port} is already in use.")
            print(f"Run: fuser -k {args.port}/tcp")
        else:
            print(f"\n❌ Server error: {e}")
    finally:
        if hasattr(app, "ros_monitor") and app.ros_monitor:
            app.ros_monitor.stop()


if __name__ == "__main__":
    main()

