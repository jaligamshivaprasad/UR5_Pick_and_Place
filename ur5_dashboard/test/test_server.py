import pytest

from ur5_dashboard.server import FastAPI, build_app


@pytest.mark.skipif(FastAPI is not None, reason="dependency-specific diagnostic test")
def test_server_reports_missing_optional_web_dependencies():
    with pytest.raises(RuntimeError, match="fastapi and uvicorn"):
        build_app()
