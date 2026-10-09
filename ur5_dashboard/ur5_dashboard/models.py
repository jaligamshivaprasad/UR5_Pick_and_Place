from dataclasses import asdict, dataclass, field
from typing import Dict, Optional


@dataclass
class Check:
    name: str
    ok: bool = False
    detail: str = "waiting"
    age_s: Optional[float] = None


@dataclass
class DashboardStatus:
    state: str = "Stopped"
    launch_owned: bool = False
    launch_pid: Optional[int] = None
    started_at: Optional[str] = None
    message: str = "Simulation is stopped"
    trajectory_state: str = "Idle"
    trajectory_pid: Optional[int] = None
    recording_state: str = "Stopped"
    recording_pid: Optional[int] = None
    recording_path: Optional[str] = None
    checks: Dict[str, Check] = field(default_factory=dict)
    logs: list[str] = field(default_factory=list)

    def jsonable(self):
        value = asdict(self)
        value["checks"] = {name: asdict(check) for name, check in self.checks.items()}
        return value
