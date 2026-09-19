from __future__ import annotations

import asyncio
import json
import os
import signal
import time
from collections.abc import Callable
from datetime import UTC, datetime
from pathlib import Path

SUPPORTED_ROBOTS = ("robot_1",)
_STAGE_MARKERS = ((1, "[1단계"), (2, "[2단계"), (3, "[3단계"), (4, "[4단계"))


def parse_event(line: str) -> dict | None:
    """Map one pipeline stdout line to a progress event, or None."""
    for stage, marker in _STAGE_MARKERS:
        if marker in line:
            return {"kind": "stage", "stage_index": stage - 1, "message": line.strip("=\n ")}
    if "회차]" in line:
        return {"kind": "message", "message": line.strip()}
    if "최종:" in line:
        return {"kind": "final", "message": line.strip()}
    if "맵 저장 완료" in line:
        return {"kind": "saved"}
    if "좌표계 손상" in line or "맵 저장 실패" in line:
        return {"kind": "corrupt"}
    return None


def build_env(overlay_dir: Path, domain: int) -> dict[str, str]:
    env = dict(os.environ)
    env.pop("ROS_LOCALHOST_ONLY", None)
    env["ROS_DOMAIN_ID"] = str(domain)
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "SUBNET"
    env["PINKY_TRACK_W"] = "0"
    env["PINKY_TRACK_H"] = "0"
    opt = Path(overlay_dir) / "opt" / "ros" / "jazzy"
    usr_lib = Path(overlay_dir) / "usr" / "lib"
    env["AMENT_PREFIX_PATH"] = os.pathsep.join(filter(None, [str(opt), env.get("AMENT_PREFIX_PATH", "")]))
    env["LD_LIBRARY_PATH"] = os.pathsep.join(filter(None, [str(opt / "lib"), str(usr_lib / "x86_64-linux-gnu"), str(usr_lib), env.get("LD_LIBRARY_PATH", "")]))
    env["PYTHONPATH"] = os.pathsep.join(filter(None, [str(opt / "lib" / "python3.12" / "site-packages"), str(overlay_dir / "usr" / "lib" / "python3" / "dist-packages"), env.get("PYTHONPATH", "")]))
    env["PATH"] = f"{opt / 'bin'}:" + env.get("PATH", "")
    return env


class MappingRunner:
    """Owns one lap585 mapping pipeline subprocess (robot_1 only)."""

    def __init__(
        self,
        lap585_dir: Path,
        overlay_dir: Path,
        state_dir: Path,
        domain: int = 12,
        preflight_state: Callable[[], tuple[bool, bool]] | None = None,
        on_event: Callable[[dict], None] | None = None,
        on_complete: Callable[[], None] | None = None,
    ) -> None:
        self.lap585_dir = Path(lap585_dir)
        self.overlay_dir = Path(overlay_dir)
        self.state_dir = Path(state_dir)
        self.domain = domain
        self.preflight_state = preflight_state
        self.on_event = on_event
        self.on_complete = on_complete
        self.log_path = self.state_dir / "mapping" / "last_run.log"
        self._proc: asyncio.subprocess.Process | None = None
        self._pump: asyncio.Task | None = None
        self._cancelled = False
        self._session: dict = {"state": "IDLE", "stage_index": -1, "stage": "", "message": "", "reason": ""}

    def status(self) -> dict:
        return dict(self._session)

    @property
    def _run_file(self) -> Path:
        return self.state_dir / "mapping" / "run.json"

    def detect_orphan(self) -> dict | None:
        """A pipeline left running by a previous backend process."""
        try:
            record = json.loads(self._run_file.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return None
        pid = int(record.get("pid", 0))
        if pid <= 0:
            return None
        try:
            os.kill(pid, 0)
        except OSError:
            return None
        return record

    async def kill_orphan(self) -> dict | None:
        orphan = self.detect_orphan()
        if orphan is None:
            return None
        try:
            os.killpg(os.getpgid(int(orphan["pid"])), signal.SIGKILL)
        except (OSError, ProcessLookupError):
            try:
                os.kill(int(orphan["pid"]), signal.SIGKILL)
            except OSError:
                pass
        return orphan

    async def start(self, robot_id: str, lease_id: str | None = None) -> dict:
        if robot_id not in SUPPORTED_ROBOTS:
            raise ValueError("ROBOT_NOT_SUPPORTED")
        if self._session["state"] == "RUNNING":
            raise ValueError("MAPPING_ACTIVE")
        script = self.lap585_dir / "run_lap_real.sh"
        if not script.exists():
            raise ValueError("MAPPING_RUNNER_ERROR")
        online, scan_fresh = self.preflight_state() if self.preflight_state else (True, True)
        if not online:
            raise ValueError("ROSBRIDGE_OFFLINE")
        if not scan_fresh:
            raise ValueError("SCAN_STALE")
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._cancelled = False
        self._session = {
            "state": "RUNNING", "robot_id": robot_id, "stage_index": -1, "stage": "",
            "message": "", "reason": "", "started_at": datetime.now(UTC).isoformat(),
        }
        self._proc = await asyncio.create_subprocess_exec(
            "bash", str(script), cwd=str(self.lap585_dir), env=build_env(self.overlay_dir, self.domain),
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT, start_new_session=True,
        )
        self._session["pid"] = self._proc.pid
        self._run_file.write_text(json.dumps({"pid": self._proc.pid, "started_at": self._session["started_at"]}), encoding="utf-8")
        self._pump = asyncio.ensure_future(self._pump_output())
        return await self._wait()

    async def _pump_output(self) -> None:
        assert self._proc is not None and self._proc.stdout is not None
        with self.log_path.open("ab") as log:
            async for raw in self._proc.stdout:
                log.write(raw)
                log.flush()
                line = raw.decode("utf-8", errors="replace")
                event = parse_event(line)
                if event is None:
                    continue
                self._apply(event)

    def _apply(self, event: dict) -> None:
        kind = event["kind"]
        if kind == "stage":
            self._session["stage_index"] = event["stage_index"]
            self._session["stage"] = event["message"]
            self._session["message"] = ""
        elif kind in ("message", "final"):
            self._session["message"] = event["message"]
        elif kind == "saved":
            self._session["saved"] = True
        elif kind == "corrupt":
            self._session["corrupt"] = True
        if self.on_event:
            self.on_event(event)

    async def _wait(self) -> dict:
        if self._pump is not None:
            try:
                await self._pump
            except BaseException:
                pass
        if self._proc is not None and self._proc.returncode is None:
            await self._proc.wait()
        return self._finish()

    def _finish(self) -> dict:
        rc = self._proc.returncode if self._proc else None
        if self._cancelled:
            self._session["state"] = "PAUSED"
            self._session["reason"] = ""
        elif rc == 0 and self._session.get("saved"):
            self._session["state"] = "COMPLETED"
            self._session["reason"] = ""
            if self.on_complete:
                self.on_complete()
        else:
            self._session["state"] = "FAILED"
            self._session["reason"] = "MAP_CORRUPTED" if self._session.get("corrupt") else "RUNNER_EXIT_" + str(rc)
        self._session["finished_at"] = datetime.now(UTC).isoformat()
        self._run_file.unlink(missing_ok=True)
        return self.status()

    async def cancel(self) -> dict:
        self._cancelled = True
        proc = self._proc
        if proc is not None and proc.returncode is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
            for _ in range(50):
                if proc.returncode is not None:
                    break
                await asyncio.sleep(0.1)
            else:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        if self._pump is not None:
            try:
                await self._pump
            except BaseException:
                pass
        if self._session["state"] == "RUNNING":
            return self._finish()
        return self.status()
