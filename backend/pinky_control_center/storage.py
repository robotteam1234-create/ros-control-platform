from __future__ import annotations

import base64
import json
import hashlib
import hmac
import logging
from importlib import resources
import secrets
import sqlite3
import threading
from collections.abc import Callable
from datetime import UTC, datetime, timedelta
from pathlib import Path
from uuid import UUID, uuid4

from pinky_control_center.models import ActiveSettings, Alert, AlertState, ControlLease, UserInfo, UserRole

Clock = Callable[[], datetime]
logger = logging.getLogger(__name__)
HISTORY_RETENTION = timedelta(days=30)
_HISTORY_EXCLUDED_KEYS = {"path", "paths", "trail", "waypoints", "scan", "costmap", "camera", "frame", "frames", "telemetry"}


def utc_now() -> datetime:
    return datetime.now(UTC)


def _timestamp(value: datetime) -> str:
    return value.astimezone(UTC).isoformat()


def _parse_timestamp(value: str) -> datetime:
    return datetime.fromisoformat(value).astimezone(UTC)


def _history_payload(value: object) -> object:
    """Keep operational outcomes, never high-rate or route/frame payloads."""
    if isinstance(value, dict):
        return {str(key): _history_payload(item) for key, item in value.items() if str(key).lower() not in _HISTORY_EXCLUDED_KEYS}
    if isinstance(value, list):
        return [_history_payload(item) for item in value]
    return value


def hash_password(password: str) -> str:
    """Use a salted, memory-hard stdlib hash; plaintext never reaches SQLite."""
    salt = secrets.token_bytes(16)
    digest = hashlib.scrypt(password.encode(), salt=salt, n=2**14, r=8, p=1, dklen=32)
    return "scrypt$16384$8$1$%s$%s" % (base64.b64encode(salt).decode(), base64.b64encode(digest).decode())


def verify_password(password: str, encoded: str) -> bool:
    try:
        algorithm, n, r, p, salt_text, digest_text = encoded.split("$")
        if algorithm != "scrypt":
            return False
        salt = base64.b64decode(salt_text)
        expected = base64.b64decode(digest_text)
        actual = hashlib.scrypt(password.encode(), salt=salt, n=int(n), r=int(r), p=int(p), dklen=len(expected))
    except (ValueError, TypeError):
        return False
    return hmac.compare_digest(actual, expected)


class Storage:
    def __init__(self, database_path: Path, clock: Clock = utc_now, lease_end_hook: Callable[[str], None] | None = None) -> None:
        self.database_path = database_path
        self.clock = clock
        self.lease_end_hook = lease_end_hook or (lambda _reason: None)
        self.database_path.parent.mkdir(parents=True, exist_ok=True)
        self.connection = sqlite3.connect(database_path, check_same_thread=False)
        self.connection.row_factory = sqlite3.Row
        self._command_lock = threading.RLock()
        self._audit_lock = threading.Lock()
        self.connection.execute("PRAGMA journal_mode=WAL")
        self.connection.execute("PRAGMA foreign_keys=ON")
        self._migrate()
        # Audit uses its own no-wait connection so a locked history table never
        # delays an adapter command or the pair-wide watchdog stop.
        self.audit_connection = sqlite3.connect(database_path, check_same_thread=False, timeout=0)
        self.audit_connection.execute("PRAGMA journal_mode=WAL")
        self.audit_connection.execute("PRAGMA foreign_keys=ON")

    def _migrate(self) -> None:
        sql = resources.files("pinky_control_center").joinpath("migrations", "001_initial.sql").read_text(encoding="utf-8")
        with self.connection:
            self.connection.executescript(sql)
            columns = {row["name"] for row in self.connection.execute("PRAGMA table_info(control_leases)")}
            if "session_token_hash" not in columns:
                self.connection.execute("ALTER TABLE control_leases ADD COLUMN session_token_hash TEXT NOT NULL DEFAULT 'legacy'")
            self.connection.execute("INSERT OR IGNORE INTO schema_migrations(version, applied_at) VALUES(1, ?)", (_timestamp(self.clock()),))
            applied = {row["version"] for row in self.connection.execute("SELECT version FROM schema_migrations")}
            if 2 not in applied:
                sql = resources.files("pinky_control_center").joinpath("migrations", "002_commands.sql").read_text(encoding="utf-8")
                self.connection.executescript(sql)
                self.connection.execute("INSERT INTO schema_migrations(version, applied_at) VALUES(2, ?)", (_timestamp(self.clock()),))
            if 3 not in applied:
                sql = resources.files("pinky_control_center").joinpath("migrations", "003_command_idempotency_window.sql").read_text(encoding="utf-8")
                self.connection.executescript(sql)
                self.connection.execute("INSERT INTO schema_migrations(version, applied_at) VALUES(3, ?)", (_timestamp(self.clock()),))
            if 4 not in applied:
                sql = resources.files("pinky_control_center").joinpath("migrations", "004_missions.sql").read_text(encoding="utf-8")
                self.connection.executescript(sql)
                self.connection.execute("INSERT INTO schema_migrations(version, applied_at) VALUES(4, ?)", (_timestamp(self.clock()),))
            if 5 not in applied:
                sql = resources.files("pinky_control_center").joinpath("migrations", "005_alerts.sql").read_text(encoding="utf-8")
                self.connection.executescript(sql)
                self.connection.execute("INSERT INTO schema_migrations(version, applied_at) VALUES(5, ?)", (_timestamp(self.clock()),))
            if 6 not in applied:
                sql = resources.files("pinky_control_center").joinpath("migrations", "006_settings.sql").read_text(encoding="utf-8")
                self.connection.executescript(sql)
                self.connection.execute("INSERT INTO schema_migrations(version, applied_at) VALUES(6, ?)", (_timestamp(self.clock()),))
            if 7 not in applied:
                sql = resources.files("pinky_control_center").joinpath("migrations", "007_history.sql").read_text(encoding="utf-8")
                self.connection.executescript(sql)
                self.connection.execute("INSERT INTO schema_migrations(version, applied_at) VALUES(7, ?)", (_timestamp(self.clock()),))

    def record_history_safe(self, *, event_type: str, user: UserInfo | None = None, user_id: str | None = None, robot_id: str | None = None, mission_id: str | None = None, payload: dict[str, object] | None = None, dedupe_key: str | None = None) -> bool:
        """Best-effort audit write: loss of history must not affect control or safety."""
        body = dict(_history_payload(payload or {}))
        if dedupe_key:
            body["_dedupe_key"] = dedupe_key
        if not self._audit_lock.acquire(blocking=False):
            logger.warning("audit history degraded: storage is busy")
            return False
        try:
            with self.audit_connection:
                self.audit_connection.execute("DELETE FROM history_events WHERE occurred_at<?", (_timestamp(self.clock() - HISTORY_RETENTION),))
                self.audit_connection.execute("INSERT OR IGNORE INTO history_events(user_id,event_type,robot_id,mission_id,occurred_at,payload_json) VALUES(?,?,?,?,?,?)", (str(user.user_id) if user else user_id, event_type, robot_id, mission_id, _timestamp(self.clock()), json.dumps(body, sort_keys=True, default=str)))
            return True
        except (sqlite3.Error, OSError, ValueError) as error:
            logger.warning("audit history degraded: %s", error)
            return False
        finally:
            self._audit_lock.release()

    def history(self, user: UserInfo, *, event_type: str | None, robot_id: str | None, mission_id: str | None, from_time: str, to_time: str, limit: int, cursor: int | None = None) -> tuple[list[dict[str, object]], int | None]:
        query = "SELECT id,event_type,robot_id,mission_id,occurred_at,payload_json FROM history_events WHERE (user_id=? OR user_id IS NULL) AND occurred_at>=? AND occurred_at<=?"
        args: list[object] = [str(user.user_id), from_time, to_time]
        if event_type: query += " AND event_type=?"; args.append(event_type)
        if robot_id: query += " AND robot_id=?"; args.append(robot_id)
        if mission_id: query += " AND mission_id=?"; args.append(mission_id)
        if cursor is not None: query += " AND id<?"; args.append(cursor)
        query += " ORDER BY id DESC LIMIT ?"; args.append(limit + 1)
        with self._command_lock:
            rows = self.connection.execute(query, args).fetchall()
        more, rows = len(rows) > limit, rows[:limit]
        items = []
        for row in rows:
            payload = json.loads(row["payload_json"]); payload.pop("_dedupe_key", None)
            items.append({"event_id": row["id"], "event_type": row["event_type"], "robot_id": row["robot_id"], "mission_id": row["mission_id"], "occurred_at": row["occurred_at"], "payload": payload})
        return items, (int(rows[-1]["id"]) if more and rows else None)

    def settings(self) -> ActiveSettings:
        with self._command_lock:
            row = self.connection.execute("SELECT version,values_json FROM active_settings WHERE id=1").fetchone()
        if row is None:  # Defensive for databases imported before migration tracking existed.
            raise RuntimeError("active settings are missing")
        return ActiveSettings.model_validate({"version": row["version"], **json.loads(row["values_json"])})

    def update_settings(self, expected_version: int, values: ActiveSettings) -> ActiveSettings | None:
        """Compare and swap the active settings; None means a stale client."""
        payload = values.model_dump(mode="json", exclude={"version"})
        with self._command_lock, self.connection:
            result = self.connection.execute(
                "UPDATE active_settings SET version=?, values_json=?, updated_at=? WHERE id=1 AND version=?",
                (expected_version + 1, json.dumps(payload, sort_keys=True), _timestamp(self.clock()), expected_version),
            )
        if result.rowcount != 1:
            return None
        return ActiveSettings.model_validate({"version": expected_version + 1, **payload})

    def upsert_alert(self, alert: Alert) -> Alert:
        payload = alert.model_dump(mode="json")
        scope = alert.robot_id or "system"
        now = _timestamp(self.clock())
        with self._command_lock, self.connection:
            self.connection.execute(
                """INSERT INTO alerts(id,code,scope,robot_id,state,severity,payload_json,acknowledged_at,acknowledged_by,created_at,updated_at)
                   VALUES(?,?,?,?,?,?,?,?,?,?,?)
                   ON CONFLICT(code,scope) DO UPDATE SET state=excluded.state,severity=excluded.severity,payload_json=excluded.payload_json,
                     acknowledged_at=excluded.acknowledged_at,acknowledged_by=excluded.acknowledged_by,updated_at=excluded.updated_at""",
                (str(alert.alert_id), alert.code, scope, alert.robot_id, alert.state.value, alert.severity.value, json.dumps(payload),
                 _timestamp(alert.acknowledged_at) if alert.acknowledged_at else None, alert.acknowledged_by, now, now),
            )
        return alert

    def alert(self, alert_id: str) -> Alert | None:
        with self._command_lock:
            row = self.connection.execute("SELECT payload_json FROM alerts WHERE id=?", (alert_id,)).fetchone()
        return None if row is None else Alert.model_validate(json.loads(row["payload_json"]))

    def alerts(self, *, state: AlertState | None = None, severity: str | None = None, robot_id: str | None = None, limit: int = 100, cursor: int = 0) -> tuple[list[Alert], int | None]:
        query = "SELECT payload_json FROM alerts WHERE 1=1"
        args: list[object] = []
        if state is not None:
            query += " AND state=?"; args.append(state.value)
        if severity is not None:
            query += " AND severity=?"; args.append(severity)
        if robot_id is not None:
            query += " AND robot_id=?"; args.append(robot_id)
        query += " ORDER BY updated_at DESC,id DESC LIMIT ? OFFSET ?"; args.extend((limit + 1, cursor))
        with self._command_lock:
            rows = self.connection.execute(query, args).fetchall()
        more = len(rows) > limit
        return [Alert.model_validate(json.loads(row["payload_json"])) for row in rows[:limit]], cursor + limit if more else None

    def acknowledge_alert(self, alert_id: str, username: str) -> Alert | None:
        alert = self.alert(alert_id)
        if alert is None:
            return None
        if alert.acknowledged_at is None:
            alert = alert.model_copy(update={"acknowledged_at": self.clock(), "acknowledged_by": username})
            self.upsert_alert(alert)
        return alert

    def create_mission(self, user: UserInfo, payload: dict[str, object]) -> dict[str, object]:
        mission_id, now = str(uuid4()), _timestamp(self.clock())
        with self._command_lock, self.connection:
            self.connection.execute("INSERT INTO missions(id,user_id,payload_json,state,created_at,updated_at) VALUES(?,?,?,?,?,?)", (mission_id, str(user.user_id), json.dumps(payload), payload["state"], now, now))
        mission = {"mission_id": mission_id, **payload, "created_at": now, "updated_at": now}
        self.record_history_safe(event_type="MISSION_CREATED", user=user, mission_id=mission_id, payload={"state": payload["state"], "name": payload.get("name")}, dedupe_key=f"mission:{mission_id}:created")
        return mission

    def create_mission_idempotent(self, user: UserInfo, request_id: UUID, payload: dict[str, object]) -> dict[str, object]:
        # The request claim and mission/result materialization share the same process lock.
        # A second caller re-reads the first caller's durable result rather than creating a twin.
        with self._command_lock:
            command = self.create_command(user, request_id, "missions", {"operation": "mission_create", "payload": payload})
            existing = command.get("result", {}).get("mission") if isinstance(command.get("result"), dict) else None
            if existing:
                return existing
            mission = self.create_mission(user, payload)
            self.set_command_result(str(command["command_id"]), {"mission": mission})
            self.set_command_state(str(command["command_id"]), "SUCCEEDED")
            return mission

    def mission(self, mission_id: str, user: UserInfo) -> dict[str, object] | None:
        with self._command_lock:
            row = self.connection.execute("SELECT id,payload_json,created_at,updated_at FROM missions WHERE id=? AND user_id=?", (mission_id, str(user.user_id))).fetchone()
        return None if row is None else {"mission_id": row["id"], **json.loads(row["payload_json"]), "created_at": row["created_at"], "updated_at": row["updated_at"]}

    def missions(self, user: UserInfo, *, state: str | None, limit: int, cursor: int, from_time: str | None = None, to_time: str | None = None) -> tuple[list[dict[str, object]], int | None]:
        query = "SELECT id,payload_json,created_at,updated_at FROM missions WHERE user_id=?"
        args: list[object] = [str(user.user_id)]
        if state: query += " AND state=?"; args.append(state)
        if from_time: query += " AND created_at>=?"; args.append(from_time)
        if to_time: query += " AND created_at<=?"; args.append(to_time)
        query += " ORDER BY created_at ASC,id ASC LIMIT ? OFFSET ?"; args.extend([limit + 1, cursor])
        with self._command_lock:
            rows = self.connection.execute(query, args).fetchall()
        more = len(rows) > limit; rows = rows[:limit]
        return ([{"mission_id": row["id"], **json.loads(row["payload_json"]), "created_at": row["created_at"], "updated_at": row["updated_at"]} for row in rows], cursor + limit if more else None)

    def update_mission(self, mission_id: str, payload: dict[str, object]) -> None:
        now = _timestamp(self.clock())
        with self._command_lock, self.connection:
            previous = self.connection.execute("SELECT user_id,payload_json FROM missions WHERE id=?", (mission_id,)).fetchone()
            self.connection.execute("UPDATE missions SET payload_json=?, state=?, updated_at=? WHERE id=?", (json.dumps(payload), payload["state"], now, mission_id))
        if previous is not None and json.loads(previous["payload_json"]).get("state") != payload.get("state"):
            self.record_history_safe(event_type="MISSION_TRANSITION", user_id=previous["user_id"], mission_id=mission_id, payload={"state": payload.get("state"), "failure_code": payload.get("failure_code")})

    def create_command(self, user: UserInfo, request_id: UUID, target: str, payload: dict[str, object]) -> dict[str, object]:
        payload_text = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        payload_hash = hashlib.sha256(payload_text.encode()).hexdigest()
        # The RLock protects this shared sqlite connection in a threaded ASGI worker;
        # BEGIN IMMEDIATE provides the corresponding database write claim.
        with self._command_lock:
            try:
                self.connection.execute("BEGIN IMMEDIATE")
                now_value = self.clock()
                cutoff = _timestamp(now_value - timedelta(hours=24))
                row = self.connection.execute("SELECT id,payload_hash,target,state,result_json FROM commands WHERE user_id=? AND request_id=? AND created_at > ? ORDER BY created_at DESC LIMIT 1", (str(user.user_id), str(request_id), cutoff)).fetchone()
                if row:
                    if row["payload_hash"] != payload_hash:
                        raise ValueError("REQUEST_ID_CONFLICT")
                    self.connection.commit()
                    return {
                        "command_id": row["id"], "target": row["target"], "state": row["state"],
                        "result": json.loads(row["result_json"] or "{}"),
                    }
                command_id = str(uuid4())
                now = _timestamp(now_value)
                self.connection.execute("INSERT INTO commands(id,user_id,request_id,created_at,payload_hash,target,state,result_json,updated_at) VALUES(?,?,?,?,?,?,?,?,?)", (command_id, str(user.user_id), str(request_id), now, payload_hash, target, "ACCEPTED", "{}", now))
                self.connection.commit()
                self.record_history_safe(event_type="COMMAND_ACCEPTED", user=user, robot_id=target if target in {"robot_1", "robot_2"} else None, payload={"command_id": command_id, "target": target, "operation": payload.get("operation")}, dedupe_key=f"command:{command_id}:accepted")
                return {"command_id": command_id, "target": target, "state": "ACCEPTED", "result": {}}
            except Exception:
                self.connection.rollback()
                raise

    def command(self, command_id: str, user: UserInfo) -> dict[str, object] | None:
        with self._command_lock:
            row = self.connection.execute("SELECT id,target,state,result_json FROM commands WHERE id=? AND user_id=?", (command_id, str(user.user_id))).fetchone()
        if row is None: return None
        result = {"command_id": row["id"], "target": row["target"], "state": row["state"]}
        result.update(json.loads(row["result_json"] or "{}")); return result

    def set_command_result(self, command_id: str, result: dict[str, object]) -> None:
        with self._command_lock, self.connection:
            self.connection.execute("UPDATE commands SET result_json=?, updated_at=? WHERE id=?", (json.dumps(result), _timestamp(self.clock()), command_id))
            row = self.connection.execute("SELECT user_id,target,state FROM commands WHERE id=?", (command_id,)).fetchone()
        if row:
            fingerprint = hashlib.sha256(json.dumps(result, sort_keys=True, default=str).encode()).hexdigest()[:16]
            self.record_history_safe(event_type="COMMAND_RESULT", user_id=row["user_id"], robot_id=row["target"] if row["target"] in {"robot_1", "robot_2"} else None, payload={"command_id": command_id, "target": row["target"], "state": row["state"], "result": _history_payload(result)}, dedupe_key=f"command:{command_id}:result:{fingerprint}")

    def set_command_state(self, command_id: str, state: str) -> None:
        with self._command_lock, self.connection:
            self.connection.execute("UPDATE commands SET state=?, updated_at=? WHERE id=?", (state, _timestamp(self.clock()), command_id))
            row = self.connection.execute("SELECT user_id,target FROM commands WHERE id=?", (command_id,)).fetchone()
        if row:
            self.record_history_safe(event_type=f"COMMAND_{state}", user_id=row["user_id"], robot_id=row["target"] if row["target"] in {"robot_1", "robot_2"} else None, payload={"command_id": command_id, "target": row["target"], "state": state}, dedupe_key=f"command:{command_id}:state:{state}")

    def delete_command(self, command_id: str) -> None:
        with self._command_lock, self.connection:
            self.connection.execute("DELETE FROM commands WHERE id=?", (command_id,))

    def close(self) -> None:
        self.audit_connection.close()
        self.connection.close()

    def create_or_reset_user(self, username: str, password: str, role: UserRole) -> UserInfo:
        user_id = uuid4()
        with self._command_lock, self.connection:
            existing = self.connection.execute("SELECT id FROM users WHERE username=?", (username,)).fetchone()
            if existing:
                user_id = UUID(existing["id"])
                self.connection.execute("UPDATE users SET password_hash=?, role=? WHERE id=?", (hash_password(password), role.value, str(user_id)))
            else:
                self.connection.execute(
                    "INSERT INTO users(id, username, password_hash, role, created_at) VALUES(?,?,?,?,?)",
                    (str(user_id), username, hash_password(password), role.value, _timestamp(self.clock())),
                )
        return UserInfo(user_id=user_id, username=username, role=role)

    def authenticate(self, username: str, password: str) -> UserInfo | None:
        with self._command_lock:
            row = self.connection.execute("SELECT id, username, password_hash, role FROM users WHERE username=?", (username,)).fetchone()
        if row is None or not verify_password(password, row["password_hash"]):
            return None
        return UserInfo(user_id=UUID(row["id"]), username=row["username"], role=UserRole(row["role"]))

    def find_user(self, username: str) -> UserInfo | None:
        """Look up a user without verifying a password.

        Only used by the explicit auth-bypass login path
        (CONTROL_PLATFORM_AUTH_BYPASS=1). Never auto-creates users.
        """
        with self._command_lock:
            row = self.connection.execute("SELECT id, username, role FROM users WHERE username=?", (username,)).fetchone()
        if row is None:
            return None
        return UserInfo(user_id=UUID(row["id"]), username=row["username"], role=UserRole(row["role"]))

    def create_session(self, user: UserInfo) -> tuple[str, str]:
        token = secrets.token_urlsafe(32)
        csrf = secrets.token_urlsafe(32)
        token_hash = hashlib.sha256(token.encode()).hexdigest()
        expires_at = self.clock() + timedelta(hours=8)
        with self._command_lock, self.connection:
            self.connection.execute("DELETE FROM sessions WHERE expires_at <= ?", (_timestamp(self.clock()),))
            self.connection.execute(
                "INSERT INTO sessions(token_hash,user_id,csrf_token,expires_at,created_at) VALUES(?,?,?,?,?)",
                (token_hash, str(user.user_id), csrf, _timestamp(expires_at), _timestamp(self.clock())),
            )
        return token, csrf

    def session_user(self, token: str | None) -> tuple[UserInfo, str] | None:
        if not token:
            return None
        with self._command_lock:
            row = self.connection.execute(
                "SELECT users.id, users.username, users.role, sessions.csrf_token, sessions.expires_at "
                "FROM sessions JOIN users ON users.id=sessions.user_id WHERE sessions.token_hash=?",
                (hashlib.sha256(token.encode()).hexdigest(),),
            ).fetchone()
        if row is None or _parse_timestamp(row["expires_at"]) <= self.clock():
            return None
        return UserInfo(user_id=UUID(row["id"]), username=row["username"], role=UserRole(row["role"])), row["csrf_token"]

    @staticmethod
    def token_hash(token: str) -> str:
        return hashlib.sha256(token.encode()).hexdigest()

    def delete_session(self, token: str | None) -> None:
        if token:
            token_hash = self.token_hash(token)
            with self._command_lock, self.connection:
                lease = self.connection.execute("DELETE FROM control_leases WHERE session_token_hash=?", (token_hash,))
                self.connection.execute("DELETE FROM sessions WHERE token_hash=?", (token_hash,))
            if lease.rowcount:
                self.lease_end_hook("SESSION_LOGOUT")

    def acquire_lease(self, user: UserInfo, request_id: UUID, session_token: str = "direct") -> ControlLease:
        now = self.clock()
        session_token_hash = self.token_hash(session_token)
        with self._command_lock, self.connection:
            self.connection.execute("DELETE FROM control_leases WHERE expires_at <= ?", (_timestamp(now),))
            row = self.connection.execute("SELECT id,user_id,session_token_hash,expires_at FROM control_leases LIMIT 1").fetchone()
            if row and (row["user_id"] != str(user.user_id) or row["session_token_hash"] != session_token_hash):
                raise LeaseConflict("another operator holds the control lease")
            if row:
                lease_id = UUID(row["id"])
            else:
                lease_id = uuid4()
                self.connection.execute("INSERT INTO control_leases(id,user_id,session_token_hash,expires_at,request_id,created_at) VALUES(?,?,?,?,?,?)", (str(lease_id), str(user.user_id), session_token_hash, _timestamp(now), str(request_id), _timestamp(now)))
            expires_at = now + timedelta(seconds=3)
            self.connection.execute("UPDATE control_leases SET expires_at=?, request_id=? WHERE id=?", (_timestamp(expires_at), str(request_id), str(lease_id)))
        return ControlLease(lease_id=lease_id, expires_at=expires_at)

    def renew_lease(self, lease_id: UUID, user: UserInfo, request_id: UUID, session_token: str = "direct") -> ControlLease:
        now = self.clock()
        with self._command_lock:
            row = self.connection.execute("SELECT user_id,session_token_hash,expires_at FROM control_leases WHERE id=?", (str(lease_id),)).fetchone()
        if row is None or row["user_id"] != str(user.user_id) or row["session_token_hash"] != self.token_hash(session_token) or _parse_timestamp(row["expires_at"]) <= now:
            raise LeaseNotFound("control lease is missing or expired")
        return self.acquire_lease(user, request_id, session_token)

    def release_lease(self, lease_id: UUID, user: UserInfo, session_token: str = "direct") -> None:
        session_token_hash = self.token_hash(session_token)
        with self._command_lock, self.connection:
            result = self.connection.execute("DELETE FROM control_leases WHERE id=? AND user_id=? AND session_token_hash=?", (str(lease_id), str(user.user_id), session_token_hash))
        if result.rowcount != 1:
            raise LeaseNotFound("control lease is missing or owned by another user")
        self.lease_end_hook("LEASE_RELEASED")

    def owns_lease(self, lease_id: UUID, user: UserInfo, session_token: str) -> bool:
        with self._command_lock:
            row = self.connection.execute("SELECT user_id,session_token_hash,expires_at FROM control_leases WHERE id=?", (str(lease_id),)).fetchone()
        return bool(row and row["user_id"] == str(user.user_id) and row["session_token_hash"] == self.token_hash(session_token) and _parse_timestamp(row["expires_at"]) > self.clock())

    def expire_security(self) -> list[str]:
        """Remove elapsed security rows and report only losses of control authority.

        An expired login session that never owned the control lease is ordinary
        authentication cleanup.  It must not stop a robot controlled by another,
        still-valid session.
        """
        now = _timestamp(self.clock())
        with self._command_lock, self.connection:
            expired_leases = self.connection.execute("DELETE FROM control_leases WHERE expires_at <= ?", (now,)).rowcount
            expired_session_rows = self.connection.execute("SELECT token_hash FROM sessions WHERE expires_at <= ?", (now,)).fetchall()
            expired_session_hashes = [row["token_hash"] for row in expired_session_rows]
            session_owned_leases = 0
            if expired_session_hashes:
                placeholders = ",".join("?" for _ in expired_session_hashes)
                session_owned_leases = self.connection.execute(
                    f"DELETE FROM control_leases WHERE session_token_hash IN ({placeholders})",
                    expired_session_hashes,
                ).rowcount
            self.connection.execute("DELETE FROM sessions WHERE expires_at <= ?", (now,))
        reasons: list[str] = []
        if expired_leases:
            reasons.append("LEASE_EXPIRED")
        if session_owned_leases:
            reasons.append("SESSION_EXPIRED")
        return reasons


class LeaseConflict(Exception):
    pass


class LeaseNotFound(Exception):
    pass
