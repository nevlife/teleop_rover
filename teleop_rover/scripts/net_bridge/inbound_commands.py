import json
import math
import threading
import time
from dataclasses import dataclass

from teleop_contracts import (
    IncompatibleSchemaError,
    parse_envelope,
    TOPIC_PONG,
    key_for,
)

from .zenoh_transport import ZenohTransport


_MAX_CONTROL_PAYLOAD = 64 * 1024
_STEER_CLAMP_RAD = 0.7
_OVERSIZE_LOG_INTERVAL = 5.0


@dataclass
class PendingCommands:
    teleop: tuple | None = None
    estop: bool | None = None
    mode: int | None = None


class InboundHandler:

    def __init__(
        self, vehicle_id: str, transport: ZenohTransport,
        logger, wheelbase: float = 0.650,
    ):
        self._vid = vehicle_id
        self._transport = transport
        self._logger = logger
        # Caller validates wheelbase > 0 before construction.
        self._wheelbase = wheelbase

        self._lock = threading.Lock()
        self._pending_teleop: tuple | None = None
        self._pending_estop: bool | None = None
        self._pending_mode: int | None = None
        self._last_teleop_seq: int | None = None
        self._last_oversize_log: float = 0.0

        self.last_hb_time: float | None = None
        self.last_ctrl_time: float = 0.0
        # Visibility counters for the periodic [BOT] snapshot. Useful for
        # confirming the net_bridge is actually subscribed and dispatching.
        self.ping_rx_count: int = 0
        self.teleop_rx_count: int = 0

    def _reject_oversize(self, kind: str, size: int) -> bool:
        now = time.monotonic()
        if now - self._last_oversize_log > _OVERSIZE_LOG_INTERVAL:
            self._last_oversize_log = now
            self._logger.warning(
                f"{kind} payload too large ({size} > {_MAX_CONTROL_PAYLOAD}); dropping"
            )
        return True

    def on_telemetry_ping(self, sample):
        """Server → bot RTT probe + e-stop heartbeat watchdog.

        Echoes the same `ts` back as `telemetry_pong` and resets the
        last_hb_time so the HealthMonitor knows the control plane is alive.
        """
        payload = bytes(sample.payload)
        if len(payload) > _MAX_CONTROL_PAYLOAD:
            self._reject_oversize("telemetry_ping", len(payload))
            return
        try:
            _v, data = parse_envelope(payload)
            ts = data.get("ts")
            if ts is None:
                return
        except IncompatibleSchemaError as e:
            self._logger.warning(f"telemetry_ping drop: {e}")
            return
        except Exception as e:
            self._logger.warning(f"telemetry_ping parse error: {e}")
            return
        with self._lock:
            first = self.last_hb_time is None
            self.last_hb_time = time.monotonic()
            self.ping_rx_count += 1
        try:
            # Echo the client's original ping ts as the pong body; the
            # transport adds the envelope's own ts on top. Client measures
            # RTT against this body-level ts (i.e. against its own ping
            # wall clock), so the transport stamp is irrelevant for RTT.
            self._transport.put(
                key_for(self._vid, TOPIC_PONG), {"ts": ts}
            )
            if first:
                self._logger.info(
                    "First telemetry_ping received → telemetry_pong published"
                )
        except Exception as e:
            self._logger.warning(f"telemetry_pong publish error: {e}")

    def on_teleop(self, sample):
        payload = bytes(sample.payload)
        if len(payload) > _MAX_CONTROL_PAYLOAD:
            self._reject_oversize("teleop", len(payload))
            return
        try:
            _v, data = parse_envelope(payload)
        except IncompatibleSchemaError as e:
            self._logger.warning(f"teleop drop: {e}")
            return
        except Exception as e:
            self._logger.warning(f"teleop JSON parse error: {e}")
            return
        seq = data.get("seq")
        if seq is not None:
            try:
                seq_int = int(seq)
            except (TypeError, ValueError):
                self._logger.warning(f"teleop invalid seq value: {seq!r}")
                return
            with self._lock:
                if (
                    self._last_teleop_seq is not None
                    and seq_int < self._last_teleop_seq
                ):
                    return
                self._last_teleop_seq = seq_int
        lx = max(min(float(data.get("linear_x", 0.0)), 2.0), -2.0)
        steer = float(data.get("steer_angle", 0.0))
        steer = max(min(steer, _STEER_CLAMP_RAD), -_STEER_CLAMP_RAD)
        if abs(steer) < 1e-6:
            az = 0.0
        elif abs(lx) < 0.05:
            az = steer
        else:
            az = lx * math.tan(steer) / self._wheelbase
        az = max(min(az, 2.0), -2.0)
        now = time.monotonic()
        with self._lock:
            self.teleop_rx_count += 1
            self._pending_teleop = (lx, az)
            self.last_ctrl_time = now

    def on_estop(self, sample):
        payload = bytes(sample.payload)
        if len(payload) > _MAX_CONTROL_PAYLOAD:
            self._reject_oversize("estop", len(payload))
            return
        try:
            _v, data = parse_envelope(payload)
        except IncompatibleSchemaError as e:
            self._logger.warning(f"estop drop: {e}")
            return
        except Exception as e:
            self._logger.warning(f"estop JSON parse error: {e}")
            return
        active = data.get("active", False)
        if not isinstance(active, bool):
            self._logger.warning(
                f"estop rejected: 'active' must be bool, got {type(active).__name__}"
            )
            return
        with self._lock:
            self._pending_estop = active

    def on_cmd_mode(self, sample):
        payload = bytes(sample.payload)
        if len(payload) > _MAX_CONTROL_PAYLOAD:
            self._reject_oversize("cmd_mode", len(payload))
            return
        try:
            _v, data = parse_envelope(payload)
        except IncompatibleSchemaError as e:
            self._logger.warning(f"cmd_mode drop: {e}")
            return
        except Exception as e:
            self._logger.warning(f"cmd_mode JSON parse error: {e}")
            return
        mode = data.get("mode", -1)
        # bool is a subclass of int — reject it explicitly.
        if isinstance(mode, bool) or not isinstance(mode, int):
            self._logger.warning(
                f"cmd_mode rejected: 'mode' must be int, got {type(mode).__name__}"
            )
            return
        with self._lock:
            self._pending_mode = mode

    def drain_pending(self) -> PendingCommands:
        with self._lock:
            cmds = PendingCommands(
                teleop=self._pending_teleop,
                estop=self._pending_estop,
                mode=self._pending_mode,
            )
            self._pending_teleop = None
            self._pending_estop = None
            self._pending_mode = None
        return cmds
