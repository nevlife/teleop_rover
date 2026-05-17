import threading
import time
from dataclasses import dataclass

# NetworkStatus.status_code is int8: 0=normal, 1=heartbeat_delay, 2=socket_error.
# bridge_flag values: 0=normal, 1=server_estop, 2=(unused/other),
# 3=heartbeat_timeout, 4=control_timeout. Keep flag 1 distinct from 3 by
# routing 1→2 (treat server_estop as a non-network "other"-class condition is
# not appropriate; surface it as socket_error-equivalent). Flag 4 collapses
# to heartbeat_delay since NetworkStatus only has three codes.
_STATUS_MAP = {0: 0, 1: 2, 3: 1, 4: 1}


@dataclass
class HealthState:
    bridge_flag: int
    flag_changed: bool
    connected: bool
    status_code: int


class HealthMonitor:

    def __init__(self, hb_timeout: float, ctrl_timeout: float):
        self._hb_timeout = hb_timeout
        self._ctrl_timeout = ctrl_timeout
        self._prev_flag = 0
        self._lock = threading.Lock()
        self._teleop_in_idle_warned = False

    def current_flag(self) -> int:
        with self._lock:
            return self._prev_flag

    def note_teleop_in_idle(self, logger) -> None:
        with self._lock:
            if self._teleop_in_idle_warned:
                return
            self._teleop_in_idle_warned = True
        logger.warning(
            "Teleop command received while current_mode == -1 (idle); "
            "command will be queued but no actuation expected until mode "
            "transitions to remote_drive (2)."
        )

    def evaluate(
        self,
        last_hb_time: float | None,
        last_ctrl_time: float,
        current_mode: int,
        server_estop: bool,
    ) -> HealthState:
        now = time.monotonic()

        if server_estop:
            flag = 1
        elif last_hb_time is not None and (now - last_hb_time) > self._hb_timeout:
            flag = 3
        elif (
            current_mode == 2
            and last_ctrl_time > 0
            and (now - last_ctrl_time) > self._ctrl_timeout
        ):
            flag = 4
        else:
            flag = 0

        with self._lock:
            changed = flag != self._prev_flag
            self._prev_flag = flag

        connected = flag == 0 and last_hb_time is not None
        status_code = _STATUS_MAP.get(flag, 2)

        return HealthState(
            bridge_flag=flag,
            flag_changed=changed,
            connected=connected,
            status_code=status_code,
        )
