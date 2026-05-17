#!/usr/bin/env python3
import os
import re
import sys
import threading
import time

# ament's install layout places this script in `lib/<pkg>/` while the
# `net_bridge/` package directory lives alongside it under the same
# install prefix. Prepend the script's own directory so the sibling
# package import resolves at runtime. Known fragile pattern: depends on
# the installer copying both alongside each other.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Make the vendored `teleop_contracts` git submodule importable. Two
# scenarios:
#   * Source layout (direct `python net_bridge.py` for unit tests): the
#     submodule sits at <repo>/teleop_contracts/teleop_contracts/.
#   * Install layout (ros2 launch): CMakeLists.txt copies the inner
#     package directory to lib/teleop_rover/teleop_contracts, and the
#     script's own directory is already on sys.path (line 13), so
#     `import teleop_contracts` resolves with no extra shim.
# Clone with --recurse-submodules or run `git submodule update --init`.
_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_CONTRACTS = os.path.abspath(os.path.join(_HERE, "..", "..", "teleop_contracts"))
if os.path.isdir(_SRC_CONTRACTS) and _SRC_CONTRACTS not in sys.path:
    sys.path.insert(0, _SRC_CONTRACTS)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.callback_groups import (  # noqa: E402
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from geometry_msgs.msg import Twist  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402
from teleop_rover_msgs.msg import EStopStatus, CmdMode, MuxStatus  # noqa: E402

# system_monitor_msgs is an optional dependency: it provides CPU/RAM/GPU/
# disk/network telemetry but is independent from the control plane (ping,
# teleop, estop) and the video plane. When missing, log a warning and
# skip the resource subscriptions so net_bridge can still run on bots
# that don't have the system-monitor stack built.
try:
    from system_monitor_msgs.msg import (  # noqa: E402
        CpuMetrics,
        MemoryMetrics,
        DiskMetrics,
        NetworkMetrics,
        GpuMetrics,
    )
    _SYS_MON_AVAILABLE = True
except ImportError:  # pragma: no cover - depends on local install
    CpuMetrics = MemoryMetrics = DiskMetrics = NetworkMetrics = GpuMetrics = None  # type: ignore[assignment]
    _SYS_MON_AVAILABLE = False

from net_bridge import (  # noqa: E402
    ZenohTransport,
    InboundHandler,
    HealthMonitor,
    TelemetrySerializer,
)
from teleop_contracts import (  # noqa: E402
    TOPIC_BOT_HEARTBEAT,
    TOPIC_CMD,
    TOPIC_CMD_MODE_BOT,
    TOPIC_CPU,
    TOPIC_DISK,
    TOPIC_ESTOP_CMD,
    TOPIC_ESTOP_STATUS,
    TOPIC_GPU,
    TOPIC_MEM,
    TOPIC_MUX,
    TOPIC_NET,
    TOPIC_PING,
    TOPIC_PONG,
    TOPIC_TWIST,
    key_for,
)


_VID_RE = re.compile(r"^[A-Za-z0-9_-]+$")


class NetBridge(Node):
    def __init__(self):
        super().__init__("net_bridge")

        self.declare_parameter("telemetry_locator", "tcp/127.0.0.1:7447")
        self.declare_parameter("vehicle_id", "0")
        self.declare_parameter("heartbeat_timeout", 2.0)
        self.declare_parameter("control_timeout", 1.0)
        self.declare_parameter("vehicle_rate", 20.0)
        self.declare_parameter("resource_rate", 1.0)
        self.declare_parameter("wheelbase", 0.650)
        self.declare_parameter("speed_topic", "/vehicle/speed")
        self.declare_parameter("angle_topic", "/vehicle/steer_angle")

        locator = self.get_parameter("telemetry_locator").value
        vid = self.get_parameter("vehicle_id").value
        hb_timeout = self.get_parameter("heartbeat_timeout").value
        ctrl_timeout = self.get_parameter("control_timeout").value
        v_rate = self.get_parameter("vehicle_rate").value
        r_rate = self.get_parameter("resource_rate").value
        wheelbase = self.get_parameter("wheelbase").value
        speed_topic = self.get_parameter("speed_topic").value
        angle_topic = self.get_parameter("angle_topic").value

        logger = self.get_logger()

        if not isinstance(vid, str) or not vid or not _VID_RE.match(vid):
            raise ValueError(
                f"Invalid vehicle_id {vid!r}: must be non-empty and match {_VID_RE.pattern}"
            )

        if not isinstance(wheelbase, (int, float)) or wheelbase <= 0:
            raise ValueError(
                f"Invalid wheelbase {wheelbase!r}: must be > 0"
            )

        self._vid = vid
        self._state_lock = threading.Lock()
        self.server_estop = False
        self.current_mode = -1

        self.transport = ZenohTransport(locator, logger)
        self.inbound = InboundHandler(vid, self.transport, logger, wheelbase)
        self.health = HealthMonitor(hb_timeout, ctrl_timeout)

        # All publish-side timers share one MutuallyExclusiveCallbackGroup so
        # they cannot interleave on the MultiThreadedExecutor. Subscriptions
        # ride a separate ReentrantCallbackGroup since they only mutate state
        # under `_state_lock` / `inbound._lock` and benefit from concurrency.
        self._publish_group = MutuallyExclusiveCallbackGroup()
        self._sub_group = ReentrantCallbackGroup()

        # Telemetry-plane outbound publishers — all under the `nev/teleop/...`
        # prefix served by the unified teleop_server router on TCP 7447.
        # `bot_heartbeat` is bot→server liveness; `telemetry_pong` is the echo
        # for the client's RTT probe (server forwards transparently). The
        # video-plane equivalents live entirely inside stream_rover.
        publish_suffixes = [
            TOPIC_MUX, TOPIC_TWIST, TOPIC_ESTOP_STATUS, TOPIC_PONG, TOPIC_BOT_HEARTBEAT,
        ]
        if _SYS_MON_AVAILABLE:
            publish_suffixes += [TOPIC_CPU, TOPIC_MEM, TOPIC_GPU, TOPIC_DISK, TOPIC_NET]
        for suffix in publish_suffixes:
            self.transport.declare_publisher(key_for(vid, suffix))

        # Subscriptions: server-issued commands ride the same `nev/teleop/...`
        # prefix (no separate `gcs` namespace anymore — the per-router split
        # already isolates command vs. video planes).
        self.transport.declare_subscriber(
            key_for(vid, TOPIC_PING),
            self.inbound.on_telemetry_ping,
        )
        self.transport.declare_subscriber(
            key_for(vid, TOPIC_CMD), self._on_teleop_relay
        )
        self.transport.declare_subscriber(
            key_for(vid, TOPIC_ESTOP_CMD), self.inbound.on_estop
        )
        self.transport.declare_subscriber(
            key_for(vid, TOPIC_CMD_MODE_BOT), self.inbound.on_cmd_mode
        )

        self.mux_status = MuxStatus()
        self.estop_status = EStopStatus()
        self.last_nav = Twist()
        self.last_teleop = Twist()
        self.last_final = Twist()
        self.current_speed: float = 0.0
        self.current_steer_angle: float = 0.0
        self.cpu_metrics = None
        self.mem_metrics = None
        self.disk_metrics = None
        self.net_metrics = None
        self.gpu_metrics = None

        self.create_subscription(
            MuxStatus, "/vehicle/mux_status",
            self._set_mux_status, 10,
            callback_group=self._sub_group,
        )
        self.create_subscription(
            EStopStatus, "/vehicle/estop_status",
            self._set_estop_status, 10,
            callback_group=self._sub_group,
        )
        self.create_subscription(
            Twist, "/cmd_vel",
            self._set_last_nav, 10,
            callback_group=self._sub_group,
        )
        self.create_subscription(
            Twist, "/remote/teleop_cmd",
            self._set_last_teleop, 10,
            callback_group=self._sub_group,
        )
        self.create_subscription(
            Twist, "/final_cmd",
            self._set_last_final, 10,
            callback_group=self._sub_group,
        )
        self.create_subscription(
            Float64, speed_topic,
            self._set_current_speed, 10,
            callback_group=self._sub_group,
        )
        self.create_subscription(
            Float64, angle_topic,
            self._set_current_steer_angle, 10,
            callback_group=self._sub_group,
        )
        if _SYS_MON_AVAILABLE:
            self.create_subscription(
                CpuMetrics, "/system_monitor/cpu",
                self._set_cpu_metrics, 10,
                callback_group=self._sub_group,
            )
            self.create_subscription(
                MemoryMetrics, "/system_monitor/memory",
                self._set_mem_metrics, 10,
                callback_group=self._sub_group,
            )
            self.create_subscription(
                DiskMetrics, "/system_monitor/disk",
                self._set_disk_metrics, 10,
                callback_group=self._sub_group,
            )
            self.create_subscription(
                NetworkMetrics, "/system_monitor/network",
                self._set_net_metrics, 10,
                callback_group=self._sub_group,
            )
            self.create_subscription(
                GpuMetrics, "/system_monitor/gpu",
                self._set_gpu_metrics, 10,
                callback_group=self._sub_group,
            )
        else:
            logger.warning(
                "system_monitor_msgs not installed — skipping resource "
                "telemetry (cpu/mem/gpu/disk/net). The control plane "
                "(server_ping/teleop/estop) and vehicle telemetry "
                "(mux/twist/estop) still run normally."
            )

        self._teleop_pub = self.create_publisher(Twist, "/remote/teleop_cmd", 10)
        self._estop_pub = self.create_publisher(EStopStatus, "/remote/estop_status", 10)
        self._mode_pub = self.create_publisher(CmdMode, "/remote/cmd_mode", 10)
        self.create_timer(
            0.05, self._process_commands, callback_group=self._publish_group
        )
        self.create_timer(
            1.0 / v_rate, self._send_vehicle, callback_group=self._publish_group
        )
        if _SYS_MON_AVAILABLE:
            self.create_timer(
                1.0 / r_rate, self._send_resources, callback_group=self._publish_group
            )
        self.create_timer(
            0.2, self._check_heartbeat, callback_group=self._publish_group
        )
        self.create_timer(
            2.0, self._log_snapshot, callback_group=self._publish_group
        )
        # Bot → server liveness on the teleop plane (1 Hz). Distinct from the
        # telemetry_ping/pong RTT mechanism: the server uses heartbeat as a
        # presence signal and pong as a latency probe.
        self.create_timer(
            1.0, self._send_heartbeat, callback_group=self._publish_group
        )

        logger.info(
            f"net_bridge started (vehicle_id={vid}) "
            f'-> {locator or "auto-discovery"}'
        )
        logger.info(f"speed_topic={speed_topic}, angle_topic={angle_topic}")

    # --- Subscription setters (run on zenoh/ROS callback threads) ------------

    def _on_teleop_relay(self, sample):
        # The HealthMonitor warns once if a teleop command arrives while the
        # bot is still in idle mode (current_mode == -1) — the inbound handler
        # still queues the command for drain.
        with self._state_lock:
            mode = self.current_mode
        if mode == -1:
            self.health.note_teleop_in_idle(self.get_logger())
        self.inbound.on_teleop(sample)

    def _set_mux_status(self, m):
        with self._state_lock:
            self.mux_status = m

    def _set_estop_status(self, m):
        with self._state_lock:
            self.estop_status = m

    def _set_last_nav(self, m):
        with self._state_lock:
            self.last_nav = m

    def _set_last_teleop(self, m):
        with self._state_lock:
            self.last_teleop = m

    def _set_last_final(self, m):
        with self._state_lock:
            self.last_final = m

    def _set_current_speed(self, m):
        with self._state_lock:
            self.current_speed = m.data

    def _set_current_steer_angle(self, m):
        with self._state_lock:
            self.current_steer_angle = m.data

    def _set_cpu_metrics(self, m):
        with self._state_lock:
            self.cpu_metrics = m

    def _set_mem_metrics(self, m):
        with self._state_lock:
            self.mem_metrics = m

    def _set_disk_metrics(self, m):
        with self._state_lock:
            self.disk_metrics = m

    def _set_net_metrics(self, m):
        with self._state_lock:
            self.net_metrics = m

    def _set_gpu_metrics(self, m):
        with self._state_lock:
            self.gpu_metrics = m

    # --- Timer callbacks (single MutuallyExclusiveCallbackGroup) -------------

    def _process_commands(self):
        cmds = self.inbound.drain_pending()

        if cmds.teleop is not None:
            msg = Twist()
            msg.linear.x, msg.angular.z = cmds.teleop
            self._teleop_pub.publish(msg)

        if cmds.estop is not None:
            with self._state_lock:
                self.server_estop = cmds.estop
            self._publish_estop()

        if cmds.mode is not None:
            with self._state_lock:
                self.current_mode = cmds.mode
            self._mode_pub.publish(CmdMode(mode=cmds.mode))

    def _check_heartbeat(self):
        with self._state_lock:
            current_mode = self.current_mode
            server_estop = self.server_estop
        state = self.health.evaluate(
            self.inbound.last_hb_time,
            self.inbound.last_ctrl_time,
            current_mode,
            server_estop,
        )

        if state.flag_changed:
            self._publish_estop(state.bridge_flag)

    def _publish_estop(self, bridge_flag: int | None = None):
        if bridge_flag is None:
            bridge_flag = self.health.current_flag()
        with self._state_lock:
            mux_flag = int(self.estop_status.mux_flag)
        self._estop_pub.publish(
            EStopStatus(
                is_estop=(bridge_flag != 0),
                bridge_flag=int(bridge_flag),
                mux_flag=mux_flag,
            )
        )

    def _send_vehicle(self):
        with self._state_lock:
            mux_status = self.mux_status
            last_nav = self.last_nav
            last_teleop = self.last_teleop
            last_final = self.last_final
            current_speed = self.current_speed
            current_steer_angle = self.current_steer_angle
            estop_status = self.estop_status
        payloads = TelemetrySerializer.serialize_vehicle(
            self._vid,
            mux_status,
            last_nav,
            last_teleop,
            last_final,
            current_speed,
            current_steer_angle,
            estop_status,
        )
        for key, data in payloads.items():
            self.transport.put(key, data)

    def _log_snapshot(self):
        now = time.monotonic()
        with self._state_lock:
            current_mode = self.current_mode
            server_estop = self.server_estop
            mux_status = self.mux_status
        hb = self.inbound.last_hb_time
        ctrl = self.inbound.last_ctrl_time
        ping_rx = self.inbound.ping_rx_count
        teleop_rx = self.inbound.teleop_rx_count
        hb_age = f"{now - hb:.1f}s" if hb is not None else "—"
        ctrl_age = f"{now - ctrl:.1f}s" if ctrl > 0 else "—"
        flag = self.health.current_flag()
        mux_active = (
            f"req={mux_status.mode}"
            f" src={mux_status.cmd_source}"
            f" rmt={int(mux_status.remote_status)}"
        )
        self.get_logger().info(
            f"[BOT] hb_age={hb_age} ctrl_age={ctrl_age} "
            f"flag={flag} mode={current_mode} "
            f"srv_estop={int(server_estop)} "
            f"ping_rx={ping_rx} "
            f"teleop_rx={teleop_rx} "
            f"mux[{mux_active}]"
        )
        # Emit a WARNING when we have never seen a server_ping. That is
        # the strongest indication that bot↔server connectivity is
        # broken on the control plane.
        if hb is None:
            self.get_logger().warning(
                "[BOT] no telemetry_ping ever received — check telemetry_locator "
                "and that the teleop_server is reachable on the configured TCP port"
            )

    def _send_resources(self):
        with self._state_lock:
            cpu = self.cpu_metrics
            mem = self.mem_metrics
            gpu = self.gpu_metrics
            disk = self.disk_metrics
            net = self.net_metrics
        payloads = TelemetrySerializer.serialize_resources(
            self._vid, cpu, mem, gpu, disk, net,
        )
        for key, data in payloads.items():
            self.transport.put(key, data)

    def _send_heartbeat(self):
        # Lightweight liveness payload. The server uses this to keep
        # per-vehicle last_robot_recv fresh even when no telemetry topic
        # is currently being produced (e.g. system_monitor missing).
        with self._state_lock:
            current_mode = int(self.current_mode)
            server_estop = bool(self.server_estop)
        self.transport.put(
            key_for(self._vid, TOPIC_BOT_HEARTBEAT),
            {
                "ts": time.time(),
                "vehicle_id": self._vid,
                "current_mode": current_mode,
                "server_estop": server_estop,
                "bridge_flag": int(self.health.current_flag()),
            },
        )

    def destroy_node(self):
        super().destroy_node()
        # Close the zenoh session *after* the ROS node has torn down its
        # timers and subscriptions, so no callback can race with session
        # teardown.
        try:
            self.transport.close()
        except Exception as e:
            self.get_logger().warning(f"transport close error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = NetBridge()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
    except RuntimeError as e:
        # Transport/setup failure surfaces here; log and shut down cleanly.
        rclpy.logging.get_logger("net_bridge").fatal(f"net_bridge startup failed: {e}")
    finally:
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
