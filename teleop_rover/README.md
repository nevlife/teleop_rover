# teleop_rover

NEV 차량(Bot)측 **텔레메트리/제어 전용** ROS2 패키지. 영상 패키지
(`stream_rover`)와 완전히 분리되어 있으며 빌드/런타임 모두 독립이다.

와이어 컨트랙트(토픽 상수, JSON envelope, 페이로드 dataclass)는 별도
패키지 [`teleop_contracts`](../teleop_contracts) 에서 가져온다. 토픽 접미사
문자열을 하드코딩하지 않으며 모든 발행/구독 JSON 은 envelope 으로
감싼다.

## 구성

| 파일 | 설명 |
|---|---|
| `teleop_rover/scripts/net_bridge.py` | rclpy + Zenoh net_bridge 노드 (MultiThreadedExecutor) |
| `teleop_rover/scripts/net_bridge/zenoh_transport.py` | Zenoh 세션/펍섭 + envelope 직렬화 |
| `teleop_rover/scripts/net_bridge/inbound_commands.py` | 서버→봇 명령 파싱·검증·디듑 |
| `teleop_rover/scripts/net_bridge/health_monitor.py` | bridge_flag 평가, hb/ctrl 워치독 |
| `teleop_rover/scripts/net_bridge/telemetry_publisher.py` | 봇→서버 텔레메트리 직렬화 |
| `teleop_rover_msgs/` | EStopStatus / MuxStatus / CmdMode / NetworkStatus |

## Wire envelope

모든 JSON 페이로드는 발행 직전 `make_envelope(...)` 으로 감싸고, 수신
시점에 `parse_envelope(...)` 으로 풀어낸다. 형식:

```json
{"v": 1, "ts": 1715600000.123, ...payload}
```

- `v` (int) — `teleop_contracts.SCHEMA_VERSION`. 메이저 버전이 다르면
  수신측이 `IncompatibleSchemaError` 로 거부 후 드롭한다 (`v=0` 미포함
  메시지는 롤아웃 호환을 위해 허용).
- `ts` (float) — 발행 시각 (wall clock, epoch sec). 페이로드 본문에
  별도 `ts` 가 들어 있는 토픽(`mux`, `twist`, `estop`, `telemetry_pong`)은
  envelope `ts` 와는 의미가 다르므로 그대로 유지된다.
- 본문 키는 envelope 키(`v`, `ts`)와 동거하며 별도 nesting 없다.

## Zenoh 토픽 (teleop 라우터, TCP 7447 / UDP 7448)

모든 키는 `nev/teleop/{vehicle_id}/{suffix}` 형태이며 `teleop_contracts.key_for(...)`
로 빌드된다. `vehicle_id` 는 `^[A-Za-z0-9_-]+$` 검증을 통과해야 한다.

| 방향 | suffix | 페이로드 | 비고 |
|---|---|---|---|
| bot → server | `mux` | MuxStatus 요약 | 20 Hz 기본 |
| bot → server | `twist` | nav/teleop/final + 차량 실측 | 20 Hz 기본 |
| bot → server | `estop` | EStopStatus (bridge_flag, mux_flag) | 상태 변경 시 |
| bot → server | `cpu` `mem` `gpu` `disk` `net` | system_monitor 텔레메트리 | 1 Hz 기본, 옵션 |
| bot → server | `bot_heartbeat` | 봇 liveness (mode, server_estop, bridge_flag) | 1 Hz |
| bot → client | `telemetry_pong` | 클라이언트 ping `ts` 에코 | 서버 라우터 통과 |
| server → bot | `cmd` | `{"linear_x":..., "steer_angle":..., "seq":?}` | seq 디듑 적용 |
| server → bot | `estop_cmd` | `{"active": bool, "seq":?}` | bool 타입 강제 |
| server → bot | `cmd_mode_bot` | `{"mode": int, "seq":?}` | client 의 `cmd_mode` 와 별도 suffix (서버 자기 피드백 방지) |
| client → bot | `telemetry_ping` | `{}` (envelope `ts` 만) | 서버 라우터 통과, hb 워치독 |

수신 페이로드는 최대 **64 KB** 까지만 허용 (초과 시 드롭하고 5초당 1회
경고 로깅).

### `cmd_mode` vs `cmd_mode_bot`

`cmd_mode` 는 client→server 방향 (운전자 조작). 서버가 이를 받아
봇으로 릴레이할 때는 동일 라우터·동일 prefix 에서 자신이 구독 중이던
suffix 로 다시 발행하면 피드백 루프가 발생한다. 그래서 봇으로 가는
복사본은 별도 suffix `cmd_mode_bot` 으로 분리되어 있으며, 봇은
**`cmd_mode_bot` 만 구독**한다.

### Inbound 검증·디듑

- `cmd` 의 `seq` 가 있으면 source 별 최신 seq 보다 작은 것은 드롭, 없는
  것은 호환을 위해 통과.
- `linear_x` 는 ±2.0 m/s, `steer_angle` 은 ±0.7 rad 로 클램프한 뒤
  `wheelbase` 로 각속도 변환.
- `estop_cmd.active` 는 strict `bool`, `cmd_mode_bot.mode` 는 strict
  `int` (`bool` 거부) 외에는 모두 경고 로깅 후 드롭.

## launch

```bash
ros2 launch teleop_rover net_bridge.launch.py
```

`launch/net_bridge.launch.py` 가 `config/network.yaml`,
`config/net_bridge_params.yaml` 을 머지해서 노드에 전달한다.

## 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `telemetry_locator` | `tcp/127.0.0.1:7447` | Zenoh router 엔드포인트(단일/CSV/리스트) |
| `vehicle_id` | `"0"` | `^[A-Za-z0-9_-]+$` 검증 |
| `heartbeat_timeout` | `2.0` | 초; `telemetry_ping` 부재 시 e-stop flag=3 |
| `control_timeout` | `1.0` | 초; REMOTE 모드에서 `cmd` 부재 시 flag=4 |
| `vehicle_rate` | `20.0` | mux/twist/estop 발행 Hz |
| `resource_rate` | `1.0` | cpu/mem/gpu/disk/net 발행 Hz |
| `wheelbase` | `0.650` | 미터; **반드시 > 0**, 위반 시 시작 거부 |
| `speed_topic` | `/vehicle/speed` | `Float64` 입력 토픽 |
| `angle_topic` | `/vehicle/steer_angle` | `Float64` 입력 토픽 |

## 동시성 모델

`NetBridge` 는 `MultiThreadedExecutor` 위에서 두 콜백 그룹으로 동작한다.

- **`ReentrantCallbackGroup`** — 모든 ROS 구독자 (mux/estop/twist/속도/조향
  /system_monitor). 콜백은 `_state_lock` 또는 `InboundHandler._lock` 으로만
  공유 상태를 만지므로 동시 진입 허용.
- **`MutuallyExclusiveCallbackGroup`** — 모든 타이머 (명령 드레인, 차량
  텔레메트리, 리소스, 워치독, 스냅샷, heartbeat). 발행 측은 직렬화한다.

`bridge_flag` 등 HealthMonitor 의 내부 상태는 `HealthMonitor.current_flag()`
공개 접근자로만 읽는다.

## 의존성

- ROS 2 Jazzy
- `teleop_rover_msgs` (자체 패키지)
- `teleop_contracts` (sibling, stdlib only — 빌드 의존 없이 PYTHONPATH 로
  로딩됨; `net_bridge.py` 상단 쉼 참고)
- `system_monitor_msgs` (선택; 미설치 시 resource 텔레메트리만 비활성화)
- `hunter_msgs` (선택; 차량별 ROS 토픽 브리지용으로 `package.xml` 에 선언)
- `zenoh-python`

`stream_rover`, GStreamer, CUDA 에 **의존하지 않는다**.

## 이력

- **토픽 prefix 통합**: 구 `nev/robot/...` (bot→server) + `nev/gcs/...`
  (server→bot) → 단일 `nev/teleop/...`. 라우터가 stream (7457/7458) /
  teleop (7447/7448) 로 물리 분리되었기 때문에 prefix 로 방향을 가를
  필요가 없어졌다.
- **RTT 페어 통합**: `server_ping`/`bot_pong` 등 두 페어 → 단일
  `telemetry_ping`/`telemetry_pong`. 서버는 라우터로 통과만 시키므로
  RTT 는 client↔bot 전체 구간. 봇 liveness 는 별도 `bot_heartbeat`
  (1 Hz).
- **`cmd_mode` → `cmd_mode_bot`**: 서버 피드백 루프 방지를 위해 봇
  방향 suffix 를 분리.
- **Envelope 도입**: 모든 JSON 페이로드가 `{v, ts, ...}` 로 감싸짐.
  major version 불일치 시 수신 드롭.
- **`SystemResources.msg` 제거**: 미사용으로 삭제. `teleop_topics_config`
  파라미터도 제거됨.
