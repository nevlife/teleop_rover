# teleop_rover

NEV 차량(Bot) 측 텔레메트리·제어 ROS 2 모듈.

| 패키지 | 역할 | 라우터 |
|---|---|---|
| [`teleop_rover/`](teleop_rover/) | 텔레메트리·제어 (cmd, estop, mux, twist, heartbeat) | teleop 라우터 (TCP 7447 / UDP 7448) |
| [`teleop_rover_msgs/`](teleop_rover_msgs/) | EStopStatus / CmdMode / MuxStatus / NetworkStatus 메시지 | — |

영상 송신은 별도 리포 [`stream_rover`](../stream_rover/)에서 다룬다.

와이어 컨트랙트(토픽 상수, JSON envelope, payload dataclass)는 외부
[`teleop_contracts`](https://github.com/nevlife/teleop_contracts) 리포에서 가져오며,
본 리포 안에서는 `teleop_contracts/` 경로의 git submodule 로 pin 된다.

```bash
git clone --recurse-submodules https://github.com/nevlife/teleop_rover.git
```

## 빌드 (ROS 2 Jazzy + colcon)

```bash
cd ~/ros2_ws/src
ln -s /path/to/teleop_rover .
cd ~/ros2_ws
colcon build --symlink-install --packages-select teleop_rover teleop_rover_msgs
source install/setup.bash
```

`teleop_contracts` 는 ROS 패키지가 아니라 pure Python 모듈이므로 colcon이 직접 빌드하지 않는다. `teleop_rover/CMakeLists.txt` 가 install 시점에 `lib/teleop_rover/teleop_contracts` 로 복사한다.

## 런타임

```bash
ros2 launch teleop_rover net_bridge.launch.py
```

## 의존성

- ROS 2 Jazzy, `zenoh-python`
- 선택: `system_monitor_msgs` (resource 텔레메트리), `hunter_msgs` (Hunter UGV bridge)

## 디렉토리

```
teleop_rover/
├── teleop_rover/         # 텔레메트리·제어 ROS 2 패키지 (Python)
├── teleop_rover_msgs/    # 메시지 패키지 (EStopStatus / CmdMode / MuxStatus / NetworkStatus)
└── teleop_contracts/     # git submodule -> nevlife/teleop_contracts
```

상세는 [`teleop_rover/README.md`](teleop_rover/README.md) 참고.
