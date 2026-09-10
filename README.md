# teleop_rover

Vehicle-side ROS 2 packages for the native teleoperation stack.

Video, telemetry, and control travel end-to-end over WebRTC to
[`teleop_client`](https://github.com/nevlife/teleop_client), paired by
[`teleop_server`](https://github.com/nevlife/teleop_server). Zenoh is not a
dependency and is not part of the rover runtime.

| Package | Role |
|---|---|
| [`teleop_rover/`](teleop_rover/) | Safety boundary (`ControlGuard`) and the native WebRTC media endpoint |
| [`teleop_rover_msgs/`](teleop_rover_msgs/) | `EStopStatus` / `CmdMode` / `MuxStatus` / `NetworkStatus` messages |

The wire contract (v2 protobuf schemas, signaling JSON schema) is pinned as a
git submodule from
[`teleop_contracts`](https://github.com/nevlife/teleop_contracts).

## Build (ROS 2 Jazzy + colcon)

```bash
git clone --recurse-submodules https://github.com/nevlife/teleop_rover.git
```

```bash
cd ~/ros2_ws/src
ln -s /path/to/teleop_rover .
cd ~/ros2_ws
colcon build --symlink-install --packages-select teleop_rover teleop_rover_msgs
source install/setup.bash
```

`teleop_rover` needs only `ament_cmake`; `rosdep` is not required for it.
Install the GStreamer 1.24 development packages before building, or the
optional media probe target is silently skipped:

```bash
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
                 libgstreamer-plugins-bad1.0-dev
```

## Test

```bash
colcon test --packages-select teleop_rover
colcon test-result --verbose
```

## Codec probe

Reports the best available encoder per codec, in the preference order used by
`config/media_profiles.yaml`:

```bash
ros2 run teleop_rover teleop-media-probe
```

```
AV1=svtav1enc
VP9=vp9enc
H264=nvh264enc
```

## Status

`ControlGuard` and the codec probe are implemented. The `webrtcbin` send
pipeline and the control/telemetry data channels are the next milestone.
