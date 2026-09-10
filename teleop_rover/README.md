# teleop_rover

Vehicle-side package of the native teleop stack. The legacy Zenoh bridge has
been removed; this package is the rover endpoint.

Implemented:

- latched session/epoch safety boundary (`ControlGuard`)
- strict uint64 sequence handling
- absolute command expiry and future-clock rejection
- 250 ms local watchdog with explicit re-arm requirement
- media and codec policy configuration

The next media target is a native GStreamer `webrtcbin` endpoint using the USB
camera directly. ROS 2 remains local to the rover and receives only commands
that passed `ControlGuard`.

Building the media target requires the GStreamer 1.24 development packages.
The safety library and its tests deliberately remain buildable without them.
