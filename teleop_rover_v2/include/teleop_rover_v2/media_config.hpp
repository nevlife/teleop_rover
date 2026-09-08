#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace teleop_rover_v2
{

struct CameraConfig
{
  std::string device{"/dev/video0"};
  int width{1280};
  int height{720};
  int fps{30};
};

struct TransportConfig
{
  std::string signaling_url{"ws://127.0.0.1:13437/ws"};
  /// Advertise only relay candidates. Set while the rover sits behind a
  /// carrier NAT that host and server-reflexive candidates cannot cross.
  bool force_relay{true};
};

struct ControlConfig
{
  int command_rate_hz{50};
  int command_valid_for_ms{200};
  int watchdog_ms{250};
  int max_future_clock_skew_ms{50};
};

struct VideoConfig
{
  /// Ordered preference. Runtime probing removes codecs with no usable
  /// encoder, so this is a wish list rather than a guarantee.
  std::vector<std::string> codec_preference{"AV1", "VP9", "H264"};
  std::int64_t target_bitrate_bps{2000000};
  int keyframe_interval_frames{30};
  int b_frames{0};
  std::string latency_mode{"realtime"};
};

struct MediaConfig
{
  CameraConfig camera;
  TransportConfig transport;
  ControlConfig control;
  VideoConfig video;
};

/// Thrown when a file is present but its contents cannot be used.
class ConfigError : public std::runtime_error
{
public:
  explicit ConfigError(const std::string & what)
  : std::runtime_error(what) {}
};

/// Parse `media_profiles.yaml`. Missing keys keep the struct defaults; a key
/// that is present but unusable is an error rather than a silent fallback,
/// because a mistyped limit must not quietly become a permissive one.
MediaConfig load_media_config(const std::string & path);

/// Parse from a YAML string. Used by the tests and by `load_media_config`.
MediaConfig parse_media_config(const std::string & yaml);

}  // namespace teleop_rover_v2
