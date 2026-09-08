#include "teleop_rover_v2/media_config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <fstream>

namespace teleop_rover_v2
{
namespace
{

/// Read one scalar. A missing key keeps `out`; a present but unparseable or
/// out-of-range key raises, so a typo cannot silently widen a limit.
template<typename T>
void read(const YAML::Node & parent, const char * key, T & out)
{
  const auto node = parent[key];
  if (!node) {
    return;
  }
  try {
    out = node.as<T>();
  } catch (const YAML::Exception &) {
    throw ConfigError(std::string("value for '") + key + "' is not usable");
  }
}

void require_positive(const char * key, long long value)
{
  if (value <= 0) {
    throw ConfigError(std::string("'") + key + "' must be positive");
  }
}

std::string upper(std::string text)
{
  std::transform(
    text.begin(), text.end(), text.begin(),
    [](unsigned char c) {return static_cast<char>(std::toupper(c));});
  return text;
}

}  // namespace

MediaConfig parse_media_config(const std::string & yaml)
{
  MediaConfig config;
  YAML::Node root;
  try {
    root = YAML::Load(yaml);
  } catch (const YAML::Exception & error) {
    throw ConfigError(std::string("not valid YAML: ") + error.what());
  }
  if (!root.IsMap() && !root.IsNull()) {
    throw ConfigError("top level must be a mapping");
  }

  if (const auto camera = root["camera"]) {
    read(camera, "device", config.camera.device);
    read(camera, "width", config.camera.width);
    read(camera, "height", config.camera.height);
    read(camera, "fps", config.camera.fps);
  }
  require_positive("camera.width", config.camera.width);
  require_positive("camera.height", config.camera.height);
  require_positive("camera.fps", config.camera.fps);
  if (config.camera.device.empty()) {
    throw ConfigError("'camera.device' must not be empty");
  }

  if (const auto transport = root["transport"]) {
    read(transport, "signaling_url", config.transport.signaling_url);
    read(transport, "force_relay", config.transport.force_relay);
  }
  if (config.transport.signaling_url.rfind("ws://", 0) != 0 &&
    config.transport.signaling_url.rfind("wss://", 0) != 0)
  {
    throw ConfigError("'transport.signaling_url' must be a ws:// or wss:// URL");
  }

  if (const auto control = root["control"]) {
    read(control, "command_rate_hz", config.control.command_rate_hz);
    read(control, "command_valid_for_ms", config.control.command_valid_for_ms);
    read(control, "watchdog_ms", config.control.watchdog_ms);
    read(control, "max_future_clock_skew_ms", config.control.max_future_clock_skew_ms);
  }
  require_positive("control.command_rate_hz", config.control.command_rate_hz);
  require_positive("control.command_valid_for_ms", config.control.command_valid_for_ms);
  require_positive("control.watchdog_ms", config.control.watchdog_ms);
  if (config.control.max_future_clock_skew_ms < 0) {
    throw ConfigError("'control.max_future_clock_skew_ms' must not be negative");
  }
  // A command must go stale before the watchdog trips. The other way round,
  // a command queued by the network could still be accepted after the guard
  // had already decided the link was dead.
  if (config.control.command_valid_for_ms > config.control.watchdog_ms) {
    throw ConfigError(
            "'control.command_valid_for_ms' must not exceed 'control.watchdog_ms'");
  }
  // Commands must arrive faster than they expire, or every one of them is
  // stale before its successor lands and the guard never stays armed.
  const int command_interval_ms = 1000 / config.control.command_rate_hz;
  if (command_interval_ms >= config.control.command_valid_for_ms) {
    throw ConfigError(
            "'control.command_rate_hz' is too low for 'control.command_valid_for_ms'");
  }

  if (const auto video = root["video"]) {
    if (const auto preference = video["codec_preference"]) {
      if (!preference.IsSequence()) {
        throw ConfigError("'video.codec_preference' must be a sequence");
      }
      std::vector<std::string> codecs;
      for (const auto & entry : preference) {
        const auto name = upper(entry.as<std::string>());
        if (name != "AV1" && name != "VP9" && name != "H264") {
          throw ConfigError("unknown codec in 'video.codec_preference': " + name);
        }
        if (std::find(codecs.begin(), codecs.end(), name) == codecs.end()) {
          codecs.push_back(name);
        }
      }
      if (codecs.empty()) {
        throw ConfigError("'video.codec_preference' must not be empty");
      }
      config.video.codec_preference = codecs;
    }
    read(video, "target_bitrate_bps", config.video.target_bitrate_bps);
    read(video, "keyframe_interval_frames", config.video.keyframe_interval_frames);
    read(video, "b_frames", config.video.b_frames);
    read(video, "latency_mode", config.video.latency_mode);
  }
  require_positive("video.target_bitrate_bps", config.video.target_bitrate_bps);
  require_positive("video.keyframe_interval_frames", config.video.keyframe_interval_frames);
  if (config.video.b_frames != 0) {
    // B-frames reorder output, which costs at least one frame of latency.
    throw ConfigError("'video.b_frames' must be 0 for a teleoperation stream");
  }
  if (config.video.latency_mode != "realtime") {
    throw ConfigError("'video.latency_mode' must be 'realtime'");
  }

  return config;
}

MediaConfig load_media_config(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {
    throw ConfigError("cannot open " + path);
  }
  const std::string text{
    std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
  return parse_media_config(text);
}

}  // namespace teleop_rover_v2
