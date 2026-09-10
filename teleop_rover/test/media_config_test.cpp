#include "teleop_rover/media_config.hpp"

#include <gtest/gtest.h>

#include <fstream>

using teleop_rover::ConfigError;
using teleop_rover::MediaConfig;
using teleop_rover::load_media_config;
using teleop_rover::parse_media_config;

namespace
{
/// The shipped profile, kept in sync with config/media_profiles.yaml.
constexpr const char * kShipped = R"(
camera:
  device: /dev/video0
  width: 1280
  height: 720
  fps: 30

transport:
  signaling_url: ws://127.0.0.1:13437/ws
  force_relay: true

control:
  command_rate_hz: 50
  command_valid_for_ms: 200
  watchdog_ms: 250
  max_future_clock_skew_ms: 50

video:
  codec_preference: [AV1, VP9, H264]
  target_bitrate_bps: 2000000
  keyframe_interval_frames: 30
  b_frames: 0
  latency_mode: realtime
)";
}  // namespace

TEST(MediaConfig, EmptyDocumentKeepsDefaults)
{
  const auto config = parse_media_config("");
  EXPECT_EQ(config.camera.device, "/dev/video0");
  EXPECT_EQ(config.camera.width, 1280);
  EXPECT_EQ(config.video.codec_preference.size(), 3u);
  EXPECT_TRUE(config.transport.force_relay);
}

TEST(MediaConfig, PartialDocumentOverridesOnlyWhatIsPresent)
{
  const auto config = parse_media_config("camera:\n  width: 1920\n  height: 1080\n");
  EXPECT_EQ(config.camera.width, 1920);
  EXPECT_EQ(config.camera.height, 1080);
  EXPECT_EQ(config.camera.fps, 30);
  EXPECT_EQ(config.camera.device, "/dev/video0");
}

TEST(MediaConfig, ReadsEveryField)
{
  const auto config = parse_media_config(R"(
camera: {device: /dev/video2, width: 640, height: 480, fps: 15}
transport: {signaling_url: "wss://example.test:13437/ws", force_relay: false}
control:
  command_rate_hz: 20
  command_valid_for_ms: 250
  watchdog_ms: 300
  max_future_clock_skew_ms: 0
video:
  codec_preference: [h264, VP9]
  target_bitrate_bps: 800000
  keyframe_interval_frames: 15
  b_frames: 0
  latency_mode: realtime
)");
  EXPECT_EQ(config.camera.device, "/dev/video2");
  EXPECT_EQ(config.camera.fps, 15);
  EXPECT_EQ(config.transport.signaling_url, "wss://example.test:13437/ws");
  EXPECT_FALSE(config.transport.force_relay);
  EXPECT_EQ(config.control.command_rate_hz, 20);
  EXPECT_EQ(config.control.max_future_clock_skew_ms, 0);
  // Codec names are normalised and de-duplicated, order preserved.
  ASSERT_EQ(config.video.codec_preference.size(), 2u);
  EXPECT_EQ(config.video.codec_preference[0], "H264");
  EXPECT_EQ(config.video.codec_preference[1], "VP9");
}

TEST(MediaConfig, DuplicateCodecsCollapse)
{
  const auto config = parse_media_config("video:\n  codec_preference: [VP9, vp9, AV1]\n");
  ASSERT_EQ(config.video.codec_preference.size(), 2u);
  EXPECT_EQ(config.video.codec_preference[0], "VP9");
  EXPECT_EQ(config.video.codec_preference[1], "AV1");
}

TEST(MediaConfig, RejectsUnknownCodec)
{
  EXPECT_THROW(parse_media_config("video:\n  codec_preference: [H265]\n"), ConfigError);
}

TEST(MediaConfig, RejectsEmptyCodecPreference)
{
  EXPECT_THROW(parse_media_config("video:\n  codec_preference: []\n"), ConfigError);
}

TEST(MediaConfig, RejectsCodecPreferenceThatIsNotASequence)
{
  EXPECT_THROW(parse_media_config("video:\n  codec_preference: AV1\n"), ConfigError);
}

TEST(MediaConfig, RejectsNonPositiveCameraGeometry)
{
  EXPECT_THROW(parse_media_config("camera:\n  width: 0\n"), ConfigError);
  EXPECT_THROW(parse_media_config("camera:\n  fps: -1\n"), ConfigError);
}

TEST(MediaConfig, RejectsEmptyDevice)
{
  EXPECT_THROW(parse_media_config("camera:\n  device: \"\"\n"), ConfigError);
}

TEST(MediaConfig, RejectsNonWebSocketSignalingUrl)
{
  EXPECT_THROW(
    parse_media_config("transport:\n  signaling_url: http://host:13437/ws\n"), ConfigError);
}

TEST(MediaConfig, RejectsCommandValidityLongerThanWatchdog)
{
  // A command must go stale before the watchdog trips, or a command held in a
  // network queue could be accepted after the guard gave up on the link.
  EXPECT_THROW(
    parse_media_config("control:\n  command_valid_for_ms: 400\n  watchdog_ms: 250\n"),
    ConfigError);
}

TEST(MediaConfig, AcceptsValidityShorterThanWatchdog)
{
  const auto config = parse_media_config(
    "control:\n  command_valid_for_ms: 100\n  watchdog_ms: 250\n");
  EXPECT_EQ(config.control.command_valid_for_ms, 100);
}

TEST(MediaConfig, RejectsCommandRateSlowerThanExpiry)
{
  // 4 Hz is one command every 250 ms, so each would expire before the next.
  EXPECT_THROW(
    parse_media_config("control:\n  command_rate_hz: 4\n"), ConfigError);
}

TEST(MediaConfig, RejectsBFrames)
{
  EXPECT_THROW(parse_media_config("video:\n  b_frames: 2\n"), ConfigError);
}

TEST(MediaConfig, RejectsNonRealtimeLatencyMode)
{
  EXPECT_THROW(parse_media_config("video:\n  latency_mode: quality\n"), ConfigError);
}

TEST(MediaConfig, RejectsUnparseableValue)
{
  EXPECT_THROW(parse_media_config("camera:\n  width: wide\n"), ConfigError);
}

TEST(MediaConfig, RejectsMalformedYaml)
{
  EXPECT_THROW(parse_media_config("camera: [unterminated\n"), ConfigError);
}

TEST(MediaConfig, RejectsNonMappingTopLevel)
{
  EXPECT_THROW(parse_media_config("- one\n- two\n"), ConfigError);
}

TEST(MediaConfig, AcceptsTheShippedProfile)
{
  const auto config = parse_media_config(kShipped);
  EXPECT_EQ(config.camera.width, 1280);
  EXPECT_EQ(config.control.watchdog_ms, 250);
  EXPECT_EQ(config.video.target_bitrate_bps, 2000000);
}

TEST(MediaConfig, MissingFileIsAnError)
{
  EXPECT_THROW(load_media_config("/nonexistent/media_profiles.yaml"), ConfigError);
}
