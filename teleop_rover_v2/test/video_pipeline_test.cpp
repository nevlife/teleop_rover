#include "teleop_rover_v2/video_pipeline.hpp"

#include <gst/gst.h>
#include <gtest/gtest.h>

#include <set>
#include <string>

using teleop_rover_v2::EncoderChoice;
using teleop_rover_v2::MediaConfig;
using teleop_rover_v2::build_send_pipeline;
using teleop_rover_v2::candidate_chains;
using teleop_rover_v2::select_encoder;

namespace
{
/// A registry stub, so codec selection is tested independently of whatever
/// GStreamer plugins the build machine happens to have.
teleop_rover_v2::FactoryPredicate having(std::set<std::string> factories)
{
  return [factories](const std::string & name) {return factories.count(name) > 0;};
}

const std::vector<std::string> kDefaultPreference{"AV1", "VP9", "H264"};
}  // namespace

TEST(SelectEncoder, PrefersTheFirstCodecWithACompleteChain)
{
  const auto choice = select_encoder(
    kDefaultPreference,
    having({"svtav1enc", "rtpav1pay", "av1parse", "vp9enc", "rtpvp9pay"}));
  ASSERT_TRUE(choice.has_value());
  EXPECT_EQ(choice->codec, "AV1");
  EXPECT_EQ(choice->encoder, "svtav1enc");
}

TEST(SelectEncoder, SkipsACodecWhosePayloaderIsMissing)
{
  // GStreamer 1.24 ships av1enc but no rtpav1pay, so AV1 cannot go over
  // WebRTC even though an encoder is present.
  const auto choice = select_encoder(
    kDefaultPreference,
    having({"svtav1enc", "av1parse", "vp9enc", "rtpvp9pay"}));
  ASSERT_TRUE(choice.has_value());
  EXPECT_EQ(choice->codec, "VP9");
}

TEST(SelectEncoder, SkipsACodecWhoseParserIsMissing)
{
  const auto choice = select_encoder(
    {"H264", "VP9"}, having({"x264enc", "rtph264pay", "vp9enc", "rtpvp9pay"}));
  ASSERT_TRUE(choice.has_value());
  EXPECT_EQ(choice->codec, "VP9") << "H264 needs h264parse, which is absent";
}

TEST(SelectEncoder, PrefersHardwareWithinACodec)
{
  const auto choice = select_encoder(
    {"H264"}, having({"nvh264enc", "x264enc", "h264parse", "rtph264pay"}));
  ASSERT_TRUE(choice.has_value());
  EXPECT_EQ(choice->encoder, "nvh264enc");
}

TEST(SelectEncoder, FallsBackToSoftwareWithinACodec)
{
  const auto choice = select_encoder(
    {"H264"}, having({"x264enc", "h264parse", "rtph264pay"}));
  ASSERT_TRUE(choice.has_value());
  EXPECT_EQ(choice->encoder, "x264enc");
}

TEST(SelectEncoder, HonoursPreferenceOrder)
{
  const std::set<std::string> everything{
    "nvh264enc", "h264parse", "rtph264pay",
    "vp9enc", "rtpvp9pay",
    "svtav1enc", "av1parse", "rtpav1pay"};
  EXPECT_EQ(select_encoder({"H264", "AV1"}, having(everything))->codec, "H264");
  EXPECT_EQ(select_encoder({"VP9", "H264"}, having(everything))->codec, "VP9");
}

TEST(SelectEncoder, ReturnsNothingWhenNoChainIsComplete)
{
  EXPECT_FALSE(select_encoder(kDefaultPreference, having({"x264enc"})).has_value());
  EXPECT_FALSE(select_encoder({}, having({"x264enc", "h264parse", "rtph264pay"})).has_value());
}

TEST(SelectEncoder, UnknownCodecHasNoChains)
{
  EXPECT_TRUE(candidate_chains("H265").empty());
}

TEST(BuildSendPipeline, DescribesTheWholeChain)
{
  MediaConfig config;
  config.camera.device = "/dev/video2";
  config.camera.width = 640;
  config.camera.height = 480;
  config.camera.fps = 15;
  config.video.target_bitrate_bps = 1500000;
  config.video.keyframe_interval_frames = 45;

  const EncoderChoice choice{"H264", "nvh264enc", "h264parse", "rtph264pay", "H264"};
  const auto description = build_send_pipeline(config, choice, "video-src");

  EXPECT_NE(description.find("v4l2src device=/dev/video2"), std::string::npos);
  EXPECT_NE(description.find("width=640,height=480,framerate=15/1"), std::string::npos);
  EXPECT_NE(description.find("nvh264enc"), std::string::npos);
  // nvh264enc takes kbit/s, so the bit/s figure must be converted.
  EXPECT_NE(description.find("bitrate=1500 "), std::string::npos);
  EXPECT_NE(description.find("gop-size=45"), std::string::npos);
  EXPECT_NE(description.find("bframes=0"), std::string::npos);
  EXPECT_NE(description.find("h264parse config-interval=-1"), std::string::npos);
  EXPECT_NE(description.find("rtph264pay pt=96"), std::string::npos);
  EXPECT_NE(description.find("encoding-name=H264"), std::string::npos);
  EXPECT_NE(description.find("queue name=video-src"), std::string::npos);
}

TEST(BuildSendPipeline, KeepsVpxBitrateInBitsPerSecond)
{
  MediaConfig config;
  config.video.target_bitrate_bps = 1500000;
  const EncoderChoice choice{"VP9", "vp9enc", "", "rtpvp9pay", "VP9"};
  const auto description = build_send_pipeline(config, choice);

  EXPECT_NE(description.find("target-bitrate=1500000"), std::string::npos);
  EXPECT_NE(description.find("deadline=1"), std::string::npos);
  EXPECT_NE(description.find("lag-in-frames=0"), std::string::npos);
  // No parser for VP9; the payloader must follow the encoder directly.
  EXPECT_EQ(description.find("parse"), std::string::npos);
}

TEST(BuildSendPipeline, EveryDescriptionParses)
{
  // A description that gst_parse_launch cannot even parse is a bug in the
  // builder, and this catches a misspelled property name on any encoder the
  // build actually has.
  MediaConfig config;
  for (const auto & codec : {"H264", "VP9", "AV1"}) {
    for (const auto & chain : candidate_chains(codec)) {
      if (!teleop_rover_v2::factory_exists(chain.encoder) ||
        !teleop_rover_v2::factory_exists(chain.payloader))
      {
        continue;
      }
      const auto description = build_send_pipeline(config, chain);
      GError * error = nullptr;
      GstElement * pipeline = gst_parse_launch(description.c_str(), &error);
      const std::string message = error != nullptr ? error->message : "";
      if (error != nullptr) {
        g_error_free(error);
      }
      EXPECT_EQ(message, "") << chain.encoder << ": " << description;
      if (pipeline != nullptr) {
        gst_object_unref(pipeline);
      }
    }
  }
}

int main(int argc, char ** argv)
{
  gst_init(&argc, &argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
