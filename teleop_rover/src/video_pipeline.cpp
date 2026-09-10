#include "teleop_rover/video_pipeline.hpp"

#include <gst/gst.h>

#include <sstream>
#include <stdexcept>

namespace teleop_rover
{
namespace
{

std::int64_t to_kbps(std::int64_t bits_per_second)
{
  const auto kbps = bits_per_second / 1000;
  return kbps > 0 ? kbps : 1;
}

/// Encoder settings differ enough between factories that a shared property
/// string is not possible: units alone vary between kbit/s and bit/s.
std::string encoder_properties(
  const std::string & encoder, const VideoConfig & video)
{
  std::ostringstream out;
  const auto gop = video.keyframe_interval_frames;

  if (encoder == "nvh264enc") {
    out << " bitrate=" << to_kbps(video.target_bitrate_bps)
        << " gop-size=" << gop
        << " bframes=0"
        << " preset=low-latency-hq";
  } else if (encoder == "x264enc") {
    out << " bitrate=" << to_kbps(video.target_bitrate_bps)
        << " key-int-max=" << gop
        << " bframes=0"
        << " speed-preset=veryfast"
        << " tune=zerolatency";
  } else if (encoder == "vp9enc") {
    // vpxenc takes bit/s, and deadline=1 is its realtime mode.
    out << " target-bitrate=" << video.target_bitrate_bps
        << " keyframe-max-dist=" << gop
        << " deadline=1"
        << " lag-in-frames=0"
        << " error-resilient=1";
  } else if (encoder == "svtav1enc") {
    out << " target-bitrate=" << to_kbps(video.target_bitrate_bps)
        << " intra-period-length=" << gop
        << " preset=10";
  } else if (encoder == "av1enc") {
    out << " target-bitrate=" << to_kbps(video.target_bitrate_bps)
        << " keyframe-max-dist=" << gop
        << " usage-profile=realtime"
        << " lag-in-frames=0";
  } else {
    // VA-API and V4L2 encoders expose bitrate in bit/s and nothing else this
    // code needs to set. Anything unrecognised runs at its own defaults
    // rather than being fed properties it may not have.
    out << " bitrate=" << video.target_bitrate_bps;
  }
  return out.str();
}

}  // namespace

bool factory_exists(const std::string & name)
{
  GstElementFactory * factory = gst_element_factory_find(name.c_str());
  if (factory == nullptr) {
    return false;
  }
  gst_object_unref(factory);
  return true;
}

std::vector<EncoderChoice> candidate_chains(const std::string & codec)
{
  if (codec == "H264") {
    std::vector<EncoderChoice> chains;
    for (const char * encoder : {"nvh264enc", "vah264enc", "v4l2h264enc", "x264enc"}) {
      chains.push_back({"H264", encoder, "h264parse", "rtph264pay", "H264"});
    }
    return chains;
  }
  if (codec == "VP9") {
    std::vector<EncoderChoice> chains;
    for (const char * encoder : {"vavp9enc", "vp9enc"}) {
      chains.push_back({"VP9", encoder, "", "rtpvp9pay", "VP9"});
    }
    return chains;
  }
  if (codec == "AV1") {
    std::vector<EncoderChoice> chains;
    for (const char * encoder : {"nvav1enc", "vaav1enc", "svtav1enc", "av1enc"}) {
      chains.push_back({"AV1", encoder, "av1parse", "rtpav1pay", "AV1"});
    }
    return chains;
  }
  return {};
}

std::optional<EncoderChoice> select_encoder(
  const std::vector<std::string> & preference, const FactoryPredicate & available)
{
  for (const auto & codec : preference) {
    for (const auto & chain : candidate_chains(codec)) {
      if (!available(chain.encoder)) {
        continue;
      }
      if (!available(chain.payloader)) {
        // The whole codec is unusable, not just this encoder.
        break;
      }
      if (!chain.parser.empty() && !available(chain.parser)) {
        break;
      }
      return chain;
    }
  }
  return std::nullopt;
}

std::optional<EncoderChoice> select_encoder(const std::vector<std::string> & preference)
{
  return select_encoder(preference, factory_exists);
}

std::string build_send_pipeline(
  const MediaConfig & config, const EncoderChoice & choice,
  const std::string & sink_name)
{
  const auto & camera = config.camera;
  std::ostringstream out;

  // MJPEG is requested explicitly: the USB bandwidth for raw YUY2 at 720p30
  // is not there on most webcams, and decodebin's negotiation is not worth
  // the latency when the format is known.
  out << "v4l2src device=" << camera.device
      << " io-mode=mmap"
      << " ! image/jpeg,width=" << camera.width
      << ",height=" << camera.height
      << ",framerate=" << camera.fps << "/1"
      << " ! jpegdec"
      << " ! videoconvert"
      << " ! video/x-raw,format=I420"
      << " ! " << choice.encoder << encoder_properties(choice.encoder, config.video);

  if (!choice.parser.empty()) {
    out << " ! " << choice.parser;
    if (choice.parser == "h264parse") {
      // Reinsert SPS/PPS on every keyframe so a receiver joining mid-stream
      // can decode without waiting for the next parameter set.
      out << " config-interval=-1";
    }
  }

  out << " ! " << choice.payloader << " pt=96";
  if (choice.payloader == "rtph264pay") {
    out << " aggregate-mode=zero-latency";
  }
  out << " ! application/x-rtp,media=video,encoding-name=" << choice.rtp_encoding_name
      << ",payload=96"
      // A short, non-leaky queue. Dropping RTP packets here would corrupt
      // the bitstream mid-frame; webrtcbin does its own pacing and congestion
      // control downstream, so this only decouples the encoder thread.
      << " ! queue name=" << sink_name
      << " max-size-buffers=0 max-size-bytes=0 max-size-time=100000000";

  return out.str();
}

}  // namespace teleop_rover
