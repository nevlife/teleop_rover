#pragma once

#include "teleop_rover_v2/media_config.hpp"

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace teleop_rover_v2
{

/// One usable send chain: an encoder plus the parser and RTP payloader that
/// carry its output. A codec whose payloader is missing cannot go over
/// WebRTC at all, so probing the encoder alone is not enough -- GStreamer
/// 1.24 ships `av1enc` but no `rtpav1pay`.
struct EncoderChoice
{
  std::string codec;      ///< "H264", "VP9" or "AV1"
  std::string encoder;    ///< selected encoder factory
  std::string parser;     ///< parser factory, empty when none is needed
  std::string payloader;  ///< RTP payloader factory
  std::string rtp_encoding_name;  ///< SDP encoding-name for the RTP caps
};

/// True when a GStreamer element factory of this name is registered.
using FactoryPredicate = std::function<bool (const std::string &)>;

/// The default predicate, backed by the GStreamer registry. `gst_init` must
/// have run first.
bool factory_exists(const std::string & name);

/// Every chain this build knows how to construct, in the order encoders are
/// preferred within each codec (hardware before software).
std::vector<EncoderChoice> candidate_chains(const std::string & codec);

/// First codec in `preference` with a complete, available chain.
std::optional<EncoderChoice> select_encoder(
  const std::vector<std::string> & preference, const FactoryPredicate & available);

/// Same, against the live GStreamer registry.
std::optional<EncoderChoice> select_encoder(const std::vector<std::string> & preference);

/// gst_parse_launch description for camera -> encode -> RTP, ending in a pad
/// named `name` ready to be linked to `webrtcbin`.
std::string build_send_pipeline(
  const MediaConfig & config, const EncoderChoice & choice,
  const std::string & sink_name = "video-src");

}  // namespace teleop_rover_v2
