#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace teleop_rover_v2
{

/// Relay credentials handed out by the signaling server in `hello_ack`.
struct TurnCredentials
{
  std::vector<std::string> urls;
  std::string username;
  std::string credential;
};

struct HelloAck
{
  int protocol_version{0};
  std::string role;
  std::string robot_id;
  std::optional<TurnCredentials> turn;
};

struct PeerReady
{
  std::string robot_id;
  std::string session_id;
  /// Sent as a JSON string, not a number: the value exceeds what a double
  /// represents exactly, so a numeric field would round.
  std::uint64_t session_epoch{0};
  std::string peer_role;
  /// The peer told to produce the SDP offer. Exactly one side gets true.
  bool create_offer{false};
};

struct SessionDescription
{
  std::string sdp;
};

struct IceCandidate
{
  std::string candidate;
  std::string sdp_mid;
  int sdp_mline_index{0};
};

struct ServerError
{
  std::string code;
};

/// A message received from the signaling server.
struct IncomingMessage
{
  enum class Kind
  {
    kHelloAck,
    kPeerReady,
    kPeerLeft,
    kOffer,
    kAnswer,
    kIce,
    kError,
    kUnknown,
  };

  Kind kind{Kind::kUnknown};
  /// The raw `type` field, kept so an unknown message can be logged usefully.
  std::string type;
  HelloAck hello_ack;
  PeerReady peer_ready;
  SessionDescription description;
  IceCandidate ice;
  ServerError error;
};

class ProtocolError : public std::runtime_error
{
public:
  explicit ProtocolError(const std::string & what)
  : std::runtime_error(what) {}
};

constexpr int kProtocolVersion = 2;

/// The first frame the rover must send after the socket opens.
std::string build_hello(const std::string & robot_id, const std::string & role = "rover");
std::string build_offer(const std::string & sdp);
std::string build_answer(const std::string & sdp);
std::string build_ice(const IceCandidate & candidate);

/// Parse one server frame. Throws `ProtocolError` on anything that is not a
/// JSON object carrying a string `type`; an unrecognised `type` is reported
/// as `kUnknown` rather than throwing, so a newer server can add messages
/// without breaking an older rover.
IncomingMessage parse_message(const std::string & json);

}  // namespace teleop_rover_v2
