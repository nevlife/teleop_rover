// Rover-side WebRTC endpoint: camera -> encoder -> webrtcbin, paired with an
// operator by teleop_server over WebSocket signaling.
#include "teleop_rover_v2/control_guard.hpp"
#include "teleop_rover_v2/fake_vehicle.hpp"
#include "teleop_rover_v2/media_config.hpp"
#include "teleop_rover_v2/signaling_protocol.hpp"
#include "teleop_rover_v2/video_pipeline.hpp"

#include <gst/gst.h>
#include <gst/sdp/sdp.h>

#define GST_USE_UNSTABLE_API
#include <gst/webrtc/webrtc.h>

#include <libsoup/soup.h>

#include "teleop/v2/control.pb.h"

#include <cstdio>
#include <cstring>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

namespace
{

using teleop_rover_v2::AcceptedMotion;
using teleop_rover_v2::ControlGuard;
using teleop_rover_v2::EncoderChoice;
using teleop_rover_v2::FakeVehicle;
using teleop_rover_v2::IncomingMessage;
using teleop_rover_v2::MediaConfig;
using teleop_rover_v2::MotionCommand;
using teleop_rover_v2::RejectReason;
using teleop_rover_v2::TurnCredentials;

std::int64_t steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::int64_t unix_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

const char * reason_text(RejectReason reason)
{
  switch (reason) {
    case RejectReason::kNotArmed: return "not_armed";
    case RejectReason::kWrongSession: return "wrong_session";
    case RejectReason::kOldSequence: return "old_sequence";
    case RejectReason::kExpired: return "expired";
    case RejectReason::kFutureTimestamp: return "future_timestamp";
    case RejectReason::kInvalidValue: return "invalid_value";
  }
  return "unknown";
}

struct Rover
{
  MediaConfig config;
  EncoderChoice choice;
  std::string robot_id;
  std::optional<TurnCredentials> turn;

  std::unique_ptr<ControlGuard> guard;
  FakeVehicle vehicle;
  /// Session identity from peer_ready, held until the control channel opens.
  std::string pending_session_id;
  std::uint64_t pending_session_epoch{0};
  GstWebRTCDataChannel * control_channel{nullptr};
  std::int64_t last_motion_steady_ns{0};
  guint watchdog_source{0};
  /// Rejections are logged once per reason rather than per command; a bad
  /// stream arrives at the command rate and would drown the log.
  std::string last_reject_reason;

  SoupSession * session{nullptr};
  SoupWebsocketConnection * socket{nullptr};
  GstElement * pipeline{nullptr};
  GstElement * webrtc{nullptr};
  GMainLoop * loop{nullptr};
  int exit_code{0};
};

void fail(Rover * rover, const std::string & reason)
{
  std::fprintf(stderr, "teleop-rover: %s\n", reason.c_str());
  rover->exit_code = 1;
  if (rover->loop != nullptr) {
    g_main_loop_quit(rover->loop);
  }
}

void send_text(Rover * rover, const std::string & text)
{
  if (rover->socket == nullptr) {
    return;
  }
  if (soup_websocket_connection_get_state(rover->socket) != SOUP_WEBSOCKET_STATE_OPEN) {
    return;
  }
  soup_websocket_connection_send_text(rover->socket, text.c_str());
}

/// webrtcbin wants one `turn://user:pass@host:port` string, while the server
/// sends `turn:host:port?transport=udp` with the credentials alongside.
std::optional<std::string> turn_server_uri(const TurnCredentials & turn)
{
  for (const auto & url : turn.urls) {
    std::string rest;
    if (url.rfind("turn:", 0) == 0) {
      rest = url.substr(std::strlen("turn:"));
    } else if (url.rfind("turns:", 0) == 0) {
      rest = url.substr(std::strlen("turns:"));
    } else {
      continue;
    }
    const auto query = rest.find('?');
    const std::string host_port = query == std::string::npos ? rest : rest.substr(0, query);
    const bool secure = url.rfind("turns:", 0) == 0;
    // The credentials are percent-safe by construction on this server, but
    // escaping them keeps a future password with '@' or ':' from breaking
    // the URI.
    gchar * user = g_uri_escape_string(turn.username.c_str(), nullptr, FALSE);
    gchar * pass = g_uri_escape_string(turn.credential.c_str(), nullptr, FALSE);
    std::string uri = std::string(secure ? "turns://" : "turn://") +
      user + ":" + pass + "@" + host_port;
    g_free(user);
    g_free(pass);
    return uri;
  }
  return std::nullopt;
}

void report_vehicle(Rover * rover)
{
  const auto & pose = rover->vehicle.pose();
  std::printf(
    "teleop-rover: drive %+.2f m/s %+.2f rad/s  pose (%+.2f, %+.2f) heading %+.2f\n",
    rover->vehicle.linear_mps(), rover->vehicle.angular_rps(),
    pose.x_m, pose.y_m, pose.heading_rad);
  std::fflush(stdout);
}

void halt(Rover * rover, const std::string & reason)
{
  if (rover->vehicle.stop_reason() == reason) {
    return;
  }
  rover->vehicle.stop(reason);
  std::printf("teleop-rover: vehicle stopped (%s)\n", reason.c_str());
  std::fflush(stdout);
}

/// Runs at the watchdog period. A link that goes quiet must stop the vehicle
/// without waiting for anything to arrive.
gboolean on_watchdog_tick(gpointer user_data)
{
  auto * rover = static_cast<Rover *>(user_data);
  if (rover->guard == nullptr) {
    return G_SOURCE_CONTINUE;
  }
  if (rover->guard->poll_watchdog(steady_ns())) {
    halt(rover, rover->guard->stop_reason());
  }
  return G_SOURCE_CONTINUE;
}

/// Arming waits for the control channel, not for pairing.
///
/// The guard's watchdog latches until a new epoch, so arming at peer_ready
/// guarantees a trip during ICE and DTLS setup and every later command is
/// rejected as not_armed. The link is only real once the channel is open.
void on_control_open(GstWebRTCDataChannel *, gpointer user_data)
{
  auto * rover = static_cast<Rover *>(user_data);
  if (rover->pending_session_id.empty()) {
    std::fprintf(stderr, "teleop-rover: control channel open with no session\n");
    return;
  }
  rover->guard->arm(rover->pending_session_id, rover->pending_session_epoch, steady_ns());
  rover->last_motion_steady_ns = 0;
  rover->last_reject_reason.clear();
  halt(rover, "awaiting_commands");

  if (rover->watchdog_source == 0) {
    // Poll faster than the watchdog period so the trip is detected promptly
    // rather than up to a full period late.
    const guint period = static_cast<guint>(rover->config.control.watchdog_ms) / 4 + 1;
    rover->watchdog_source = g_timeout_add(period, on_watchdog_tick, rover);
  }
  std::printf(
    "teleop-rover: armed for session %s\n", rover->pending_session_id.c_str());
  std::fflush(stdout);
}

void on_control_message(
  GstWebRTCDataChannel *, GBytes * payload, gpointer user_data)
{
  auto * rover = static_cast<Rover *>(user_data);
  gsize size = 0;
  const auto * data = static_cast<const guint8 *>(g_bytes_get_data(payload, &size));

  nev::teleop::v2::MotionCommand wire;
  if (!wire.ParseFromArray(data, static_cast<int>(size))) {
    std::fprintf(stderr, "teleop-rover: control frame is not a MotionCommand\n");
    return;
  }

  MotionCommand command;
  command.robot_id = wire.robot_id();
  command.session_id = wire.session_id();
  command.session_epoch = wire.session_epoch();
  command.sequence = wire.sequence();
  command.sent_unix_ns = wire.sent_unix_ns();
  command.valid_for_ms = wire.valid_for_ms();
  command.linear_mps = wire.linear_mps();
  command.angular_rps = wire.angular_rps();
  command.deadman = wire.deadman();

  const auto now_steady = steady_ns();
  const auto result = rover->guard->accept(command, unix_ns(), now_steady);
  if (result.rejected.has_value()) {
    const std::string reason = reason_text(*result.rejected);
    if (rover->last_reject_reason != reason) {
      rover->last_reject_reason = reason;
      std::fprintf(stderr, "teleop-rover: command rejected (%s)\n", reason.c_str());
    }
    halt(rover, reason);
    return;
  }
  rover->last_reject_reason.clear();

  const double dt_s = rover->last_motion_steady_ns == 0 ?
    0.0 : static_cast<double>(now_steady - rover->last_motion_steady_ns) / 1e9;
  rover->last_motion_steady_ns = now_steady;
  rover->vehicle.apply(*result.motion, dt_s);

  // One line a second, not one per command: the operator sends at 50 Hz.
  static std::int64_t last_report_ns = 0;
  if (now_steady - last_report_ns > 1000000000LL) {
    last_report_ns = now_steady;
    report_vehicle(rover);
  }
}

void open_control_channel(Rover * rover)
{
  GstStructure * options = gst_structure_new(
    "options",
    // control.proto: the motion channel is unordered and partially reliable.
    // A late command is worse than a lost one, so nothing is retransmitted.
    "ordered", G_TYPE_BOOLEAN, FALSE,
    "max-retransmits", G_TYPE_INT, 0,
    nullptr);
  g_signal_emit_by_name(
    rover->webrtc, "create-data-channel", "control", options, &rover->control_channel);
  gst_structure_free(options);

  if (rover->control_channel == nullptr) {
    std::fprintf(stderr, "teleop-rover: could not open the control channel\n");
    return;
  }
  g_signal_connect(
    rover->control_channel, "on-message-data", G_CALLBACK(on_control_message), rover);
  g_signal_connect(rover->control_channel, "on-open", G_CALLBACK(on_control_open), rover);
  std::printf("teleop-rover: control channel created\n");
}

void on_offer_created(GstPromise * promise, gpointer user_data)
{
  auto * rover = static_cast<Rover *>(user_data);
  const GstStructure * reply = gst_promise_get_reply(promise);
  GstWebRTCSessionDescription * offer = nullptr;
  gst_structure_get(reply, "offer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &offer, nullptr);
  gst_promise_unref(promise);
  if (offer == nullptr) {
    fail(rover, "webrtcbin produced no offer");
    return;
  }

  GstPromise * local = gst_promise_new();
  g_signal_emit_by_name(rover->webrtc, "set-local-description", offer, local);
  gst_promise_interrupt(local);
  gst_promise_unref(local);

  gchar * text = gst_sdp_message_as_text(offer->sdp);
  std::printf("teleop-rover: sending offer (%zu bytes)\n", std::strlen(text));
  send_text(rover, teleop_rover_v2::build_offer(text));
  g_free(text);
  gst_webrtc_session_description_free(offer);
}

void on_negotiation_needed(GstElement *, gpointer user_data)
{
  auto * rover = static_cast<Rover *>(user_data);
  GstPromise * promise = gst_promise_new_with_change_func(on_offer_created, rover, nullptr);
  g_signal_emit_by_name(rover->webrtc, "create-offer", nullptr, promise);
}

void on_ice_candidate(GstElement *, guint mline_index, gchar * candidate, gpointer user_data)
{
  auto * rover = static_cast<Rover *>(user_data);
  teleop_rover_v2::IceCandidate ice;
  ice.candidate = candidate != nullptr ? candidate : "";
  ice.sdp_mline_index = static_cast<int>(mline_index);
  send_text(rover, teleop_rover_v2::build_ice(ice));
}

void on_bus_message(GstBus *, GstMessage * message, gpointer user_data)
{
  auto * rover = static_cast<Rover *>(user_data);
  if (GST_MESSAGE_TYPE(message) != GST_MESSAGE_ERROR) {
    return;
  }
  GError * error = nullptr;
  gchar * debug = nullptr;
  gst_message_parse_error(message, &error, &debug);
  const std::string reason = std::string("pipeline error: ") + error->message +
    (debug != nullptr ? std::string("\n") + debug : std::string());
  g_error_free(error);
  g_free(debug);
  fail(rover, reason);
}

void stop_pipeline(Rover * rover)
{
  if (rover->pipeline == nullptr) {
    return;
  }
  gst_element_set_state(rover->pipeline, GST_STATE_NULL);
  gst_object_unref(rover->pipeline);
  rover->pipeline = nullptr;
  rover->webrtc = nullptr;
  // Owned by webrtcbin, which has just been disposed with the pipeline.
  rover->control_channel = nullptr;
}

bool start_pipeline(Rover * rover)
{
  stop_pipeline(rover);

  std::string description = teleop_rover_v2::build_send_pipeline(
    rover->config, rover->choice, "video-src");
  description += " ! webrtcbin name=webrtc bundle-policy=max-bundle latency=0";

  GError * error = nullptr;
  rover->pipeline = gst_parse_launch(description.c_str(), &error);
  if (error != nullptr) {
    const std::string message = error->message;
    g_error_free(error);
    fail(rover, "cannot build pipeline: " + message);
    return false;
  }

  rover->webrtc = gst_bin_get_by_name(GST_BIN(rover->pipeline), "webrtc");
  if (rover->webrtc == nullptr) {
    fail(rover, "pipeline has no webrtcbin");
    return false;
  }
  // gst_bin_get_by_name hands back a reference; the pipeline owns the element.
  gst_object_unref(rover->webrtc);

  if (rover->turn.has_value()) {
    if (const auto uri = turn_server_uri(*rover->turn)) {
      g_object_set(rover->webrtc, "turn-server", uri->c_str(), nullptr);
      std::printf("teleop-rover: TURN relay configured\n");
    } else {
      std::fprintf(stderr, "teleop-rover: no usable TURN url in hello_ack\n");
    }
  }
  if (rover->config.transport.force_relay) {
    g_object_set(
      rover->webrtc, "ice-transport-policy", GST_WEBRTC_ICE_TRANSPORT_POLICY_RELAY, nullptr);
    std::printf("teleop-rover: ICE restricted to relay candidates\n");
  }

  g_signal_connect(rover->webrtc, "on-ice-candidate", G_CALLBACK(on_ice_candidate), rover);

  GstBus * bus = gst_element_get_bus(rover->pipeline);
  gst_bus_add_signal_watch(bus);
  g_signal_connect(bus, "message", G_CALLBACK(on_bus_message), rover);
  gst_object_unref(bus);

  // Order matters here. webrtcbin refuses to create a data channel while it
  // is still in NULL ("is_closed"), and the channel has to exist before
  // negotiation or it will not appear in the offer. So: READY, then the
  // channel, then arm negotiation, then PLAYING.
  if (gst_element_set_state(rover->pipeline, GST_STATE_READY) == GST_STATE_CHANGE_FAILURE) {
    fail(rover, "pipeline will not reach READY");
    return false;
  }
  open_control_channel(rover);
  g_signal_connect(
    rover->webrtc, "on-negotiation-needed", G_CALLBACK(on_negotiation_needed), rover);

  if (gst_element_set_state(rover->pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    fail(rover, "pipeline will not reach PLAYING");
    return false;
  }
  std::printf(
    "teleop-rover: streaming %s via %s\n",
    rover->choice.codec.c_str(), rover->choice.encoder.c_str());
  return true;
}

void apply_remote_answer(Rover * rover, const std::string & sdp_text)
{
  GstSDPMessage * sdp = nullptr;
  if (gst_sdp_message_new_from_text(sdp_text.c_str(), &sdp) != GST_SDP_OK) {
    fail(rover, "answer is not parseable SDP");
    return;
  }
  GstWebRTCSessionDescription * answer =
    gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_ANSWER, sdp);
  GstPromise * promise = gst_promise_new();
  g_signal_emit_by_name(rover->webrtc, "set-remote-description", answer, promise);
  gst_promise_interrupt(promise);
  gst_promise_unref(promise);
  gst_webrtc_session_description_free(answer);
  std::printf("teleop-rover: answer applied, negotiation complete\n");
}

void handle_message(Rover * rover, const std::string & text)
{
  IncomingMessage message;
  try {
    message = teleop_rover_v2::parse_message(text);
  } catch (const teleop_rover_v2::ProtocolError & error) {
    std::fprintf(stderr, "teleop-rover: bad frame: %s\n", error.what());
    return;
  }

  switch (message.kind) {
    case IncomingMessage::Kind::kHelloAck:
      rover->turn = message.hello_ack.turn;
      std::printf(
        "teleop-rover: registered as %s, waiting for operator\n",
        message.hello_ack.robot_id.c_str());
      break;

    case IncomingMessage::Kind::kPeerReady:
      std::printf(
        "teleop-rover: paired (session %s, epoch %lu)\n",
        message.peer_ready.session_id.c_str(),
        static_cast<unsigned long>(message.peer_ready.session_epoch));
      rover->pending_session_id = message.peer_ready.session_id;
      rover->pending_session_epoch = message.peer_ready.session_epoch;
      halt(rover, "awaiting_control_link");
      if (message.peer_ready.create_offer) {
        start_pipeline(rover);
      }
      break;

    case IncomingMessage::Kind::kAnswer:
      if (rover->webrtc == nullptr) {
        std::fprintf(stderr, "teleop-rover: answer arrived with no session\n");
        break;
      }
      apply_remote_answer(rover, message.description.sdp);
      break;

    case IncomingMessage::Kind::kIce:
      if (rover->webrtc != nullptr) {
        g_signal_emit_by_name(
          rover->webrtc, "add-ice-candidate",
          static_cast<guint>(message.ice.sdp_mline_index), message.ice.candidate.c_str());
      }
      break;

    case IncomingMessage::Kind::kPeerLeft:
      std::printf("teleop-rover: operator left, stopping stream\n");
      rover->guard->disarm("peer_left");
      halt(rover, "peer_left");
      rover->pending_session_id.clear();
      if (rover->watchdog_source != 0) {
        g_source_remove(rover->watchdog_source);
        rover->watchdog_source = 0;
      }
      stop_pipeline(rover);
      break;

    case IncomingMessage::Kind::kError:
      fail(rover, "server rejected the session: " + message.error.code);
      break;

    case IncomingMessage::Kind::kOffer:
      // The rover is always the offerer in this protocol.
      std::fprintf(stderr, "teleop-rover: unexpected offer from the operator\n");
      break;

    case IncomingMessage::Kind::kUnknown:
      std::printf("teleop-rover: ignoring unknown message '%s'\n", message.type.c_str());
      break;
  }
}

void on_socket_message(
  SoupWebsocketConnection *, gint type, GBytes * payload, gpointer user_data)
{
  if (type != SOUP_WEBSOCKET_DATA_TEXT) {
    return;
  }
  gsize size = 0;
  const auto * data = static_cast<const char *>(g_bytes_get_data(payload, &size));
  handle_message(static_cast<Rover *>(user_data), std::string(data, size));
}

void on_socket_closed(SoupWebsocketConnection *, gpointer user_data)
{
  auto * rover = static_cast<Rover *>(user_data);
  std::fprintf(stderr, "teleop-rover: signaling closed\n");
  stop_pipeline(rover);
  g_main_loop_quit(rover->loop);
}

void on_connected(GObject * source, GAsyncResult * result, gpointer user_data)
{
  auto * rover = static_cast<Rover *>(user_data);
  GError * error = nullptr;
  rover->socket = soup_session_websocket_connect_finish(
    SOUP_SESSION(source), result, &error);
  if (error != nullptr) {
    const std::string message = error->message;
    g_error_free(error);
    fail(rover, "cannot reach signaling: " + message);
    return;
  }

  g_signal_connect(rover->socket, "message", G_CALLBACK(on_socket_message), rover);
  g_signal_connect(rover->socket, "closed", G_CALLBACK(on_socket_closed), rover);
  std::printf("teleop-rover: signaling connected\n");
  send_text(rover, teleop_rover_v2::build_hello(rover->robot_id));
}

}  // namespace

int main(int argc, char ** argv)
{
  gst_init(&argc, &argv);

  std::string config_path = "config/media_profiles.yaml";
  std::string robot_id = "rover-01";
  std::string server_override;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string flag = argv[i];
    if (flag == "--config") {
      config_path = argv[i + 1];
    } else if (flag == "--robot") {
      robot_id = argv[i + 1];
    } else if (flag == "--server") {
      server_override = argv[i + 1];
    } else {
      std::fprintf(stderr, "usage: %s [--config PATH] [--robot ID] [--server URL]\n", argv[0]);
      return 2;
    }
  }

  Rover rover;
  try {
    rover.config = teleop_rover_v2::load_media_config(config_path);
  } catch (const teleop_rover_v2::ConfigError & error) {
    std::fprintf(stderr, "teleop-rover: %s\n", error.what());
    return 2;
  }
  if (!server_override.empty()) {
    rover.config.transport.signaling_url = server_override;
  }
  rover.robot_id = robot_id;
  rover.guard = std::make_unique<ControlGuard>(
    robot_id,
    static_cast<std::uint32_t>(rover.config.control.watchdog_ms),
    static_cast<std::uint32_t>(rover.config.control.max_future_clock_skew_ms));

  const auto choice = teleop_rover_v2::select_encoder(rover.config.video.codec_preference);
  if (!choice) {
    std::fprintf(
      stderr,
      "teleop-rover: no codec in the preference list has a complete "
      "encoder/parser/payloader chain in this GStreamer build\n");
    return 2;
  }
  rover.choice = *choice;
  std::printf(
    "teleop-rover: %s via %s -> %s\n",
    choice->codec.c_str(), choice->encoder.c_str(), choice->payloader.c_str());

  rover.loop = g_main_loop_new(nullptr, FALSE);
  rover.session = soup_session_new();
  SoupMessage * request = soup_message_new(
    "GET", rover.config.transport.signaling_url.c_str());
  if (request == nullptr) {
    std::fprintf(
      stderr, "teleop-rover: bad signaling url %s\n",
      rover.config.transport.signaling_url.c_str());
    return 2;
  }
  std::printf("teleop-rover: connecting to %s\n", rover.config.transport.signaling_url.c_str());
  soup_session_websocket_connect_async(
    rover.session, request, nullptr, nullptr, G_PRIORITY_DEFAULT, nullptr,
    on_connected, &rover);
  g_object_unref(request);

  g_main_loop_run(rover.loop);

  if (rover.watchdog_source != 0) {
    g_source_remove(rover.watchdog_source);
  }
  stop_pipeline(&rover);
  if (rover.socket != nullptr) {
    g_object_unref(rover.socket);
  }
  g_object_unref(rover.session);
  g_main_loop_unref(rover.loop);
  return rover.exit_code;
}
