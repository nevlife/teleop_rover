#include "teleop_rover_v2/signaling_protocol.hpp"

#include <json-glib/json-glib.h>

#include <cerrno>
#include <cstdlib>
#include <memory>

namespace teleop_rover_v2
{
namespace
{

using ParserPtr = std::unique_ptr<JsonParser, decltype(&g_object_unref)>;
using GeneratorPtr = std::unique_ptr<JsonGenerator, decltype(&g_object_unref)>;
using NodePtr = std::unique_ptr<JsonNode, decltype(&json_node_free)>;

std::string generate(JsonBuilder * builder)
{
  NodePtr root(json_builder_get_root(builder), json_node_free);
  GeneratorPtr generator(json_generator_new(), g_object_unref);
  json_generator_set_root(generator.get(), root.get());
  gsize length = 0;
  gchar * text = json_generator_to_data(generator.get(), &length);
  std::string out(text, length);
  g_free(text);
  return out;
}

const gchar * optional_string(JsonObject * object, const char * key)
{
  if (!json_object_has_member(object, key)) {
    return nullptr;
  }
  JsonNode * node = json_object_get_member(object, key);
  if (JSON_NODE_TYPE(node) != JSON_NODE_VALUE ||
    json_node_get_value_type(node) != G_TYPE_STRING)
  {
    return nullptr;
  }
  return json_node_get_string(node);
}

std::string required_string(JsonObject * object, const char * key, const char * context)
{
  const gchar * value = optional_string(object, key);
  if (value == nullptr) {
    throw ProtocolError(std::string(context) + " is missing a string '" + key + "'");
  }
  return value;
}

/// `session_epoch` arrives as a decimal string. Parsing it as a JSON number
/// would lose precision above 2^53.
std::uint64_t parse_epoch(JsonObject * object)
{
  if (!json_object_has_member(object, "session_epoch")) {
    return 0;
  }
  JsonNode * node = json_object_get_member(object, "session_epoch");
  if (json_node_get_value_type(node) == G_TYPE_STRING) {
    const gchar * text = json_node_get_string(node);
    errno = 0;
    char * end = nullptr;
    const auto value = std::strtoull(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0') {
      throw ProtocolError("'session_epoch' is not a decimal integer");
    }
    return value;
  }
  return static_cast<std::uint64_t>(json_node_get_int(node));
}

std::optional<TurnCredentials> parse_turn(JsonObject * object)
{
  if (!json_object_has_member(object, "turn")) {
    return std::nullopt;
  }
  JsonNode * node = json_object_get_member(object, "turn");
  if (JSON_NODE_TYPE(node) != JSON_NODE_OBJECT) {
    return std::nullopt;
  }
  JsonObject * turn = json_node_get_object(node);
  TurnCredentials credentials;
  if (json_object_has_member(turn, "urls")) {
    JsonNode * urls_node = json_object_get_member(turn, "urls");
    if (JSON_NODE_TYPE(urls_node) == JSON_NODE_ARRAY) {
      JsonArray * urls = json_node_get_array(urls_node);
      const guint count = json_array_get_length(urls);
      for (guint i = 0; i < count; ++i) {
        const gchar * url = json_array_get_string_element(urls, i);
        if (url != nullptr) {
          credentials.urls.emplace_back(url);
        }
      }
    }
  }
  if (const gchar * username = optional_string(turn, "username")) {
    credentials.username = username;
  }
  if (const gchar * credential = optional_string(turn, "credential")) {
    credentials.credential = credential;
  }
  if (credentials.urls.empty()) {
    return std::nullopt;
  }
  return credentials;
}

}  // namespace

std::string build_hello(const std::string & robot_id, const std::string & role)
{
  std::unique_ptr<JsonBuilder, decltype(&g_object_unref)> builder(
    json_builder_new(), g_object_unref);
  json_builder_begin_object(builder.get());
  json_builder_set_member_name(builder.get(), "type");
  json_builder_add_string_value(builder.get(), "hello");
  json_builder_set_member_name(builder.get(), "protocol_version");
  json_builder_add_int_value(builder.get(), kProtocolVersion);
  json_builder_set_member_name(builder.get(), "role");
  json_builder_add_string_value(builder.get(), role.c_str());
  json_builder_set_member_name(builder.get(), "robot_id");
  json_builder_add_string_value(builder.get(), robot_id.c_str());
  json_builder_end_object(builder.get());
  return generate(builder.get());
}

namespace
{
std::string build_description(const char * type, const std::string & sdp)
{
  std::unique_ptr<JsonBuilder, decltype(&g_object_unref)> builder(
    json_builder_new(), g_object_unref);
  json_builder_begin_object(builder.get());
  json_builder_set_member_name(builder.get(), "type");
  json_builder_add_string_value(builder.get(), type);
  json_builder_set_member_name(builder.get(), "sdp");
  json_builder_add_string_value(builder.get(), sdp.c_str());
  json_builder_end_object(builder.get());
  return generate(builder.get());
}
}  // namespace

std::string build_offer(const std::string & sdp) {return build_description("offer", sdp);}
std::string build_answer(const std::string & sdp) {return build_description("answer", sdp);}

std::string build_ice(const IceCandidate & candidate)
{
  std::unique_ptr<JsonBuilder, decltype(&g_object_unref)> builder(
    json_builder_new(), g_object_unref);
  json_builder_begin_object(builder.get());
  json_builder_set_member_name(builder.get(), "type");
  json_builder_add_string_value(builder.get(), "ice");
  json_builder_set_member_name(builder.get(), "candidate");
  json_builder_begin_object(builder.get());
  json_builder_set_member_name(builder.get(), "candidate");
  json_builder_add_string_value(builder.get(), candidate.candidate.c_str());
  json_builder_set_member_name(builder.get(), "sdpMid");
  json_builder_add_string_value(builder.get(), candidate.sdp_mid.c_str());
  json_builder_set_member_name(builder.get(), "sdpMLineIndex");
  json_builder_add_int_value(builder.get(), candidate.sdp_mline_index);
  json_builder_end_object(builder.get());
  json_builder_end_object(builder.get());
  return generate(builder.get());
}

IncomingMessage parse_message(const std::string & json)
{
  ParserPtr parser(json_parser_new(), g_object_unref);
  GError * error = nullptr;
  if (!json_parser_load_from_data(
      parser.get(), json.c_str(), static_cast<gssize>(json.size()), &error))
  {
    const std::string message = error != nullptr ? error->message : "unknown";
    if (error != nullptr) {
      g_error_free(error);
    }
    throw ProtocolError("not valid JSON: " + message);
  }

  JsonNode * root = json_parser_get_root(parser.get());
  if (root == nullptr || JSON_NODE_TYPE(root) != JSON_NODE_OBJECT) {
    throw ProtocolError("frame is not a JSON object");
  }
  JsonObject * object = json_node_get_object(root);

  IncomingMessage message;
  message.type = required_string(object, "type", "frame");

  if (message.type == "hello_ack") {
    message.kind = IncomingMessage::Kind::kHelloAck;
    if (json_object_has_member(object, "protocol_version")) {
      message.hello_ack.protocol_version =
        static_cast<int>(json_object_get_int_member(object, "protocol_version"));
    }
    if (const gchar * role = optional_string(object, "role")) {
      message.hello_ack.role = role;
    }
    if (const gchar * robot_id = optional_string(object, "robot_id")) {
      message.hello_ack.robot_id = robot_id;
    }
    message.hello_ack.turn = parse_turn(object);
  } else if (message.type == "peer_ready") {
    message.kind = IncomingMessage::Kind::kPeerReady;
    message.peer_ready.robot_id = required_string(object, "robot_id", "peer_ready");
    message.peer_ready.session_id = required_string(object, "session_id", "peer_ready");
    message.peer_ready.session_epoch = parse_epoch(object);
    if (const gchar * peer_role = optional_string(object, "peer_role")) {
      message.peer_ready.peer_role = peer_role;
    }
    if (json_object_has_member(object, "create_offer")) {
      message.peer_ready.create_offer = json_object_get_boolean_member(object, "create_offer");
    }
  } else if (message.type == "peer_left") {
    message.kind = IncomingMessage::Kind::kPeerLeft;
  } else if (message.type == "offer" || message.type == "answer") {
    message.kind = message.type == "offer" ?
      IncomingMessage::Kind::kOffer : IncomingMessage::Kind::kAnswer;
    message.description.sdp = required_string(object, "sdp", message.type.c_str());
  } else if (message.type == "ice") {
    message.kind = IncomingMessage::Kind::kIce;
    if (!json_object_has_member(object, "candidate")) {
      throw ProtocolError("ice is missing 'candidate'");
    }
    JsonNode * node = json_object_get_member(object, "candidate");
    if (JSON_NODE_TYPE(node) != JSON_NODE_OBJECT) {
      throw ProtocolError("ice 'candidate' is not an object");
    }
    JsonObject * candidate = json_node_get_object(node);
    message.ice.candidate = required_string(candidate, "candidate", "ice candidate");
    if (const gchar * mid = optional_string(candidate, "sdpMid")) {
      message.ice.sdp_mid = mid;
    }
    if (json_object_has_member(candidate, "sdpMLineIndex")) {
      message.ice.sdp_mline_index =
        static_cast<int>(json_object_get_int_member(candidate, "sdpMLineIndex"));
    }
  } else if (message.type == "error") {
    message.kind = IncomingMessage::Kind::kError;
    if (const gchar * code = optional_string(object, "code")) {
      message.error.code = code;
    }
  }

  return message;
}

}  // namespace teleop_rover_v2
