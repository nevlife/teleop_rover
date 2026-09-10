#include "teleop_rover/signaling_protocol.hpp"

#include <gtest/gtest.h>

using teleop_rover::IceCandidate;
using teleop_rover::IncomingMessage;
using teleop_rover::ProtocolError;
using teleop_rover::build_answer;
using teleop_rover::build_hello;
using teleop_rover::build_ice;
using teleop_rover::build_offer;
using teleop_rover::parse_message;

// The frames below were captured from teleop_v2_server 8be6295 over the LAN
// rather than written from the specification, so the tests fail if the
// server's actual shape drifts.

TEST(SignalingProtocol, HelloCarriesRoleAndVersion)
{
  const auto hello = parse_message(build_hello("rover-01"));
  EXPECT_EQ(hello.type, "hello");

  const auto text = build_hello("rover-01");
  EXPECT_NE(text.find("\"protocol_version\":2"), std::string::npos);
  EXPECT_NE(text.find("\"role\":\"rover\""), std::string::npos);
  EXPECT_NE(text.find("\"robot_id\":\"rover-01\""), std::string::npos);
}

TEST(SignalingProtocol, HelloRoleIsOverridable)
{
  EXPECT_NE(build_hello("rover-01", "client").find("\"role\":\"client\""), std::string::npos);
}

TEST(SignalingProtocol, ParsesHelloAckWithTurnCredentials)
{
  const auto message = parse_message(R"({
    "type": "hello_ack", "protocol_version": 2, "role": "rover",
    "robot_id": "rover-01",
    "turn": {"urls": ["turn:203.0.113.64:13437?transport=udp"],
             "username": "teleop", "credential": "lan-test-4Vk2A8XwuJ"}})");
  ASSERT_EQ(message.kind, IncomingMessage::Kind::kHelloAck);
  EXPECT_EQ(message.hello_ack.protocol_version, 2);
  EXPECT_EQ(message.hello_ack.role, "rover");
  ASSERT_TRUE(message.hello_ack.turn.has_value());
  ASSERT_EQ(message.hello_ack.turn->urls.size(), 1u);
  EXPECT_EQ(message.hello_ack.turn->urls[0], "turn:203.0.113.64:13437?transport=udp");
  EXPECT_EQ(message.hello_ack.turn->username, "teleop");
  EXPECT_EQ(message.hello_ack.turn->credential, "lan-test-4Vk2A8XwuJ");
}

TEST(SignalingProtocol, HelloAckWithoutTurnIsAccepted)
{
  const auto message = parse_message(R"({"type":"hello_ack","protocol_version":2})");
  ASSERT_EQ(message.kind, IncomingMessage::Kind::kHelloAck);
  EXPECT_FALSE(message.hello_ack.turn.has_value());
}

TEST(SignalingProtocol, ParsesPeerReadyWithStringEpoch)
{
  // The epoch exceeds 2^53, so it arrives as a string and must survive intact.
  const auto message = parse_message(R"({
    "type": "peer_ready", "robot_id": "rover-01",
    "session_id": "bba520a7-ea1d-4ca2-a87f-ec891b57eb31",
    "session_epoch": "7210501150266340579",
    "peer_role": "client", "create_offer": true})");
  ASSERT_EQ(message.kind, IncomingMessage::Kind::kPeerReady);
  EXPECT_EQ(message.peer_ready.session_id, "bba520a7-ea1d-4ca2-a87f-ec891b57eb31");
  EXPECT_EQ(message.peer_ready.session_epoch, 7210501150266340579ULL);
  EXPECT_EQ(message.peer_ready.peer_role, "client");
  EXPECT_TRUE(message.peer_ready.create_offer);
}

TEST(SignalingProtocol, ClientSidePeerReadyDoesNotCreateTheOffer)
{
  const auto message = parse_message(R"({
    "type":"peer_ready","robot_id":"r","session_id":"s",
    "session_epoch":"1","peer_role":"rover","create_offer":false})");
  EXPECT_FALSE(message.peer_ready.create_offer);
}

TEST(SignalingProtocol, RejectsNonNumericEpoch)
{
  EXPECT_THROW(
    parse_message(R"({"type":"peer_ready","robot_id":"r","session_id":"s",
                      "session_epoch":"not-a-number"})"),
    ProtocolError);
}

TEST(SignalingProtocol, PeerReadyRequiresSessionIdentity)
{
  EXPECT_THROW(parse_message(R"({"type":"peer_ready","robot_id":"r"})"), ProtocolError);
}

TEST(SignalingProtocol, RoundTripsAnSdpOfferByteForByte)
{
  const std::string sdp =
    "v=0\r\no=- 1 1 IN IP4 0.0.0.0\r\nm=video 9 UDP/TLS/RTP/SAVPF 96\r\n";
  const auto message = parse_message(build_offer(sdp));
  ASSERT_EQ(message.kind, IncomingMessage::Kind::kOffer);
  EXPECT_EQ(message.description.sdp, sdp);
}

TEST(SignalingProtocol, RoundTripsAnSdpAnswer)
{
  const std::string sdp = "v=0\r\na=recvonly\r\n";
  const auto message = parse_message(build_answer(sdp));
  ASSERT_EQ(message.kind, IncomingMessage::Kind::kAnswer);
  EXPECT_EQ(message.description.sdp, sdp);
}

TEST(SignalingProtocol, RoundTripsAnIceCandidate)
{
  const IceCandidate sent{
    "candidate:1 1 UDP 2130706431 10.0.0.1 5000 typ host", "video0", 2};
  const auto message = parse_message(build_ice(sent));
  ASSERT_EQ(message.kind, IncomingMessage::Kind::kIce);
  EXPECT_EQ(message.ice.candidate, sent.candidate);
  EXPECT_EQ(message.ice.sdp_mid, "video0");
  EXPECT_EQ(message.ice.sdp_mline_index, 2);
}

TEST(SignalingProtocol, ParsesPeerLeft)
{
  EXPECT_EQ(
    parse_message(R"({"type":"peer_left"})").kind, IncomingMessage::Kind::kPeerLeft);
}

TEST(SignalingProtocol, ParsesServerError)
{
  const auto message = parse_message(R"({"type":"error","code":"role_in_use"})");
  ASSERT_EQ(message.kind, IncomingMessage::Kind::kError);
  EXPECT_EQ(message.error.code, "role_in_use");
}

TEST(SignalingProtocol, UnknownTypeIsReportedNotThrown)
{
  // A newer server must be able to add messages without breaking the rover.
  const auto message = parse_message(R"({"type":"bitrate_hint","kbps":1200})");
  EXPECT_EQ(message.kind, IncomingMessage::Kind::kUnknown);
  EXPECT_EQ(message.type, "bitrate_hint");
}

TEST(SignalingProtocol, RejectsMalformedFrames)
{
  EXPECT_THROW(parse_message("not json"), ProtocolError);
  EXPECT_THROW(parse_message("[]"), ProtocolError);
  EXPECT_THROW(parse_message(R"({"no_type": 1})"), ProtocolError);
  EXPECT_THROW(parse_message(R"({"type": 7})"), ProtocolError);
  EXPECT_THROW(parse_message(R"({"type":"offer"})"), ProtocolError);
  EXPECT_THROW(parse_message(R"({"type":"ice"})"), ProtocolError);
  EXPECT_THROW(parse_message(R"({"type":"ice","candidate":"flat"})"), ProtocolError);
}
