#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace teleop_rover_v2
{

struct MotionCommand
{
  std::string robot_id;
  std::string session_id;
  std::uint64_t session_epoch{0};
  std::uint64_t sequence{0};
  std::int64_t sent_unix_ns{0};
  std::uint32_t valid_for_ms{0};
  double linear_mps{0.0};
  double angular_rps{0.0};
  bool deadman{false};
};

enum class RejectReason
{
  kNotArmed,
  kWrongSession,
  kOldSequence,
  kExpired,
  kFutureTimestamp,
  kInvalidValue,
};

struct AcceptedMotion
{
  double linear_mps;
  double angular_rps;
  bool deadman;
};

struct CommandResult
{
  std::optional<AcceptedMotion> motion;
  std::optional<RejectReason> rejected;
};

class ControlGuard
{
public:
  explicit ControlGuard(
    std::string robot_id,
    std::uint32_t watchdog_ms = 250,
    std::uint32_t max_future_skew_ms = 50);

  bool arm(const std::string & session_id, std::uint64_t epoch, std::int64_t now_steady_ns);
  void disarm(const std::string & reason);
  CommandResult accept(
    const MotionCommand & command,
    std::int64_t now_unix_ns,
    std::int64_t now_steady_ns);
  bool poll_watchdog(std::int64_t now_steady_ns);

  [[nodiscard]] bool armed() const noexcept {return armed_;}
  [[nodiscard]] std::uint64_t epoch() const noexcept {return epoch_;}
  [[nodiscard]] const std::string & stop_reason() const noexcept {return stop_reason_;}

private:
  CommandResult reject(RejectReason reason) const;

  std::string robot_id_;
  std::string session_id_;
  std::uint64_t epoch_{0};
  std::uint64_t last_sequence_{0};
  bool have_sequence_{false};
  bool armed_{false};
  std::int64_t last_command_steady_ns_{0};
  std::int64_t watchdog_ns_;
  std::int64_t future_skew_ns_;
  std::string stop_reason_{"not_armed"};
};

}  // namespace teleop_rover_v2
