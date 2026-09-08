#include "teleop_rover_v2/control_guard.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace teleop_rover_v2
{

namespace
{
constexpr std::int64_t kNsPerMs = 1'000'000;
}

ControlGuard::ControlGuard(
  std::string robot_id,
  std::uint32_t watchdog_ms,
  std::uint32_t max_future_skew_ms)
: robot_id_(std::move(robot_id)),
  watchdog_ns_(static_cast<std::int64_t>(watchdog_ms) * kNsPerMs),
  future_skew_ns_(static_cast<std::int64_t>(max_future_skew_ms) * kNsPerMs)
{
  if (robot_id_.empty()) {throw std::invalid_argument("robot_id must not be empty");}
  if (watchdog_ms == 0) {throw std::invalid_argument("watchdog_ms must be positive");}
}

bool ControlGuard::arm(
  const std::string & session_id,
  std::uint64_t epoch,
  std::int64_t now_steady_ns)
{
  if (session_id.empty() || epoch <= epoch_) {return false;}
  session_id_ = session_id;
  epoch_ = epoch;
  last_sequence_ = 0;
  have_sequence_ = false;
  last_command_steady_ns_ = now_steady_ns;
  armed_ = true;
  stop_reason_.clear();
  return true;
}

void ControlGuard::disarm(const std::string & reason)
{
  armed_ = false;
  stop_reason_ = reason.empty() ? "disarmed" : reason;
}

CommandResult ControlGuard::reject(RejectReason reason) const
{
  return CommandResult{std::nullopt, reason};
}

CommandResult ControlGuard::accept(
  const MotionCommand & command,
  std::int64_t now_unix_ns,
  std::int64_t now_steady_ns)
{
  if (!armed_) {return reject(RejectReason::kNotArmed);}
  if (
    command.robot_id != robot_id_ || command.session_id != session_id_ ||
    command.session_epoch != epoch_)
  {
    return reject(RejectReason::kWrongSession);
  }
  if (have_sequence_ && command.sequence <= last_sequence_) {
    return reject(RejectReason::kOldSequence);
  }
  if (command.valid_for_ms == 0 || command.valid_for_ms > 1000) {
    return reject(RejectReason::kInvalidValue);
  }
  if (!std::isfinite(command.linear_mps) || !std::isfinite(command.angular_rps)) {
    return reject(RejectReason::kInvalidValue);
  }
  if (command.sent_unix_ns > now_unix_ns + future_skew_ns_) {
    return reject(RejectReason::kFutureTimestamp);
  }
  const auto age_ns = now_unix_ns - command.sent_unix_ns;
  const auto valid_ns = static_cast<std::int64_t>(command.valid_for_ms) * kNsPerMs;
  if (age_ns > valid_ns) {return reject(RejectReason::kExpired);}

  last_sequence_ = command.sequence;
  have_sequence_ = true;
  last_command_steady_ns_ = now_steady_ns;
  const double linear = command.deadman ? command.linear_mps : 0.0;
  const double angular = command.deadman ? command.angular_rps : 0.0;
  return CommandResult{AcceptedMotion{linear, angular, command.deadman}, std::nullopt};
}

bool ControlGuard::poll_watchdog(std::int64_t now_steady_ns)
{
  if (!armed_) {return false;}
  if (now_steady_ns - last_command_steady_ns_ <= watchdog_ns_) {return false;}
  disarm("command_watchdog");
  return true;
}

}  // namespace teleop_rover_v2
