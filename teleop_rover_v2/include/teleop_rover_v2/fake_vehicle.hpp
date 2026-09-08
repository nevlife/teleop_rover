#pragma once

#include "teleop_rover_v2/control_guard.hpp"

#include <string>

namespace teleop_rover_v2
{

/// Stand-in for the drive-by-wire layer.
///
/// Integrates accepted motion commands into a pose so the control path can be
/// exercised end to end without a vehicle attached. It deliberately models
/// only what the teleoperation loop can observe: where the commands would
/// have taken the rover, and whether it is currently moving.
class FakeVehicle
{
public:
  struct Pose
  {
    double x_m{0.0};
    double y_m{0.0};
    double heading_rad{0.0};
  };

  /// Apply a command accepted by the guard. `dt_s` is clamped, so a stalled
  /// caller cannot teleport the pose.
  void apply(const AcceptedMotion & motion, double dt_s);

  /// Bring the vehicle to a stop. Called when the guard rejects, when the
  /// watchdog trips, and when the operator leaves.
  void stop(const std::string & reason);

  [[nodiscard]] const Pose & pose() const noexcept {return pose_;}
  [[nodiscard]] double linear_mps() const noexcept {return linear_mps_;}
  [[nodiscard]] double angular_rps() const noexcept {return angular_rps_;}
  [[nodiscard]] bool moving() const noexcept;
  [[nodiscard]] const std::string & stop_reason() const noexcept {return stop_reason_;}

  static constexpr double kMaxTickSeconds = 0.1;
  /// Below this the vehicle counts as stopped, so floating-point dust does
  /// not read as motion.
  static constexpr double kMotionEpsilon = 1e-6;

private:
  Pose pose_;
  double linear_mps_{0.0};
  double angular_rps_{0.0};
  std::string stop_reason_{"not_started"};
};

}  // namespace teleop_rover_v2
