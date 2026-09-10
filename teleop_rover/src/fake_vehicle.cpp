#include "teleop_rover/fake_vehicle.hpp"

#include <algorithm>
#include <cmath>

namespace teleop_rover
{
namespace
{
double sanitize(double value)
{
  return std::isfinite(value) ? value : 0.0;
}
}  // namespace

void FakeVehicle::apply(const AcceptedMotion & motion, double dt_s)
{
  // A command without the deadman held commands no motion, whatever its
  // velocity fields say.
  linear_mps_ = motion.deadman ? sanitize(motion.linear_mps) : 0.0;
  angular_rps_ = motion.deadman ? sanitize(motion.angular_rps) : 0.0;
  stop_reason_.clear();

  const double tick = std::clamp(sanitize(dt_s), 0.0, kMaxTickSeconds);
  // Unicycle integration: heading advances first, then the step is taken
  // along the new heading.
  pose_.heading_rad += angular_rps_ * tick;
  // Keep the heading in (-pi, pi] so a long run does not accumulate a number
  // too large to print usefully.
  pose_.heading_rad = std::remainder(pose_.heading_rad, 2.0 * M_PI);
  pose_.x_m += linear_mps_ * tick * std::cos(pose_.heading_rad);
  pose_.y_m += linear_mps_ * tick * std::sin(pose_.heading_rad);
}

void FakeVehicle::stop(const std::string & reason)
{
  linear_mps_ = 0.0;
  angular_rps_ = 0.0;
  stop_reason_ = reason;
}

bool FakeVehicle::moving() const noexcept
{
  return std::abs(linear_mps_) > kMotionEpsilon ||
         std::abs(angular_rps_) > kMotionEpsilon;
}

}  // namespace teleop_rover
