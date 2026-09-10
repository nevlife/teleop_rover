#include "teleop_rover/control_guard.hpp"

#include <gtest/gtest.h>

#include <limits>

using teleop_rover::ControlGuard;
using teleop_rover::MotionCommand;
using teleop_rover::RejectReason;

namespace
{
MotionCommand command(std::uint64_t sequence, std::int64_t sent_ns)
{
  return MotionCommand{
    "rover-01", "session-a", 10, sequence, sent_ns, 200, 0.5, 0.2, true};
}
}

TEST(ControlGuard, RequiresIncreasingEpochAndSequence)
{
  ControlGuard guard("rover-01");
  EXPECT_TRUE(guard.arm("session-a", 10, 0));
  EXPECT_FALSE(guard.arm("session-b", 10, 0));
  EXPECT_TRUE(guard.accept(command(0, 1'000'000'000), 1'100'000'000, 100).motion.has_value());
  const auto duplicate = guard.accept(command(0, 1'100'000'000), 1'100'000'000, 110);
  ASSERT_TRUE(duplicate.rejected.has_value());
  EXPECT_EQ(*duplicate.rejected, RejectReason::kOldSequence);
}

TEST(ControlGuard, RejectsExpiredAndInvalidValues)
{
  ControlGuard guard("rover-01");
  ASSERT_TRUE(guard.arm("session-a", 10, 0));
  auto expired = guard.accept(command(1, 1'000'000'000), 1'300'000'000, 10);
  ASSERT_TRUE(expired.rejected.has_value());
  EXPECT_EQ(*expired.rejected, RejectReason::kExpired);

  auto invalid = command(2, 1'300'000'000);
  invalid.linear_mps = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(*guard.accept(invalid, 1'300'000'000, 20).rejected, RejectReason::kInvalidValue);
}

TEST(ControlGuard, WatchdogLatchesUntilNewEpoch)
{
  ControlGuard guard("rover-01", 250);
  ASSERT_TRUE(guard.arm("session-a", 10, 0));
  ASSERT_TRUE(guard.accept(command(1, 1'000'000'000), 1'000'000'000, 10).motion.has_value());
  EXPECT_FALSE(guard.poll_watchdog(250'000'010));
  EXPECT_TRUE(guard.poll_watchdog(260'000'011));
  EXPECT_FALSE(guard.armed());
  EXPECT_EQ(guard.stop_reason(), "command_watchdog");
  EXPECT_FALSE(guard.accept(command(2, 1'000'000'000), 1'000'000'000, 270'000'000).motion);
  EXPECT_FALSE(guard.arm("session-a", 10, 270'000'000));
  EXPECT_TRUE(guard.arm("session-b", 11, 270'000'000));
}

TEST(ControlGuard, DeadmanReleaseProducesZero)
{
  ControlGuard guard("rover-01");
  ASSERT_TRUE(guard.arm("session-a", 10, 0));
  auto cmd = command(1, 1'000'000'000);
  cmd.deadman = false;
  const auto result = guard.accept(cmd, 1'000'000'000, 1);
  ASSERT_TRUE(result.motion.has_value());
  EXPECT_DOUBLE_EQ(result.motion->linear_mps, 0.0);
  EXPECT_DOUBLE_EQ(result.motion->angular_rps, 0.0);
}
