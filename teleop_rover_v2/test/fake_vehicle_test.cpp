#include "teleop_rover_v2/fake_vehicle.hpp"

#include <gtest/gtest.h>

#include <cmath>

using teleop_rover_v2::AcceptedMotion;
using teleop_rover_v2::FakeVehicle;

namespace
{
AcceptedMotion driving(double linear, double angular = 0.0)
{
  return AcceptedMotion{linear, angular, true};
}

void advance(FakeVehicle & vehicle, const AcceptedMotion & motion, double seconds)
{
  constexpr double kTick = 0.02;
  for (int i = 0; i < static_cast<int>(seconds / kTick + 0.5); ++i) {
    vehicle.apply(motion, kTick);
  }
}
}  // namespace

TEST(FakeVehicle, StartsStoppedAtTheOrigin)
{
  const FakeVehicle vehicle;
  EXPECT_EQ(vehicle.pose().x_m, 0.0);
  EXPECT_EQ(vehicle.pose().y_m, 0.0);
  EXPECT_FALSE(vehicle.moving());
  EXPECT_EQ(vehicle.stop_reason(), "not_started");
}

TEST(FakeVehicle, DrivesForwardAlongX)
{
  FakeVehicle vehicle;
  advance(vehicle, driving(0.5), 2.0);
  EXPECT_NEAR(vehicle.pose().x_m, 1.0, 1e-9);
  EXPECT_NEAR(vehicle.pose().y_m, 0.0, 1e-9);
  EXPECT_TRUE(vehicle.moving());
}

TEST(FakeVehicle, ReverseMovesBackwards)
{
  FakeVehicle vehicle;
  advance(vehicle, driving(-0.5), 2.0);
  EXPECT_NEAR(vehicle.pose().x_m, -1.0, 1e-9);
}

TEST(FakeVehicle, YawTurnsInPlace)
{
  FakeVehicle vehicle;
  advance(vehicle, driving(0.0, 1.0), 1.0);
  EXPECT_NEAR(vehicle.pose().heading_rad, 1.0, 1e-9);
  EXPECT_NEAR(vehicle.pose().x_m, 0.0, 1e-9);
  EXPECT_NEAR(vehicle.pose().y_m, 0.0, 1e-9);
  EXPECT_TRUE(vehicle.moving());
}

TEST(FakeVehicle, HeadingStaysWrapped)
{
  FakeVehicle vehicle;
  advance(vehicle, driving(0.0, 2.0), 10.0);
  EXPECT_LE(std::abs(vehicle.pose().heading_rad), M_PI + 1e-9);
}

TEST(FakeVehicle, WithoutTheDeadmanNothingMoves)
{
  FakeVehicle vehicle;
  advance(vehicle, AcceptedMotion{1.0, 1.0, false}, 2.0);
  EXPECT_NEAR(vehicle.pose().x_m, 0.0, 1e-9);
  EXPECT_NEAR(vehicle.pose().heading_rad, 0.0, 1e-9);
  EXPECT_FALSE(vehicle.moving());
}

TEST(FakeVehicle, StopHaltsAndRecordsWhy)
{
  FakeVehicle vehicle;
  advance(vehicle, driving(0.5), 1.0);
  ASSERT_TRUE(vehicle.moving());

  vehicle.stop("watchdog");
  EXPECT_FALSE(vehicle.moving());
  EXPECT_EQ(vehicle.linear_mps(), 0.0);
  EXPECT_EQ(vehicle.angular_rps(), 0.0);
  EXPECT_EQ(vehicle.stop_reason(), "watchdog");
}

TEST(FakeVehicle, StopLeavesThePoseWhereItWas)
{
  FakeVehicle vehicle;
  advance(vehicle, driving(0.5), 2.0);
  const auto x = vehicle.pose().x_m;
  vehicle.stop("operator_left");
  EXPECT_EQ(vehicle.pose().x_m, x);
}

TEST(FakeVehicle, StallDoesNotTeleport)
{
  // One apply may advance at most kMaxTickSeconds no matter the elapsed time.
  FakeVehicle vehicle;
  vehicle.apply(driving(1.0), 60.0);
  EXPECT_NEAR(vehicle.pose().x_m, FakeVehicle::kMaxTickSeconds, 1e-9);
}

TEST(FakeVehicle, NonFiniteValuesAreIgnored)
{
  FakeVehicle vehicle;
  vehicle.apply(AcceptedMotion{std::nan(""), INFINITY, true}, 0.02);
  EXPECT_EQ(vehicle.linear_mps(), 0.0);
  EXPECT_EQ(vehicle.angular_rps(), 0.0);
  EXPECT_TRUE(std::isfinite(vehicle.pose().x_m));
  EXPECT_TRUE(std::isfinite(vehicle.pose().heading_rad));
}

TEST(FakeVehicle, TurningWhileDrivingCurvesThePath)
{
  FakeVehicle vehicle;
  advance(vehicle, driving(1.0, 1.0), 2.0);
  // A unit-speed unit-rate turn traces a circle of radius 1; after pi
  // radians the vehicle sits near (0, 2).
  EXPECT_GT(vehicle.pose().y_m, 0.5);
  EXPECT_LT(std::abs(vehicle.pose().x_m), 1.5);
}
