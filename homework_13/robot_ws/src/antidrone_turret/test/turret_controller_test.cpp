#include <gtest/gtest.h>

#include "antidrone_turret/turret_controller.hpp"

namespace {

constexpr float kConfidenceThreshold = 0.80f;
constexpr float kMaxDistanceM = 30.0f;

TEST(EvaluateTargetTest, NotVisibleIsTargetNone)
{
  EXPECT_EQ(turret_controller::evaluate_target(false, 0.99f, kConfidenceThreshold), turret_controller::TargetStates::TARGET_NONE);
}

TEST(EvaluateTargetTest, BelowThresholdIsLowConfidence)
{
  EXPECT_EQ(turret_controller::evaluate_target(true, 0.79f, kConfidenceThreshold), turret_controller::TargetStates::TARGET_LOW_CONFIDENCE);
}

TEST(EvaluateTargetTest, AtOrAboveThresholdIsLocked)
{
  EXPECT_EQ(turret_controller::evaluate_target(true, 0.80f, kConfidenceThreshold), turret_controller::TargetStates::TARGET_LOCKED);
  EXPECT_EQ(turret_controller::evaluate_target(true, 0.95f, kConfidenceThreshold), turret_controller::TargetStates::TARGET_LOCKED);
}

TEST(ServoCommandTest, RightWhenXGreaterThanCenter)
{
  const auto cmd = turret_controller::make_servo_command(420.0f);
  EXPECT_EQ(cmd.direction, turret_controller::ServoDirections::RIGHT);
  EXPECT_FLOAT_EQ(cmd.target_x, 420.0f);
  EXPECT_FLOAT_EQ(cmd.error_x, 100.0f);
  EXPECT_GT(cmd.error_x, 0.0f);
}

TEST(ServoCommandTest, LeftWhenXLessThanCenter)
{
  const auto cmd = turret_controller::make_servo_command(200.0f);
  EXPECT_EQ(cmd.direction, turret_controller::ServoDirections::LEFT);
  EXPECT_LT(cmd.error_x, 0.0f);
}

TEST(ServoCommandTest, CenterWhenXEqualsCenter)
{
  const auto cmd = turret_controller::make_servo_command(320.0f);
  EXPECT_EQ(cmd.direction, turret_controller::ServoDirections::CENTER);
  EXPECT_FLOAT_EQ(cmd.error_x, 0.0f);
}

TEST(GimbalCommandTest, UpWhenYLessThanCenter)
{
  const auto cmd = turret_controller::make_gimbal_command(180.0f);
  EXPECT_EQ(cmd.direction, turret_controller::GimbalDirections::UP);
  EXPECT_FLOAT_EQ(cmd.target_y, 180.0f);
  EXPECT_FLOAT_EQ(cmd.error_y, 60.0f);
  EXPECT_GT(cmd.error_y, 0.0f);
}

TEST(GimbalCommandTest, DownWhenYGreaterThanCenter)
{
  const auto cmd = turret_controller::make_gimbal_command(300.0f);
  EXPECT_EQ(cmd.direction, turret_controller::GimbalDirections::DOWN);
  EXPECT_LT(cmd.error_y, 0.0f);
}

TEST(GimbalCommandTest, CenterWhenYEqualsCenter)
{
  const auto cmd = turret_controller::make_gimbal_command(240.0f);
  EXPECT_EQ(cmd.direction, turret_controller::GimbalDirections::CENTER);
  EXPECT_FLOAT_EQ(cmd.error_y, 0.0f);
}

TEST(TriggerDecisionTest, CloseAndReadyIsRequested)
{
  EXPECT_EQ(turret_controller::make_trigger_decision(25.0f, kMaxDistanceM, turret_controller::ActuatorStatuses::READY),
            turret_controller::TriggerStates::TRIGGER_REQUESTED);
}

TEST(TriggerDecisionTest, CloseAndReloadingIsReloading)
{
  EXPECT_EQ(turret_controller::make_trigger_decision(25.0f, kMaxDistanceM, turret_controller::ActuatorStatuses::RELOADING),
            turret_controller::TriggerStates::TRIGGER_RELOADING);
}

TEST(TriggerDecisionTest, FarIsSkipRegardlessOfActuator)
{
  EXPECT_EQ(turret_controller::make_trigger_decision(50.0f, kMaxDistanceM, turret_controller::ActuatorStatuses::READY),
            turret_controller::TriggerStates::TRIGGER_SKIP);

  EXPECT_EQ(turret_controller::make_trigger_decision(50.0f, kMaxDistanceM, turret_controller::ActuatorStatuses::RELOADING),
            turret_controller::TriggerStates::TRIGGER_SKIP);
}

TEST(BuildTurretStatusTest, LowConfidenceIsIdleAndSkip)
{
  const auto status = turret_controller::build_turret_status(
    true, 0.50f, 20.0f, kConfidenceThreshold, kMaxDistanceM, turret_controller::ActuatorStatuses::READY);
  EXPECT_EQ(status.target_state, turret_controller::TargetStates::TARGET_LOW_CONFIDENCE);
  EXPECT_EQ(status.action, turret_controller::ActionStates::ACTION_IDLE);
  EXPECT_EQ(status.trigger_state, turret_controller::TriggerStates::TRIGGER_SKIP);
}

TEST(BuildTurretStatusTest, FarValidTargetIsLockedTrackSkip)
{
  const auto status = turret_controller::build_turret_status(
    true, 0.90f, 45.0f, kConfidenceThreshold, kMaxDistanceM, turret_controller::ActuatorStatuses::READY);
  EXPECT_EQ(status.target_state, turret_controller::TargetStates::TARGET_LOCKED);
  EXPECT_EQ(status.action, turret_controller::ActionStates::ACTION_TRACK);
  EXPECT_EQ(status.trigger_state, turret_controller::TriggerStates::TRIGGER_SKIP);
}

TEST(BuildTurretStatusTest, CloseValidReadyIsRequested)
{
  const auto status = turret_controller::build_turret_status(
    true, 0.90f, 20.0f, kConfidenceThreshold, kMaxDistanceM, turret_controller::ActuatorStatuses::READY);
  EXPECT_EQ(status.target_state, turret_controller::TargetStates::TARGET_LOCKED);
  EXPECT_EQ(status.action, turret_controller::ActionStates::ACTION_TRACK);
  EXPECT_EQ(status.trigger_state, turret_controller::TriggerStates::TRIGGER_REQUESTED);
}

TEST(BuildTurretStatusTest, CloseValidReloadingIsReloading)
{
  const auto status = turret_controller::build_turret_status(
    true, 0.90f, 20.0f, kConfidenceThreshold, kMaxDistanceM, turret_controller::ActuatorStatuses::RELOADING);
  EXPECT_EQ(status.trigger_state, turret_controller::TriggerStates::TRIGGER_RELOADING);
}

TEST(BuildTurretStatusTest, NotVisibleIsNoneIdleSkip)
{
  const auto status = turret_controller::build_turret_status(
    false, 0.0f, 0.0f, kConfidenceThreshold, kMaxDistanceM, turret_controller::ActuatorStatuses::READY);
  EXPECT_EQ(status.target_state, turret_controller::TargetStates::TARGET_NONE);
  EXPECT_EQ(status.action, turret_controller::ActionStates::ACTION_IDLE);
  EXPECT_EQ(status.trigger_state, turret_controller::TriggerStates::TRIGGER_SKIP);
}

}  // namespace