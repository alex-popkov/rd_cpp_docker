#pragma once

namespace turret_controller {

enum class ActuatorStatuses { READY, RELOADING };

enum class TargetStates { TARGET_NONE, TARGET_LOW_CONFIDENCE, TARGET_LOCKED };

enum class ServoDirections { RIGHT, LEFT, CENTER };

enum class GimbalDirections { UP, DOWN, CENTER };

enum class TriggerStates { TRIGGER_SKIP, TRIGGER_REQUESTED, TRIGGER_RELOADING };

enum class ActionStates { ACTION_IDLE, ACTION_TRACK };

struct TurretStatus {
  TargetStates target_state;
  ActionStates action;
  TriggerStates trigger_state;
  float confidence;
  float distance_m;
};

struct ServoCommand {
  float target_x;
  float error_x;
  ServoDirections direction;
};

struct GimbalCommand {
  float target_y;
  float error_y;
  GimbalDirections direction;
};

inline TargetStates evaluate_target(bool visible, float confidence, float confidence_threshold)
{
  if (!visible) {
    return TargetStates::TARGET_NONE;
  }
  if (confidence < confidence_threshold) {
    return TargetStates::TARGET_LOW_CONFIDENCE;
  }

  return TargetStates::TARGET_LOCKED;
}

inline ServoCommand make_servo_command(float target_x)
{
  ServoDirections direction = ServoDirections::CENTER;

  if (target_x > 320) {
    direction = ServoDirections::RIGHT;
  }
  else if (target_x < 320) {
    direction = ServoDirections::LEFT;
  }

  return {.target_x = target_x, .error_x = target_x - 320, .direction = direction};
}

inline GimbalCommand make_gimbal_command(float target_y)
{
  GimbalDirections direction = GimbalDirections::CENTER;

  if (target_y > 240) {
    direction = GimbalDirections::DOWN;
  }
  else if (target_y < 240) {
    direction = GimbalDirections::UP;
  }

  return {.target_y = target_y, .error_y = 240 - target_y, .direction = direction};
}

inline TriggerStates make_trigger_decision(float distance_m, float max_distance_m, ActuatorStatuses actuatorStatus)
{
  if (distance_m > max_distance_m) {
    return TriggerStates::TRIGGER_SKIP;
  }

  if (actuatorStatus == ActuatorStatuses::RELOADING) {
    return TriggerStates::TRIGGER_RELOADING;
  }

  return TriggerStates::TRIGGER_REQUESTED;
}

inline TurretStatus build_turret_status(
  bool visible, float confidence, float distance_m, float confidence_threshold, float max_distance_m, ActuatorStatuses actuator_status)
{
  TurretStatus status;
  status.confidence = confidence;
  status.distance_m = distance_m;
  status.target_state = evaluate_target(visible, confidence, confidence_threshold);

  if (status.target_state == TargetStates::TARGET_LOCKED) {
    status.action = ActionStates::ACTION_TRACK;
    status.trigger_state = make_trigger_decision(distance_m, max_distance_m, actuator_status);

    return status;
  }

  status.action = ActionStates::ACTION_IDLE;
  status.trigger_state = TriggerStates::TRIGGER_SKIP;

  return status;
}
}  // namespace turret_controller