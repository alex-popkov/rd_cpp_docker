#include <cmath>
#include "simulation.hpp"
#include "drone_controller.hpp"

DroneController::DroneController(const DroneConfig& config)
  : config(config)
{
}

auto DroneController::computeControl(const DroneTelemetry& telemetry, const Coord& aimPoint) -> dlink::Control
{
  float dx = aimPoint.x - telemetry.pos.x;
  float dy = aimPoint.y - telemetry.pos.y;
  float desiredDir = std::atan2(dy, dx);
  float deltaAngle = normalizeAngle(desiredDir - telemetry.direction);

  // швидкість обертання лінії візування (LOS rate) — щоб не відставати від рухомої цілі
  float losRate = 0.0f;
  if (this->hasPrev) {
    float dt = telemetry.timeSecSinceStart - this->prevTime;
    if (dt > 1e-3f) {
      losRate = normalizeAngle(desiredDir - this->prevDesiredDir) / dt;
    }
  }
  this->prevDesiredDir = desiredDir;
  this->prevTime = telemetry.timeSecSinceStart;
  this->hasPrev = true;

  // PD-структура з feed-forward похідної орієнтира
  const float Kp = 3.0f;
  const float Kd = 1.5f;
  float turnRate = std::clamp(Kp * deltaAngle + Kd * losRate, -1.0f, 1.0f);

  float accel;
  if (std::fabs(deltaAngle) > this->config.turnThreshold) {
    accel = 0.0f;
  }
  else if (telemetry.speed >= this->config.attackSpeed) {
    float speedError = telemetry.speed - this->config.attackSpeed;
    accel = std::clamp(-2.0f * speedError, -1.0f, 0.0f);
  }
  else {
    accel = 1.0f;
  }
  return dlink::Control{accel, turnRate};
}
