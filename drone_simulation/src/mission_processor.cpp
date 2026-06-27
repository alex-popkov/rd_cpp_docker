
#include <iostream>
#include <cmath>
#include <algorithm>
#include "mission_processor.hpp"

MissionProcessor::MissionProcessor(std::unique_ptr<IBallisticSolver> solver)
  : ballisticSolver(std::move(solver))
{
}

auto MissionProcessor::init(const dlink::AmmoCfg& ammoCfg, float altitude, float attackSpd) -> void
{
  this->ammo = {.name = std::string(ammoCfg.name), .mass = ammoCfg.mass, .drag = ammoCfg.drag, .lift = ammoCfg.lift};
  this->hitRadius = ammoCfg.hitRadius;
  this->attackSpeed = attackSpd;

  float accelPath = 50.0f;
  this->acceleration = this->attackSpeed * this->attackSpeed / (2.0f * accelPath);

  DroneConfig tempConfig{};
  tempConfig.altitude = altitude;
  tempConfig.attackSpeed = this->attackSpeed;

  this->ammoFlightTime = getAmmoFlightTime(tempConfig, this->ammo);
  this->ammoHorizontalFlightDistance = getAmmoHorizontalFlightDistance(this->attackSpeed, this->ammoFlightTime, this->ammo);

  this->initialized = true;

  std::cout << "MP init: flightTime=" << ammoFlightTime << " horizDist=" << ammoHorizontalFlightDistance << std::endl;
}

auto MissionProcessor::process(const dlink::Telemetry& telem,
                               const std::vector<dlink::TargetPos>& targets,
                               int targetCount) -> MissionResult
{
  if (!this->initialized || targetCount <= 0) {
    return {dlink::Control{0.0f, 0.0f}, false, -1};
  }

  DroneTelemetry droneTelem = {
    .pos = {telem.x, telem.y}, .speed = telem.speed, .direction = telem.dir, .timeSecSinceStart = telem.t_ms / 1000.0f};

  // Обчислити швидкості цілей з різниці позицій між тактами
  float currentTime = telem.t_ms / 1000.0f;

  if (prevTime >= 0.0f && !prevTargetPositions.empty()) {
    float dt = currentTime - prevTime;
    if (dt > 1e-6f) {
      targetVelocities.resize(targetCount, {0.0f, 0.0f});
      for (int i = 0; i < targetCount && i < (int)targets.size() && i < (int)prevTargetPositions.size(); ++i) {
        Coord current = {targets[i].x, targets[i].y};
        targetVelocities[i] = (current - prevTargetPositions[i]) / dt;
      }
    }
  }
  else {
    targetVelocities.resize(targetCount, {0.0f, 0.0f});
  }

  // Зберегти поточні позиції для наступного такту
  prevTargetPositions.resize(targetCount);
  for (int i = 0; i < targetCount && i < (int)targets.size(); ++i) {
    prevTargetPositions[i] = {targets[i].x, targets[i].y};
  }
  prevTime = currentTime;

  // Знайти найкращу ціль
  int bestTargetIndex = -1;
  float bestTotalTime = INFINITY;
  Coord bestFireCoord = {0, 0};

  for (int i = 0; i < targetCount && i < (int)targets.size(); ++i) {
    Coord targetPos = {targets[i].x, targets[i].y};
    Coord velocity = (i < (int)targetVelocities.size()) ? targetVelocities[i] : Coord{0, 0};
    Coord fireCoord;
    float totalTime = evaluateTarget(i, droneTelem, targetPos, velocity, fireCoord);

    if (totalTime < bestTotalTime) {
      bestTotalTime = totalTime;
      bestTargetIndex = i;
      bestFireCoord = fireCoord;
    }
  }

  if (bestTargetIndex < 0) {
    return {dlink::Control{0.0f, 0.0f}, false, -1};
  }

  Coord bestTargetPos = {targets[bestTargetIndex].x, targets[bestTargetIndex].y};
  bool shouldDrop = checkDrop(droneTelem, bestTargetPos);

  dlink::Control control = computeControl(droneTelem, bestFireCoord);

  this->prevTargetIndex = bestTargetIndex;

  return {control, shouldDrop, bestTargetIndex};
}

auto MissionProcessor::evaluateTarget(
  int targetIndex, const DroneTelemetry& telemetry, const Coord& targetPos, const Coord& targetVelocity, Coord& outFireCoord) -> float
{
  // Прохід 1: грубий — fire point для поточної позиції цілі
  float distanceToTarget = getDistanceToTarget(targetPos, telemetry.pos);
  if (distanceToTarget < 1e-3f) {
    return INFINITY;
  }

  float ratio = getRatio(distanceToTarget, this->ammoHorizontalFlightDistance);
  Coord fireCoord = getFireCoords(targetPos, telemetry.pos, ratio);
  float distanceToFire = getDistanceToTarget(fireCoord, telemetry.pos);
  float droneToFireTime = distanceToFire / this->attackSpeed;
  float totalTimeRough = droneToFireTime + this->ammoFlightTime;

  // Прохід 2: точний — передбачити де буде ціль через totalTimeRough
  Coord predictedTarget = targetPos + targetVelocity * totalTimeRough;

  float newDistanceToTarget = getDistanceToTarget(predictedTarget, telemetry.pos);
  if (newDistanceToTarget < 1e-3f) {
    return INFINITY;
  }

  float newRatio = getRatio(newDistanceToTarget, this->ammoHorizontalFlightDistance);
  outFireCoord = getFireCoords(predictedTarget, telemetry.pos, newRatio);
  float newDistanceToFire = getDistanceToTarget(outFireCoord, telemetry.pos);
  float newDroneToFireTime = newDistanceToFire / this->attackSpeed;
  float totalTime = newDroneToFireTime + this->ammoFlightTime;

  // Штраф за маневр якщо fire point за дроном
  if (newRatio < 0) {
    totalTime += getManeuveringTime(newDistanceToTarget, this->ammoHorizontalFlightDistance, this->attackSpeed, this->acceleration, 1.0f);
  }

  // Штраф за зміну цілі
  if (this->prevTargetIndex != targetIndex && this->prevTargetIndex != -1) {
    totalTime += telemetry.speed / this->acceleration;
  }

  return totalTime;
}

auto MissionProcessor::checkDrop(const DroneTelemetry& telemetry, const Coord& targetPos) -> bool
{
  Coord bombLand = getBombLandCoord(telemetry, ammoHorizontalFlightDistance);
  float miss = getDistanceToTarget(bombLand, targetPos);

  float dx = targetPos.x - telemetry.pos.x;
  float dy = targetPos.y - telemetry.pos.y;
  float desiredDir = std::atan2(dy, dx);
  float deltaAngle = normalizeAngle(desiredDir - telemetry.direction);

  return miss <= hitRadius && telemetry.speed > 5.0f && std::fabs(deltaAngle) < 0.3f;
}

auto MissionProcessor::computeControl(const DroneTelemetry& telemetry, const Coord& firePoint) -> dlink::Control
{
  float dx = firePoint.x - telemetry.pos.x;
  float dy = firePoint.y - telemetry.pos.y;
  float desiredDir = std::atan2(dy, dx);
  float deltaAngle = normalizeAngle(desiredDir - telemetry.direction);

  float Kp = 2.0f;
  float turnRate = std::clamp(Kp * deltaAngle, -1.0f, 1.0f);

  float accel;
  if (std::fabs(deltaAngle) > 0.3f) {
    accel = 0.2f;
  }
  else {
    accel = 1.0f;
  }

  return dlink::Control{accel, turnRate};
}

auto MissionProcessor::reset() -> void
{
  prevTargetIndex = -1;
  prevTime = -1.0f;
  prevTargetPositions.clear();
  targetVelocities.clear();
  initialized = false;
}

auto MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> ballisticSolver) -> void
{
  this->ballisticSolver = std::move(ballisticSolver);
}
