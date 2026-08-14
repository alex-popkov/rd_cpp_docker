
#include <iostream>
#include <cmath>
#include <algorithm>
#include "mission_processor.hpp"
#include "log.hpp"

MissionProcessor::MissionProcessor(std::unique_ptr<IBallisticSolver> solver, const DroneConfig& config)
  : ballisticSolver(std::move(solver))
  , droneConfig(config)
{
}

auto MissionProcessor::init(const dlink::AmmoCfg& ammoCfg, float altitude) -> void
{
  this->ammo = {.name = std::string(ammoCfg.name), .mass = ammoCfg.mass, .drag = ammoCfg.drag, .lift = ammoCfg.lift};
  this->hitRadius = ammoCfg.hitRadius;

  this->droneConfig.altitude = altitude;
  this->ammoFlightTime = getAmmoFlightTime(this->droneConfig, this->ammo);
  this->ammoHorizontalFlightDistance = getAmmoHorizontalFlightDistance(this->droneConfig.attackSpeed, this->ammoFlightTime, this->ammo);

  this->initialized = true;
}

auto MissionProcessor::process(const dlink::Telemetry& telemetry,
                               const std::vector<dlink::TargetPos>& targets,
                               int targetCount) -> MissionResult
{
  if (!this->initialized || targetCount <= 0) {
    return {Coord{0, 0}, Coord{0, 0}, false, -1};
  }

  DroneTelemetry droneTelemetry = {
    .pos = {telemetry.x, telemetry.y}, .speed = telemetry.speed, .direction = telemetry.dir, .timeSecSinceStart = telemetry.t_ms / 1000.0f};
  float currentTime = telemetry.t_ms / 1000.0f;

  if (this->prevTime >= 0.0f && !this->prevTargetPositions.empty()) {
    float dt = currentTime - this->prevTime;
    if (dt > 1e-6f) {
      this->targetVelocities.resize(targetCount, {0.0f, 0.0f});
      for (int i = 0; i < targetCount && i < (int)targets.size() && i < (int)this->prevTargetPositions.size(); ++i) {
        Coord current = {targets[i].x, targets[i].y};
        this->targetVelocities[i] = (current - this->prevTargetPositions[i]) / dt;
      }
    }
  }
  else {
    this->targetVelocities.resize(targetCount, {0.0f, 0.0f});
  }

  this->prevTargetPositions.resize(targetCount);
  for (int i = 0; i < targetCount && i < (int)targets.size(); ++i) {
    this->prevTargetPositions[i] = {targets[i].x, targets[i].y};
  }
  this->prevTime = currentTime;

  int bestTargetIndex = -1;
  float bestTotalTime = INFINITY;
  Coord bestFireCoord = {0, 0};
  Coord bestAimCoord = {0, 0};

  for (int i = 0; i < targetCount && i < (int)targets.size(); ++i) {
    Coord targetPos = {targets[i].x, targets[i].y};
    Coord velocity = (i < (int)this->targetVelocities.size()) ? this->targetVelocities[i] : Coord{0, 0};
    Coord fireCoord;
    Coord aimCoord;
    float totalTime = this->evaluateTarget(i, droneTelemetry, targetPos, velocity, fireCoord, aimCoord);

    if (totalTime < bestTotalTime) {
      bestTotalTime = totalTime;
      bestTargetIndex = i;
      bestFireCoord = fireCoord;
      bestAimCoord = aimCoord;
    }
  }

  if (bestTargetIndex < 0) {
    return {Coord{0, 0}, Coord{0, 0}, false, -1};
  }

  Coord bestTargetPos = {targets[bestTargetIndex].x, targets[bestTargetIndex].y};
  Coord bestVelocity = (bestTargetIndex < (int)this->targetVelocities.size()) ? this->targetVelocities[bestTargetIndex] : Coord{0, 0};
  bool shouldDrop = this->checkDrop(droneTelemetry, bestTargetPos, bestVelocity);

  this->prevTargetIndex = bestTargetIndex;

  return {bestFireCoord, bestAimCoord, shouldDrop, bestTargetIndex};
}

auto MissionProcessor::evaluateTarget(int targetIndex,
                                      const DroneTelemetry& telemetry,
                                      const Coord& targetPos,
                                      const Coord& targetVelocity,
                                      Coord& outFireCoord,
                                      Coord& outAimCoord) -> float
{
  float currentHorizDist = getAmmoHorizontalFlightDistance(telemetry.speed, this->ammoFlightTime, this->ammo);

  float distanceToTarget = getDistanceToTarget(targetPos, telemetry.pos);
  if (distanceToTarget < 1e-3f) {
    return INFINITY;
  }

  float ratio = getRatio(distanceToTarget, currentHorizDist);
  Coord fireCoord = getFireCoords(targetPos, telemetry.pos, ratio);
  float distanceToFire = getDistanceToTarget(fireCoord, telemetry.pos);
  float droneToFireTime = distanceToFire / std::max(telemetry.speed, 1.0f);
  float totalTimeRough = droneToFireTime + this->ammoFlightTime;

  Coord predictedTarget = targetPos + targetVelocity * totalTimeRough;
  outAimCoord = predictedTarget;
  float newDistanceToTarget = getDistanceToTarget(predictedTarget, telemetry.pos);
  if (newDistanceToTarget < 1e-3f) {
    return INFINITY;
  }

  float newRatio = getRatio(newDistanceToTarget, currentHorizDist);
  outFireCoord = getFireCoords(predictedTarget, telemetry.pos, newRatio);
  float newDistanceToFire = getDistanceToTarget(outFireCoord, telemetry.pos);
  float newDroneToFireTime = newDistanceToFire / std::max(telemetry.speed, 1.0f);
  float totalTime = newDroneToFireTime + this->ammoFlightTime;
  float acceleration = this->droneConfig.attackSpeed * this->droneConfig.attackSpeed / (2.0f * this->droneConfig.accelPath);

  if (newRatio < 0) {
    totalTime += getManeuveringTime(
      newDistanceToTarget, this->ammoHorizontalFlightDistance, this->droneConfig.attackSpeed, acceleration, this->droneConfig.angularSpeed);
  }

  if (this->prevTargetIndex != targetIndex && this->prevTargetIndex != -1) {
    totalTime += telemetry.speed / acceleration;
  }

  return totalTime;
}

auto MissionProcessor::checkDrop(const DroneTelemetry& telemetry, const Coord& targetPos, const Coord& targetVelocity) -> bool
{
  float currentHorizDist = getAmmoHorizontalFlightDistance(telemetry.speed, this->ammoFlightTime, this->ammo);
  Coord bombLandCoord = getBombLandCoord(telemetry, currentHorizDist);

  Coord predictedTarget = targetPos + targetVelocity * this->ammoFlightTime;
  float bombMissDistance = getDistanceToTarget(bombLandCoord, predictedTarget);

  DEBUG("  CD: miss=" << bombMissDistance << " predTgt=(" << predictedTarget.x << "," << predictedTarget.y << ")" << " bomb=("
                      << bombLandCoord.x << "," << bombLandCoord.y << ")" << " hd=" << currentHorizDist);

  return bombMissDistance <= this->hitRadius && telemetry.speed >= this->droneConfig.attackSpeed;
}

auto MissionProcessor::reset() -> void
{
  this->prevTargetIndex = -1;
  this->prevTime = -1.0f;
  this->prevTargetPositions.clear();
  this->targetVelocities.clear();
  this->initialized = false;
}

auto MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> ballisticSolver) -> void
{
  this->ballisticSolver = std::move(ballisticSolver);
}
