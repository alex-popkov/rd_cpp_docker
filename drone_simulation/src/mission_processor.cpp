
#include <iostream>
#include "mission_processor.hpp"
#include "drone_states/state_stopped.hpp"

MissionProcessor::MissionProcessor(std::unique_ptr<ITargetProvider> targetProvider,
                                   std::unique_ptr<IBallisticSolver> solver,
                                   DronePhysics* physics)
  : targets(std::move(targetProvider))
  , ballisticSolver(std::move(solver))
  , dronePhysics(physics)
{
}

auto MissionProcessor::init(std::unique_ptr<IConfigLoader> configLoader) -> void
{
  configLoader->load();
  this->droneConfig = configLoader->getConfig();
  this->ammo = configLoader->getAmmoParams();

  this->ammoFlightTime = getAmmoFlightTime(this->droneConfig, this->ammo);
  this->ammoHorizontalFlightDistance = getAmmoHorizontalFlightDistance(this->droneConfig.attackSpeed, this->ammoFlightTime, this->ammo);
}

auto MissionProcessor::getCurrentStep() -> int
{
  return this->currentStep;
}

auto MissionProcessor::step() -> SimulationStep
{
  int bestTargetIndex = -1;
  float bestTotalTime = INFINITY;
  Coord bestFireCoord = {.x = 0, .y = 0};
  DroneContext droneContext = this->dronePhysics->getContext();
  DroneTelemetry droneTelemetry = this->dronePhysics->getTelemetry();

  for (int i = 0; i < this->targets->getTargetCount(); ++i) {
    Coord fireCoord;
    float totalTime = this->evaluateTarget(i, droneContext, fireCoord);

    if (totalTime < bestTotalTime) {
      bestTotalTime = totalTime;
      bestTargetIndex = i;
      bestFireCoord = fireCoord;
    }
  }

  if (bestTargetIndex < 0) {
    std::cout << "No valid target at this step, skipping" << std::endl;
    ++this->currentStep;
    this->currentTime += this->droneConfig.simTimeStep;

    SimulationStep emptyStep = {.hit = false,
                                .target = -1,
                                .droneSpeed = droneContext.speed,
                                .droneDirection = droneContext.direction,
                                .timeSecSinceStart = droneTelemetry.timeSecSinceStart,
                                .dropPoint = {0, 0},
                                .aimPoint = {0, 0},
                                .predictedTarget = {0, 0},
                                .dronePosition = droneContext.position,
                                .droneState = StateStopped::NAME};

    return emptyStep;
  }

  Coord deltaToFire = bestFireCoord - droneContext.position;
  Coord dirVec = normalize(deltaToFire);
  float newDirection = atan2(dirVec.y, dirVec.x);

  SimulationStep simulationStep = this->getSimulationStep(bestTargetIndex, bestFireCoord, droneContext);
  simulationStep.droneState = this->dronePhysics->getStateName();
  simulationStep.timeSecSinceStart = droneTelemetry.timeSecSinceStart;
  this->hit = simulationStep.hit;

  this->prevTargetIndex = bestTargetIndex;
  this->currentStep++;
  this->currentTime += this->droneConfig.simTimeStep;
  this->dronePhysics->sendCommand({.directionToTarget = newDirection, .prevDirectionToTarget = droneContext.directionToTarget});

  return simulationStep;
}

auto MissionProcessor::reset() -> void
{
  this->hit = false;
  this->prevTargetIndex = -1;
  this->currentStep = 1;
  this->currentTime = 0.0f;
}

auto MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> ballisticSolver) -> void
{
  this->ballisticSolver = std::move(ballisticSolver);
}

auto MissionProcessor::start() -> void
{
  this->running.store(true);
}

auto MissionProcessor::isThreadReady() const -> bool
{
  return this->ready.load();
}

auto MissionProcessor::getSteps() const -> const std::vector<SimulationStep>&
{
  return this->simulationSteps;
}

void MissionProcessor::run()
{
  this->ready.store(true);

  while (!this->running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  this->simulationSteps.push_back({.hit = false,
                                   .target = -1,
                                   .droneDirection = this->droneConfig.initialDir,
                                   .dropPoint = {0, 0},
                                   .aimPoint = {0, 0},
                                   .predictedTarget = {0, 0},
                                   .dronePosition = this->droneConfig.startPos,
                                   .droneState = StateStopped::NAME});

  while (this->hasNext()) {
    SimulationStep simulationStep = this->step();
    if (simulationStep.target >= 0) {
      this->simulationSteps.push_back(simulationStep);
    }
    std::this_thread::sleep_for(std::chrono::duration<float>(this->droneConfig.simTimeStep / this->droneConfig.timeScale));
  }
}

auto MissionProcessor::evaluateTarget(int targetIndex, const DroneContext& droneContext, Coord& outFireCoord) -> float
{
  Target t = this->targets->getTarget(targetIndex);
  Coord target = t.pos;

  float distanceToTarget = getDistanceToTarget(target, droneContext.position);
  if (distanceToTarget < 1e-3f) {
    std::cout << "Distance to the target " << targetIndex << " is near or less then 0\n" << std::endl;
    return INFINITY;
  }
  float ratio = getRatio(distanceToTarget, this->ammoHorizontalFlightDistance);

  Coord fireCoord = getFireCoords(target, droneContext.position, ratio);
  float distanceToFire = getDistanceToTarget(fireCoord, droneContext.position);
  float droneToFireTime = distanceToFire / this->droneConfig.attackSpeed;
  float totalTimeRough = droneToFireTime + this->ammoFlightTime;

  Coord predictedTarget = t.pos + t.velocity * totalTimeRough;

  float newDistanceToTarget = getDistanceToTarget(predictedTarget, droneContext.position);
  if (newDistanceToTarget < 1e-3f) {
    std::cout << "Predicted distance to the target " << targetIndex << " is near or less then 0\n" << std::endl;
    return INFINITY;
  }

  float newRatio = getRatio(newDistanceToTarget, this->ammoHorizontalFlightDistance);
  outFireCoord = getFireCoords(predictedTarget, droneContext.position, newRatio);
  float newDistanceToFire = getDistanceToTarget(outFireCoord, droneContext.position);
  float newDroneToFireTime = newDistanceToFire / this->droneConfig.attackSpeed;
  float totalTime = newDroneToFireTime + this->ammoFlightTime;

  if (newRatio < 0) {
    totalTime += getManeuveringTime(newDistanceToTarget,
                                    this->ammoHorizontalFlightDistance,
                                    this->droneConfig.attackSpeed,
                                    droneContext.acceleration,
                                    this->droneConfig.angularSpeed);
  }

  if (this->prevTargetIndex != targetIndex && this->prevTargetIndex != -1) {
    totalTime += droneContext.timeToStop;
  }

  return totalTime;
}

auto MissionProcessor::getSimulationStep(int targetIndex, const Coord& fireCoord, const DroneContext& droneContext) -> SimulationStep
{
  Coord bombLandCoord = getBombLandCoord(droneContext, this->ammoHorizontalFlightDistance);

  Target target = this->targets->getTarget(targetIndex);
  Coord targetAtImpact = target.pos + target.velocity * this->ammoFlightTime;

  float bombMissDistance = getDistanceToTarget(bombLandCoord, targetAtImpact);
  Coord dir = {.x = std::cos(droneContext.direction), .y = std::sin(droneContext.direction)};
  Coord aimPoint = droneContext.position + dir * this->ammoHorizontalFlightDistance;

  bool isHit = bombMissDistance < this->droneConfig.hitRadius || bombLandCoord == targetAtImpact;

  if (isHit) {
    std::cout << (bombLandCoord == targetAtImpact ? "Direct hit" : "Drone hit the target. Bomb miss: ")
              << (bombLandCoord == targetAtImpact ? "" : std::to_string(bombMissDistance)) << std::endl;
  }

  return {.hit = isHit,
          .target = targetIndex,
          .droneSpeed = droneContext.speed,
          .droneDirection = droneContext.direction,
          .dropPoint = fireCoord,
          .aimPoint = aimPoint,
          .predictedTarget = targetAtImpact,
          .dronePosition = droneContext.position,
          .droneState = ""};
}

auto MissionProcessor::hasNext() -> bool
{
  return this->currentStep <= this->MAX_STEPS && !this->hit;
}