
#include <iostream>
#include "mission_processor.hpp"
#include "drone_states/state_stopped.hpp"

MissionProcessor::MissionProcessor(std::unique_ptr<ITargetProvider> targetProvider, std::unique_ptr<IBallisticSolver> solver)
  : targets(std::move(targetProvider))
  , ballisticSolver(std::move(solver))
{
}

auto MissionProcessor::init(std::unique_ptr<IConfigLoader> configLoader) -> void
{
  configLoader->load();
  this->droneConfig = configLoader->getConfig();
  this->ammo = configLoader->getAmmoParams();

  this->ammoFlightTime = getAmmoFlightTime(this->droneConfig, this->ammo);
  this->ammoHorizontalFlightDistance = getAmmoHorizontalFlightDistance(this->droneConfig.attackSpeed, this->ammoFlightTime, this->ammo);
  this->droneContext = {
    .directionToTarget = 0.0f,
    .prevDirectionToTarget = 0.0f,
    .speed = 0.0f,
    .direction = this->droneConfig.initialDir,
    .acceleration = this->droneConfig.attackSpeed * this->droneConfig.attackSpeed / (2.0f * this->droneConfig.accelPath),
    .timeToStop = 0.0f,
    .position = this->droneConfig.startPos,
    .config = &this->droneConfig};
  this->state = std::make_unique<StateStopped>();
}

auto MissionProcessor::hasNext() -> bool
{
  return this->currentStep <= this->MAX_STEPS && !this->hit;
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

  for (int i = 0; i < this->targets->getTargetCount(); ++i) {
    Coord fireCoord;
    float totalTime = this->evaluateTarget(i, this->currentTime, fireCoord);

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
                                .droneSpeed = this->droneContext.speed,
                                .droneDirection = this->droneContext.direction,
                                .dropPoint = {0, 0},
                                .aimPoint = {0, 0},
                                .predictedTarget = {0, 0},
                                .dronePosition = this->droneContext.position,
                                .droneState = StateStopped::NAME};

    return emptyStep;
  }

  Coord deltaToFire = bestFireCoord - this->droneContext.position;
  Coord dirVec = normalize(deltaToFire);
  this->droneContext.prevDirectionToTarget = this->droneContext.directionToTarget;
  this->droneContext.directionToTarget = atan2(dirVec.y, dirVec.x);

  auto nextState = this->state->execute(this->droneContext);
  if (nextState)
    this->state = std::move(nextState);

  SimulationStep simulationStep = this->getSimulationStep(bestTargetIndex, bestFireCoord);
  this->hit = simulationStep.hit;

  this->prevTargetIndex = bestTargetIndex;
  this->currentStep++;
  this->currentTime += this->droneConfig.simTimeStep;

  return simulationStep;
}

auto MissionProcessor::reset() -> void
{
  this->hit = false;
  this->prevTargetIndex = -1;
  this->currentStep = 1;
  this->currentTime = 0.0f;

  this->droneContext = {.directionToTarget = 0.0f,
                        .prevDirectionToTarget = this->droneConfig.initialDir,
                        .speed = 0.0f,
                        .direction = this->droneConfig.initialDir,
                        .acceleration = this->droneContext.acceleration,
                        .timeToStop = 0.0f,
                        .position = this->droneConfig.startPos,
                        .config = &this->droneConfig};

  this->state = std::make_unique<StateStopped>();
}

auto MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> ballisticSolver) -> void
{
  this->ballisticSolver = std::move(ballisticSolver);
}

auto MissionProcessor::evaluateTarget(int targetIndex, float currentTime, Coord& outFireCoord) -> float
{
  Coord target = getInterpolatedCoords(currentTime, this->droneConfig.arrayTimeStep, this->targets->getTarget(targetIndex));

  float distanceToTarget = getDistanceToTarget(target, this->droneContext.position);
  if (distanceToTarget < 1e-3f) {
    std::cout << "Distance to the target " << targetIndex << " is near or less then 0\n" << std::endl;
    return INFINITY;
  }
  float ratio = getRatio(distanceToTarget, this->ammoHorizontalFlightDistance);

  Coord fireCoord = getFireCoords(target, this->droneContext.position, ratio);
  float distanceToFire = getDistanceToTarget(fireCoord, this->droneContext.position);
  float droneToFireTime = distanceToFire / this->droneConfig.attackSpeed;
  float totalTimeRough = droneToFireTime + this->ammoFlightTime;

  Coord predictedTarget =
    getPredictedTargetCoords(currentTime, this->droneConfig.arrayTimeStep, this->targets->getTarget(targetIndex), target, totalTimeRough);

  float newDistanceToTarget = getDistanceToTarget(predictedTarget, this->droneContext.position);
  if (newDistanceToTarget < 1e-3f) {
    std::cout << "Predicted distance to the target " << targetIndex << " is near or less then 0\n" << std::endl;
    return INFINITY;
  }

  float newRatio = getRatio(newDistanceToTarget, this->ammoHorizontalFlightDistance);
  outFireCoord = getFireCoords(predictedTarget, this->droneContext.position, newRatio);
  float newDistanceToFire = getDistanceToTarget(outFireCoord, this->droneContext.position);
  float newDroneToFireTime = newDistanceToFire / this->droneConfig.attackSpeed;
  float totalTime = newDroneToFireTime + this->ammoFlightTime;

  if (newRatio < 0) {
    totalTime += getManeuveringTime(newDistanceToTarget,
                                    this->ammoHorizontalFlightDistance,
                                    this->droneConfig.attackSpeed,
                                    this->droneContext.acceleration,
                                    this->droneConfig.angularSpeed);
  }

  if (this->prevTargetIndex != targetIndex && this->prevTargetIndex != -1) {
    totalTime += this->droneContext.timeToStop;
  }

  return totalTime;
}

auto MissionProcessor::getSimulationStep(int targetIndex, const Coord& fireCoord) -> SimulationStep
{
  Coord bombLandCoord = getBombLandCoord(this->droneContext, this->ammoHorizontalFlightDistance);
  Coord targetAtImpact =
    getInterpolatedCoords(this->currentTime + this->ammoFlightTime, this->droneConfig.arrayTimeStep, this->targets->getTarget(targetIndex));

  float bombMissDistance = getDistanceToTarget(bombLandCoord, targetAtImpact);
  Coord dir = {.x = std::cos(this->droneContext.direction), .y = std::sin(this->droneContext.direction)};
  Coord aimPoint = this->droneContext.position + dir * this->ammoHorizontalFlightDistance;

  bool isHit = bombMissDistance < this->droneConfig.hitRadius || bombLandCoord == targetAtImpact;

  if (isHit) {
    std::cout << (bombLandCoord == targetAtImpact ? "Direct hit" : "Drone hit the target. Bomb miss: ")
              << (bombLandCoord == targetAtImpact ? "" : std::to_string(bombMissDistance)) << std::endl;
  }

  return {.hit = isHit,
          .target = targetIndex,
          .droneSpeed = this->droneContext.speed,
          .droneDirection = this->droneContext.direction,
          .dropPoint = fireCoord,
          .aimPoint = aimPoint,
          .predictedTarget = targetAtImpact,
          .dronePosition = this->droneContext.position,
          .droneState = this->state->name()};
}