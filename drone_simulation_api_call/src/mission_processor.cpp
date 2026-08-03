
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
  this->droneState = std::make_unique<StateStopped>();
}

auto MissionProcessor::init(std::unique_ptr<IConfigLoader> configLoader) -> void
{
  configLoader->load();
  this->droneConfig = configLoader->getConfig();
  this->ammo = configLoader->getAmmoParams();

  BallisticResult ballisticResult = this->ballisticSolver->solve(this->ammo);
  this->ammoFlightTime = ballisticResult.flightTime;
  this->ammoHorizontalFlightDistance = ballisticResult.hDist;
  this->acceleration = this->droneConfig.attackSpeed * this->droneConfig.attackSpeed / (2.0f * this->droneConfig.accelPath);
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
  DroneTelemetry droneTelemetry = this->dronePhysics->getTelemetry();

  for (int i = 0; i < this->targets->getTargetCount(); ++i) {
    Coord fireCoord;
    float totalTime = this->evaluateTarget(i, droneTelemetry, fireCoord);

    if (totalTime < bestTotalTime) {
      bestTotalTime = totalTime;
      bestTargetIndex = i;
      bestFireCoord = fireCoord;
    }
  }

  if (bestTargetIndex < 0) {
    ++this->currentStep;
    this->currentTime += this->droneConfig.simTimeStep;

    SimulationStep emptyStep = {.hit = false,
                                .target = -1,
                                .droneSpeed = droneTelemetry.speed,
                                .droneDirection = droneTelemetry.direction,
                                .timeSecSinceStart = droneTelemetry.timeSecSinceStart,
                                .dropPoint = {0, 0},
                                .aimPoint = {0, 0},
                                .predictedTarget = {0, 0},
                                .dronePosition = droneTelemetry.pos,
                                .droneState = DroneStates::Stopped};

    return emptyStep;
  }

  Coord deltaToFire = bestFireCoord - droneTelemetry.pos;
  Coord dirVec = normalize(deltaToFire);
  float newDirection = atan2(dirVec.y, dirVec.x);

  float deltaAngle = normalizeAngle(newDirection - droneTelemetry.direction);

  DroneStateInput stateInput = {.deltaAngle = deltaAngle, .speed = droneTelemetry.speed, .config = this->droneConfig};
  auto nextState = this->droneState->execute(stateInput);
  if (nextState) {
    this->droneState = std::move(nextState);
  }

  SimulationStep simulationStep = this->getSimulationStep(bestTargetIndex, bestFireCoord, droneTelemetry);
  simulationStep.timeSecSinceStart = droneTelemetry.timeSecSinceStart;
  simulationStep.droneState = this->droneState->name();

  this->hit = simulationStep.hit;

  float sign = (deltaAngle > 0) ? 1.0f : -1.0f;
  float angleSpeed = sign * std::min(std::fabs(deltaAngle) / this->droneConfig.simTimeStep, this->droneConfig.angularSpeed);

  DroneCommand command = {.state = this->droneState->name(), .angleSpeed = angleSpeed};
  this->dronePhysics->sendCommand(command);

  this->prevTargetIndex = bestTargetIndex;
  this->currentStep++;
  this->currentTime += this->droneConfig.simTimeStep;
  this->prevDirectionToTarget = newDirection;

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
                                   .droneState = DroneStates::Stopped});

  while (this->hasNext()) {
    SimulationStep simulationStep = this->step();
    if (simulationStep.target >= 0) {
      this->simulationSteps.push_back(simulationStep);
    }
    std::this_thread::sleep_for(std::chrono::duration<float>(this->droneConfig.simTimeStep / this->droneConfig.timeScale));
  }
}

auto MissionProcessor::evaluateTarget(int targetIndex, const DroneTelemetry& telemetry, Coord& outFireCoord) -> float
{
  Target t = this->targets->getTarget(targetIndex);
  Coord target = t.pos;

  float distanceToTarget = getDistanceToTarget(target, telemetry.pos);
  if (distanceToTarget < 1e-3f) {
    return INFINITY;
  }
  float ratio = getRatio(distanceToTarget, this->ammoHorizontalFlightDistance);

  Coord fireCoord = getFireCoords(target, telemetry.pos, ratio);
  float distanceToFire = getDistanceToTarget(fireCoord, telemetry.pos);
  float droneToFireTime = distanceToFire / this->droneConfig.attackSpeed;
  float totalTimeRough = droneToFireTime + this->ammoFlightTime;

  Coord predictedTarget = t.pos + t.velocity * totalTimeRough;

  float newDistanceToTarget = getDistanceToTarget(predictedTarget, telemetry.pos);
  if (newDistanceToTarget < 1e-3f) {
    return INFINITY;
  }

  float newRatio = getRatio(newDistanceToTarget, this->ammoHorizontalFlightDistance);
  outFireCoord = getFireCoords(predictedTarget, telemetry.pos, newRatio);
  float newDistanceToFire = getDistanceToTarget(outFireCoord, telemetry.pos);
  float newDroneToFireTime = newDistanceToFire / this->droneConfig.attackSpeed;
  float totalTime = newDroneToFireTime + this->ammoFlightTime;

  if (newRatio < 0) {
    totalTime += getManeuveringTime(newDistanceToTarget,
                                    this->ammoHorizontalFlightDistance,
                                    this->droneConfig.attackSpeed,
                                    this->acceleration,
                                    this->droneConfig.angularSpeed);
  }

  if (this->prevTargetIndex != targetIndex && this->prevTargetIndex != -1) {
    totalTime += telemetry.speed / this->acceleration;
  }

  return totalTime;
}

auto MissionProcessor::getSimulationStep(int targetIndex, const Coord& fireCoord, const DroneTelemetry& telemetry) -> SimulationStep
{
  Coord bombLandCoord = getBombLandCoord(telemetry, this->ammoHorizontalFlightDistance);

  Target target = this->targets->getTarget(targetIndex);
  Coord targetAtImpact = target.pos + target.velocity * this->ammoFlightTime;

  float bombMissDistance = getDistanceToTarget(bombLandCoord, targetAtImpact);
  Coord dir = {.x = std::cos(telemetry.direction), .y = std::sin(telemetry.direction)};
  Coord aimPoint = telemetry.pos + dir * this->ammoHorizontalFlightDistance;

  bool isHit = bombMissDistance < this->droneConfig.hitRadius || bombLandCoord == targetAtImpact;

  return {.hit = isHit,
          .target = targetIndex,
          .droneSpeed = telemetry.speed,
          .droneDirection = telemetry.direction,
          .dropPoint = fireCoord,
          .aimPoint = aimPoint,
          .predictedTarget = targetAtImpact,
          .dronePosition = telemetry.pos,
          .droneState = DroneStates::Stopped};
}

auto MissionProcessor::hasNext() -> bool
{
  return this->currentStep <= this->MAX_STEPS && !this->hit;
}