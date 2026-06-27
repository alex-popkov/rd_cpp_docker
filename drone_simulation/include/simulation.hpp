#pragma once
#include <cmath>
#include <cstring>
#include <vector>
#include <unordered_map>
#include "json.hpp"
#include "drone_link.hpp"

using json = nlohmann::json;

enum class DroneStates { Stopped, Turning, Accelerating, Moving, Decelerating };

struct DroneCommand {
  DroneStates state;
  float angleSpeed;
};

struct Coord {
  float x;
  float y;

  Coord operator+(const Coord& other) const
  {
    Coord result;
    result.x = x + other.x;
    result.y = y + other.y;
    return result;
  }

  Coord operator-(const Coord& other) const
  {
    Coord result;
    result.x = x - other.x;
    result.y = y - other.y;
    return result;
  }

  Coord operator*(float s) const
  {
    Coord result;
    result.x = x * s;
    result.y = y * s;
    return result;
  }

  Coord operator/(float s) const
  {
    Coord result;
    result.x = x / s;
    result.y = y / s;
    return result;
  }

  bool operator==(const Coord& coordToCompare) const
  {
    const float eps = 1e-6f;

    return fabs(x - coordToCompare.x) < eps && fabs(y - coordToCompare.y) < eps;
  }
};

struct AmmoParams {
  std::string name;
  float mass;
  float drag;
  float lift;
};

struct DroneConfig {
  Coord startPos;
  float altitude;
  float initialDir;
  float attackSpeed;
  float accelPath;
  std::string ammoName;
  float arrayTimeStep;
  float simTimeStep;
  float hitRadius;
  float angularSpeed;
  float turnThreshold;
  float physicsTimeStep;
  float timeScale;
};

struct DroneStateInput {
  float deltaAngle;
  float speed;
  const DroneConfig& config;
};

struct DroneContext {
  float speed = 0.0f;
  float direction;
  float acceleration;
  Coord position;
  const DroneConfig* config;
};

struct SimulationStep {
  bool hit = false;
  int target;
  float droneSpeed = 0.0f;
  float droneDirection;
  float timeSecSinceStart = 0.0f;
  Coord dropPoint;
  Coord aimPoint;
  Coord predictedTarget;
  Coord dronePosition;
  DroneStates droneState = DroneStates::Stopped;
};

struct Target {
  Coord pos;
  Coord velocity;
};
struct DroneTelemetry {
  Coord pos;
  float speed;
  float direction;
  float timeSecSinceStart;
};

struct MissionResult {
  dlink::Control control;
  bool shouldDrop;
  int bestTargetIndex;
};

float getDistanceToTarget(const Coord& target, const Coord& posiiton);

Coord getFireCoords(const Coord& targetCoord, const Coord& dronePos, const float& ratio);

Coord getInterpolatedCoords(const float& currentTime, const float& arrayTimeStep, const std::vector<Coord>& coordInTime);

Coord getPredictedTargetCoords(
  float& currentTime, float& timePeriod, const std::vector<Coord>& coordInTime, const Coord& target, float& totalTime);

float normalizeAngle(float angle);

float turnDrone(const float& deltaAngle, const float& angularSpeed, const float& simTimeStep, float droneDir);

float getRatio(const float& distanceToTarget, const float& ammoHorizontalFlightDistance);

auto updateDronePosition(const DroneContext& context) -> Coord;

float getManeuveringTime(const float& distanceToTarget,
                         const float& ammoHorizontalFlightDistance,
                         const float& attackSpeed,
                         const float& acceleration,
                         const float& angularSpeed);

Coord getBombLandCoord(const DroneTelemetry& telemetry, const float& ammoHorizontalFlightDistance);

void writeSimulationJSONFile(const std::vector<SimulationStep>& simSteps);

float getAmmoHorizontalFlightDistance(const float& attackSpeed, const float& ammoFlightTime, const AmmoParams& ammo);

json parseJSONfile(const std::string& path);

DroneConfig readDroneConfig(const std::string& path);

std::unordered_map<std::string, AmmoParams> getAmmoMap(json& ammoJSON);

AmmoParams findAmmo(const std::unordered_map<std::string, AmmoParams>& ammoMap, const std::string ammoName);

std::vector<std::vector<Coord>> fillTargets(json& targetsJSON);

float getAmmoFlightTime(const DroneConfig& droneConfig, const AmmoParams& ammo);

float length(const Coord& coord);

Coord normalize(const Coord& coord);

dlink::Control computeControl(const dlink::Telemetry& telem, const dlink::TargetPos& target);