#include <iostream>
#include <cmath>
#include <cstring>
#include <fstream>
#include "../include/json.hpp"
#include "../include/simulation.hpp"

using json = nlohmann::json;

#define ENABLE_LOG	1
#define ENABLE_DEBUG  0
 
#if ENABLE_LOG
  #define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
  #define LOG(msg)
#endif
 
#if ENABLE_DEBUG
  #define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
  #define DEBUG(msg)
#endif



float getDistanceToTarget(
    const Coord& target, const Coord& posiiton
) {
    return sqrt(pow(target.x - posiiton.x, 2) + pow(target.y - posiiton.y, 2));
}

Coord getFireCoords(
    const Coord& targetCoord, const Coord& dronePos, const float& ratio
) {
    return  dronePos + (targetCoord - dronePos) * ratio;
}

Coord getInterpolatedCoords(
    const float& currentTime, 
    const float& arrayTimeStep, 
    const std::vector<Coord>& coordInTime
) {
    int timeSteps = coordInTime.size();
    int idx = (int)floor(currentTime / arrayTimeStep) % timeSteps;
    int next = (idx + 1) % timeSteps;
    float frac = (currentTime - idx * arrayTimeStep) / arrayTimeStep;

    Coord interpolatedCoord = coordInTime[idx] + (coordInTime[next] - coordInTime[idx]) * frac;

    return interpolatedCoord;
}

Coord getPredictedTargetCoords (
    float& currentTime, 
    float& timePeriod, 
    const std::vector<Coord>& coordInTime, 
    const Coord& target,
    float& totalTime
 ) {
    Coord nextTargetCoord = getInterpolatedCoords(currentTime + timePeriod, timePeriod, coordInTime);
    Coord targetVelocity = (nextTargetCoord - target) / timePeriod;
    Coord predictedTarget = target + targetVelocity * totalTime;

    return predictedTarget;
 }

float normalizeAngle(float angle) {
    while (angle > M_PI)  {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2 * M_PI;
    }

    return angle;
}

float turnDrone(
    const float& deltaAngle,
    const float& angularSpeed,
    const float& simTimeStep,
    float droneDir
) {
    const float sign = (deltaAngle > 0) ? 1.0f : -1.0f;
    float step = angularSpeed * simTimeStep;
    if (step > fabs(deltaAngle)) {
        step = fabs(deltaAngle); 
    }
    droneDir += sign * step;
    droneDir = normalizeAngle(droneDir);

    return droneDir;
}

float getRatio(const float& distanceToTarget, const float& ammoHorizontalFlightDistance) {
    return (distanceToTarget - ammoHorizontalFlightDistance) / distanceToTarget;
}

auto updateDronePosition(const DroneContext& context) -> Coord
{
    Coord position = {
        context.position.x,
        context.position.y
    };

    position.x += context.speed * cos(context.direction) * context.config->simTimeStep;
    position.y += context.speed * sin(context.direction) * context.config->simTimeStep;

    return position;
}

float getManeuveringTime(
    const float& distanceToTarget, 
    const float& ammoHorizontalFlightDistance,
    const float& attackSpeed,
    const float& acceleration,
    const float& angularSpeed
) {
    float overshootDistance = ammoHorizontalFlightDistance - distanceToTarget;            
    float flyAwayTime = overshootDistance / attackSpeed;
    float decelerateTime = attackSpeed / acceleration;
    float turnAroundTime = M_PI / angularSpeed; 
    float accelerateTime = attackSpeed / acceleration;
    float flyBackTime = overshootDistance / attackSpeed;

    return flyAwayTime + decelerateTime + turnAroundTime + accelerateTime + flyBackTime;
}

Coord getBombLandCoord(
    const DroneContext& droneContext,
    const float& ammoHorizontalFlightDistance
) {
    const float bombLandX = droneContext.position.x + ammoHorizontalFlightDistance * cos(droneContext.direction);
    const float bombLandY = droneContext.position.y + ammoHorizontalFlightDistance * sin(droneContext.direction);
    Coord bombLandCoord = {
        .x = bombLandX,
        .y = bombLandY
    };

    return bombLandCoord;
}

void writeSimulationJSONFile(
    const std::vector<SimulationStep>& simSteps
) {
    json out;
    out["totalSteps"] = simSteps.size();
    out["steps"] = json::array();

    for (const SimulationStep& step : simSteps) {
        json stepJson;
        stepJson["position"] = {
            {"x", step.dronePosition.x},
             {"y", step.dronePosition.y}
        };
        stepJson["direction"] = step.droneDirection;
        stepJson["state"] = step.droneState;
        stepJson["targetIndex"] = step.target;
        stepJson["dropPoint"] = {
            {"x", step.dropPoint.x},
            {"y", step.dropPoint.y}
        };
        stepJson["aimPoint"] = {
            {"x", step.aimPoint.x},
            {"y", step.aimPoint.y}
        };
        stepJson["predictedTarget"] = {
            {"x", step.predictedTarget.x},
            {"y", step.predictedTarget.y}
        };
        out["steps"].push_back(stepJson);
    }
    std::ofstream fout("simulation.json");
    fout << out.dump(2);
}

float getAmmoHorizontalFlightDistance(
    const float& attackSpeed,
    const float& ammoFlightTime,
    const AmmoParams& ammo
) {
    const float g = 9.81f; 
    float ammoHorizontalFlightDistance = attackSpeed * ammoFlightTime
        - pow(ammoFlightTime, 2) * ammo.drag * attackSpeed / (2 * ammo.mass)
        + pow(ammoFlightTime, 3) * (6 * ammo.drag * g * ammo.lift * ammo.mass - 6 * pow(ammo.drag, 2) * (pow(ammo.lift, 2) - 1) * attackSpeed) / (36 * pow(ammo.mass, 2))
        + pow(ammoFlightTime, 4) * (-6 * pow(ammo.drag, 2) * g * ammo.lift * (1 + pow(ammo.lift, 2) + pow(ammo.lift, 4)) * ammo.mass + 3 * pow(ammo.drag, 3) * pow(ammo.lift, 2) * (1 + pow(ammo.lift, 2)) * attackSpeed + 6 * pow(ammo.drag, 3) * pow(ammo.lift, 4) * (1 + pow(ammo.lift, 2)) * attackSpeed) / (36 * pow(1 + pow(ammo.lift, 2), 2) * pow(ammo.mass, 3))
        + pow(ammoFlightTime, 5) * (3 * pow(ammo.drag, 3) * g * pow(ammo.lift, 3) * ammo.mass - 3 * pow(ammo.drag, 4) * pow(ammo.lift, 2) * (1 + pow(ammo.lift, 2)) * attackSpeed) / (36 * (1 + pow(ammo.lift, 2)) * pow(ammo.mass, 4));

    if (ammoHorizontalFlightDistance < 1e-6f) {
        throw std::runtime_error("Horizontal distance to the target should be more than 0\n");
    }

    return ammoHorizontalFlightDistance;
}

json parseJSONfile(const std::string& path) {
    std::ifstream configFile(path);
    if (!configFile) {
        throw std::runtime_error("Could not open " + path + " file\n ");
    }

    json configJSON; 
    configFile >> configJSON;
    configFile.close();

    return configJSON;
}

DroneConfig readDroneConfig(const std::string& path) {
    json configJSON = parseJSONfile(path);

    DroneConfig  droneConfig = {
        .startPos = {
            .x = configJSON["drone"]["position"]["x"],
            .y = configJSON["drone"]["position"]["y"]
        },
        .altitude = configJSON["drone"]["altitude"],
        .initialDir = configJSON["drone"]["initialDirection"],
        .attackSpeed = configJSON["drone"]["attackSpeed"],
        .accelPath = configJSON["drone"]["accelerationPath"],
        .ammoName = "",
        .arrayTimeStep = configJSON["targetArrayTimeStep"],
        .simTimeStep = configJSON["simulation"]["timeStep"],
        .hitRadius = configJSON["simulation"]["hitRadius"],
        .angularSpeed = configJSON["drone"]["angularSpeed"],
        .turnThreshold = configJSON["drone"]["turnThreshold"],
    };

    droneConfig.ammoName = configJSON["ammo"].get<std::string>();

    DEBUG("ammo config: " << droneConfig.ammoName);

    return droneConfig;
}

std::unordered_map<std::string, AmmoParams> getAmmoMap(json& ammoJSON) {
    int ammoCount = ammoJSON.size();
    std::unordered_map<std::string, AmmoParams> ammoMap;

    for (int i = 0; i < ammoCount; i++) {
        const std::string ammoName = ammoJSON[i]["name"].get<std::string>();
         AmmoParams ammo = {
            .name = ammoName,
            .mass = ammoJSON[i]["mass"],
            .drag = ammoJSON[i]["drag"],
            .lift = ammoJSON[i]["lift"]
        };
        ammoMap[ammoName] = ammo;
    }

    return ammoMap;
}

AmmoParams findAmmo(const std::unordered_map<std::string, AmmoParams>& ammoMap, const std::string ammoName) {
    auto it = ammoMap.find(ammoName);
    if (it == ammoMap.end()) {
        throw std::runtime_error("Unknown ammo\n");
    }
    return it->second;
}

std::vector<std::vector<Coord>> fillTargets(json& targetsJSON) {
    int targetsCount = targetsJSON["targetCount"];
    int timeSteps = targetsJSON["timeSteps"];

    std::vector<std::vector<Coord>> targets;
    for (int i = 0; i < targetsCount; i++) {
        std::vector<Coord> track; 
        for (int j = 0; j < timeSteps; j++) {
            track.push_back({
                targetsJSON["targets"][i]["positions"][j]["x"],
                targetsJSON["targets"][i]["positions"][j]["y"]
            });
        }
        targets.push_back(track);
    }

    return targets;
}


float getAmmoFlightTime(
    const DroneConfig& droneConfig,
    const AmmoParams& ammo
) {
    const float g = 9.81f; 
    float a = ammo.drag * g * ammo.mass - 2 * pow(ammo.drag, 2) * ammo.lift * droneConfig.attackSpeed;
    float b = -3 * g * pow(ammo.mass, 2) + 3 * ammo.drag * ammo.lift * ammo.mass * droneConfig.attackSpeed;
    float c = 6 * pow(ammo.mass, 2) * droneConfig.altitude;
    float p = -pow(b, 2) / (3 * pow(a, 2));
    float q = 2 * pow(b, 3) / (27 * pow(a, 3)) + c / a;
    float acosArg = 3 * q / (2 * p) * sqrt(-3 / p);

    if (acosArg > 1 || acosArg < -1) {
        throw std::runtime_error("Drone position is too high\n");
    }

    float phi = acos(acosArg);
    float ammoFlightTime = 2 * sqrt(-p / 3 ) * cos((phi + 4 * M_PI) / 3) - b / (3 * a);

    if (ammoFlightTime <= 0) {
        throw std::runtime_error("Ammo flight time should be more than 0\n");
    }

    return ammoFlightTime;
}

float length(const Coord& coord) { 
    return std::hypot(coord.x, coord.y); 
}

Coord normalize(const Coord& coord) { 
    return coord / length(coord); 
}