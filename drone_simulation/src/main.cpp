#include <iostream>
#include <cmath>
#include <cstring>
#include <fstream>
#include "json.hpp"
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

enum DroneState {
    STOPPED,
    ACCELERATING,
    DECELERATING,
    TURNING,
    MOVING
};

struct Coord {
	float x;
	float y;
 
	Coord operator+(const Coord& other) const {
    	Coord result;
        result.x = x + other.x;
        result.y = y + other.y;
        return result;
	}
 
	Coord operator-(const Coord& other) const {
    	Coord result;
        result.x = x - other.x;
        result.y = y - other.y;
        return result;
	}
 
	Coord operator*(float s) const {
    	Coord result;
        result.x = x * s;
        result.y = y * s;
        return result;
	}

	Coord operator/(float s) const {
    	Coord result;
        result.x = x / s;
        result.y = y / s;
        return result;
	}

    bool operator==(const Coord& coordToCompare) const {
        const float eps = 1e-6f;
        
        return fabs(x - coordToCompare.x) < eps && fabs(y - coordToCompare.y) < eps;
	}

};

struct AmmoParams {
	char name[32];
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
	char  ammoName[32];
	float arrayTimeStep;
	float simTimeStep;
	float hitRadius; 
	float angularSpeed;
	float turnThreshold;
};

struct DroneMotion {
    float speed = 0.0f;
    float dir;
    Coord pos; 
    DroneState state = STOPPED;
};
struct SimulationStep {
    int target;
    Coord dropPoint;
	Coord aimPoint;
	Coord predictedTarget;
    DroneMotion droneMotion;
};

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
    const Coord* coordInTime,
    const int& timeSteps
) {
    int idx = (int)floor(currentTime / arrayTimeStep) % timeSteps;
    int next = (idx + 1) % timeSteps;
    float frac = (currentTime - idx * arrayTimeStep) / arrayTimeStep;

    Coord interpolatedCoord = coordInTime[idx] + (coordInTime[next] - coordInTime[idx]) * frac;

    return interpolatedCoord;
}

Coord getPredictedTargetCoords (
    float& currentTime, 
    float& timePeriod, 
    const Coord* coordInTime, 
    const Coord& target,
    float& totalTime,
    const int& timeSteps
 ) {
    Coord nextTargetCoord = getInterpolatedCoords(currentTime + timePeriod, timePeriod, coordInTime, timeSteps);
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

float getTimeToStop(
    const float& attackSpeed,
    const float& acceleration,
    const float& prevDirToTarget,
    const float& angularSpeed,
    const DroneMotion& droneMotion
) {
    switch (droneMotion.state) {
        case STOPPED:
            return 0; 
            break;

        case ACCELERATING: 
            return  droneMotion.speed / acceleration; 
            break;

        case MOVING:       
            return attackSpeed / acceleration;
            break;

        case TURNING:   
            return fabs(normalizeAngle(prevDirToTarget - droneMotion.dir)) / angularSpeed;
            break;

        case DECELERATING: 
            return droneMotion.speed / acceleration;
            break;

        default:
            return 0;
    }
}

float getRatio(const float& distanceToTarget, const float& ammoHorizontalFlightDistance) {
    return (distanceToTarget - ammoHorizontalFlightDistance) / distanceToTarget;
}

DroneState updateDroneState(
    const DroneState& droneState, 
    const float& deltaAngle, 
    const float& turnThreshold,
    const float& droneSpeed,
    const float& attackSpeed
) {
    const bool deltaAngleOkToMove = fabs(deltaAngle) <= turnThreshold;
    const bool deltaAngleNeedTurn = fabs(deltaAngle) > turnThreshold;

    if (droneState == MOVING && deltaAngleNeedTurn) {

        return DECELERATING;
    } else if (droneState == DECELERATING && droneSpeed < 1e-6f) {

        return STOPPED;
    } else if (droneState == STOPPED && deltaAngleNeedTurn) {

        return TURNING;
    } else if (deltaAngleOkToMove && (droneState == TURNING 
        || droneState == STOPPED
        || droneState == DECELERATING)) {

        return ACCELERATING;
    } else if (droneState == ACCELERATING && droneSpeed >= attackSpeed) {

        return MOVING;
    } 

    return droneState;
}

DroneMotion updateDroneVelocity(
    const float& acceleration,
    const float& simTimeStep,
    const float& attackSpeed,
    const float& angularSpeed,
    const float& deltaAngle,
    DroneMotion droneMotion
) {
    switch (droneMotion.state) {
            case STOPPED:
                droneMotion.speed = 0;
                break;

            case ACCELERATING: 
                droneMotion.speed += acceleration * simTimeStep;
                if (droneMotion.speed > attackSpeed) {
                    droneMotion.speed = attackSpeed;
                }

                if (fabs(deltaAngle) > 1e-6f) {
                    droneMotion.dir = turnDrone(deltaAngle, angularSpeed, simTimeStep, droneMotion.dir); 
                }

                break;

            case MOVING:
                droneMotion.speed = attackSpeed;
                if (fabs(deltaAngle) > 1e-6f) {
                    droneMotion.dir = turnDrone(deltaAngle, angularSpeed, simTimeStep, droneMotion.dir);
                }
                break;

            case DECELERATING:
                droneMotion.speed -= acceleration * simTimeStep;
                if (droneMotion.speed <= 1e-6f) {
                    droneMotion.speed = 0;
                }
                break;

            case TURNING:
                {
                    droneMotion.speed = 0;
                    droneMotion.dir = turnDrone(
                        deltaAngle, angularSpeed, simTimeStep, droneMotion.dir
                    );
                }
                break;
        }

        return droneMotion;
}

DroneMotion updateDroneMotion(
    const float& acceleration,
    const float& simTimeStep,
    const float& attackSpeed,
    const float& angularSpeed,
    const float& dirToTarget,
    const float& turnThreshold,
    DroneMotion droneMotion
) {
    float deltaAngle = normalizeAngle(dirToTarget - droneMotion.dir);

    droneMotion.state = updateDroneState(
        droneMotion.state, 
        deltaAngle, 
        turnThreshold, 
        droneMotion.speed, 
        attackSpeed
    );
    droneMotion = updateDroneVelocity(
        acceleration,
        simTimeStep,
        attackSpeed,
        angularSpeed,
        deltaAngle,
        droneMotion
    );
    droneMotion.pos.x += droneMotion.speed * cos(droneMotion.dir) * simTimeStep;
    droneMotion.pos.y += droneMotion.speed * sin(droneMotion.dir) * simTimeStep;

    return droneMotion;
}

inline float getManeuveringTime(
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
    DroneMotion& droneMotion,
    const float& ammoHorizontalFlightDistance
) {
    const float bombLandX = droneMotion.pos.x + ammoHorizontalFlightDistance * cos(droneMotion.dir);
    const float bombLandY = droneMotion.pos.y + ammoHorizontalFlightDistance * sin(droneMotion.dir);
    Coord bombLandCoord = {
        .x = bombLandX,
        .y = bombLandY
    };

    return bombLandCoord;
}

void writeSimulationJSONFile(
    SimulationStep simSteps[10001],
    int currentStep
) {
    json out;
    int totalRecords = currentStep + 1;
    out["totalSteps"] = totalRecords;
    out["steps"] = json::array();

    for (int i = 0; i < totalRecords; i++) {
        json step;
        step["position"] = {
            {"x", simSteps[i].droneMotion.pos.x},
             {"y", simSteps[i].droneMotion.pos.y}
        };
        step["direction"] = simSteps[i].droneMotion.dir;
        step["state"] = simSteps[i].droneMotion.state;
        step["targetIndex"] = simSteps[i].target;
        step["dropPoint"] = {
            {"x", simSteps[i].dropPoint.x},
            {"y", simSteps[i].dropPoint.y}
        };
        step["aimPoint"] = {
            {"x", simSteps[i].aimPoint.x},
            {"y", simSteps[i].aimPoint.y}
        };
        step["predictedTarget"] = {
            {"x", simSteps[i].predictedTarget.x},
            {"y", simSteps[i].predictedTarget.y}
        };
        out["steps"].push_back(step);
    }
    std::ofstream fout("simulation.json");
    fout << out.dump(2);
}

inline float getAmmoHorizontalFlightDistance(
    const float& attackSpeed,
    const float& ammoFlightTime,
    const AmmoParams& ammo,
    const float& g
) {
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

    std::string tmp = configJSON["ammo"].get<std::string>();
    const char* cstr = tmp.c_str();
    std::strncpy(droneConfig.ammoName, cstr, 31);

    DEBUG("ammo config: " << droneConfig.ammoName);

    return droneConfig;
}

AmmoParams* readAmmo(json& ammoJSON) {
    int ammoCount = ammoJSON.size();
    AmmoParams* ammoArr = new AmmoParams[ammoCount];
    for (int i = 0; i < ammoCount; i++) {
        std::strncpy(ammoArr[i].name, ammoJSON[i]["name"].get<std::string>().c_str(), 31);
        ammoArr[i].mass = ammoJSON[i]["mass"];
        ammoArr[i].drag = ammoJSON[i]["drag"];
        ammoArr[i].lift = ammoJSON[i]["lift"];
    }

    return ammoArr;
}

AmmoParams findAmmo(const AmmoParams* ammoArr, const char ammoName[32], const int& ammoCount) {
    bool ammoFound = false;
    AmmoParams ammo;
    for (int i = 0; i < ammoCount; ++i) {
        if (strcmp(ammoName, ammoArr[i].name) == 0) {
            ammo = ammoArr[i];
            ammoFound = true;
            break;
        }
    }

    if (!ammoFound) {
        throw std::runtime_error("Unknown ammo\n ");
    }

    return ammo;
}

Coord** fillTargets(json& targetsJSON) {
    int targetsCount = targetsJSON["targetCount"];
    int timeSteps = targetsJSON["timeSteps"];

    Coord** targets = new Coord*[targetsCount];
    for (int i = 0; i < targetsCount; i++) {
        targets[i] = new Coord[timeSteps];
        for (int j = 0; j < timeSteps; j++) {
            targets[i][j].x = targetsJSON["targets"][i]["positions"][j]["x"];
            targets[i][j].y = targetsJSON["targets"][i]["positions"][j]["y"];
        }
    }

    return targets;
}


float getAmmoFlightTime(
    const DroneConfig& droneConfig,
    const AmmoParams& ammo,
    const float& g
) {
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

void freeTargets(
    Coord**& targets,
    const int& targetsCount
) {
    if (!targets) {
        return;
    }

    for (int i = 0; i < targetsCount; i++) {
        delete[] targets[i];   
        targets[i] = nullptr;  
    }  
    delete[] targets;
    targets = nullptr;
}

float length(const Coord& coord) { 
    return std::hypot(coord.x, coord.y); 
}

Coord normalize(const Coord& coord) { 
    return coord / length(coord); 
}

int main() {
    const float g = 9.81f;

    DroneConfig droneConfig;
    json ammoJSON;
    AmmoParams* ammoArr = nullptr;
    AmmoParams ammo;
    json targetsJSON;
    Coord** targets = nullptr;
    float ammoFlightTime;
    float ammoHorizontalFlightDistance;
    int targetsCount;
    int ammoCount;
    int timeSteps;

    try {
        droneConfig = readDroneConfig("config.json");
        ammoJSON = parseJSONfile("ammo.json");
        ammoCount = ammoJSON.size();
        ammoArr = readAmmo(ammoJSON);
        ammo = findAmmo(ammoArr, droneConfig.ammoName, ammoCount);
        delete[] ammoArr;
        ammoArr = nullptr;
        targetsJSON = parseJSONfile("targets.json");
        targetsCount = targetsJSON["targetCount"];
        timeSteps = targetsJSON["timeSteps"];
        targets = fillTargets(targetsJSON);
        ammoFlightTime = getAmmoFlightTime(droneConfig, ammo, g);
        ammoHorizontalFlightDistance =  getAmmoHorizontalFlightDistance(
            droneConfig.attackSpeed,
            ammoFlightTime,
            ammo,
            g
        );
    } catch (const std::exception& error) {
        LOG("Error: " << error.what());
        freeTargets(targets, targetsCount);

        return 1;
    }

    //simulation start
    const int MAX_STEPS = 10000;
    const float acceleration = droneConfig.attackSpeed * droneConfig.attackSpeed / (2.0f * droneConfig.accelPath);
    int currentStep = 1;
    float currentTime = 0.0f;
    SimulationStep* simSteps = new SimulationStep[MAX_STEPS + 1];
    DroneMotion droneMotion = {
        .speed = 0.0f,
        .dir = droneConfig.initialDir,
        .pos = {
            .x = droneConfig.startPos.x,
            .y = droneConfig.startPos.y
        },
        .state = STOPPED
    };
    int prevTargetIndex = -1;
    float prevDirToTarget = droneConfig.initialDir;

    //write initial sim data
    simSteps[0] = {
        .target = -1,
        .droneMotion = {
            .dir = droneConfig.initialDir,
            .pos = droneConfig.startPos,
            .state = STOPPED
        }
    };

    //simulation loop
    while(true) {
        if (currentStep > MAX_STEPS) {
            LOG("The maximum number of steps has been reached\n");
            freeTargets(targets, targetsCount); 

            return 1;
        }

        int bestTargetIndex = -1;
        float bestTotalTime = INFINITY;
        Coord bestFireCoord = {
            .x = 0,
            .y = 0
        };

        //iteration by targets
        for (int i = 0; i < targetsCount; ++i) {
            Coord target = getInterpolatedCoords(currentTime, droneConfig.arrayTimeStep, targets[i], timeSteps);

            // balistic
            float distanceToTarget = getDistanceToTarget(target, droneMotion.pos);
            if (distanceToTarget < 1e-6f) {
                LOG("Distance to the target " << i << " is near or less then 0\n");
                continue;
            }
            float ratio = getRatio(distanceToTarget, ammoHorizontalFlightDistance);

            Coord fireCoord = getFireCoords(target, droneMotion.pos, ratio);
            float distanceToFire = getDistanceToTarget(fireCoord, droneMotion.pos);
            float droneToFireTime = distanceToFire / droneConfig.attackSpeed;
            float totalTimeRough = droneToFireTime + ammoFlightTime;

            Coord predictedTarget = getPredictedTargetCoords(
                currentTime, droneConfig.arrayTimeStep, targets[i], target, totalTimeRough, timeSteps
            );

            //predicted balistic
            float newDistanceToTarget = getDistanceToTarget(predictedTarget, droneMotion.pos);
            if (newDistanceToTarget < 1e-6f) {
                LOG("Predicted distance to the target " << i << " is near or less then 0\n");
                continue;
            }

            float newRatio = getRatio(newDistanceToTarget, ammoHorizontalFlightDistance);
            Coord newFireCoord = getFireCoords(predictedTarget, droneMotion.pos, newRatio);
            float newDistanceToFire = getDistanceToTarget(newFireCoord, droneMotion.pos);
            float newDroneToFireTime = newDistanceToFire / droneConfig.attackSpeed;
            float newTotalTime = newDroneToFireTime + ammoFlightTime;
          
            if (newRatio < 0) {
                newTotalTime += getManeuveringTime(
                    newDistanceToTarget, 
                    ammoHorizontalFlightDistance,
                    droneConfig.attackSpeed,
                    acceleration,
                    droneConfig.angularSpeed
                );
            }

            if (prevTargetIndex != i && prevTargetIndex != -1) {
                newTotalTime += getTimeToStop(
                    droneConfig.attackSpeed, 
                    acceleration,  
                    prevDirToTarget, 
                    droneConfig.angularSpeed,
                    droneMotion
                );
            }

            if (newTotalTime < bestTotalTime) {
                bestTotalTime = newTotalTime;
                bestTargetIndex = i;
                bestFireCoord = newFireCoord;
            }
        }


        if (bestTargetIndex < 0) {
            LOG("No valid target at this step, skipping");
            ++currentStep;
            currentTime += droneConfig.simTimeStep;

            continue;
        }


        Coord deltaToFire = bestFireCoord - droneMotion.pos;
        Coord dirVec = normalize(deltaToFire);
        float dirToTarget = atan2(dirVec.y, dirVec.x);

        droneMotion = updateDroneMotion(
            acceleration,
            droneConfig.simTimeStep,
            droneConfig.attackSpeed,
            droneConfig.angularSpeed,
            dirToTarget,
            droneConfig.turnThreshold,
            droneMotion
        );

        Coord bombLandCoord = getBombLandCoord(droneMotion, ammoHorizontalFlightDistance);
        Coord targetAtImpact = getInterpolatedCoords(
            currentTime + ammoFlightTime, 
            droneConfig.arrayTimeStep,
            targets[bestTargetIndex],
            timeSteps
        );

        float bombMissDistance = getDistanceToTarget(bombLandCoord, targetAtImpact);
        Coord dir = { 
            .x = std::cos(droneMotion.dir), 
            .y = std::sin(droneMotion.dir) 
        };
    
        Coord aimPoint = droneMotion.pos + dir * ammoHorizontalFlightDistance;

        //push value to simArray
        simSteps[currentStep] = {
            .target = bestTargetIndex,
            .dropPoint = bestFireCoord,
            .aimPoint = aimPoint,
            .predictedTarget = targetAtImpact,
            .droneMotion = droneMotion
        };

        if (bombMissDistance < droneConfig.hitRadius) {
            LOG("Drone hit the target. Bomb miss: " << bombMissDistance);
        
            break;
        }

        if (bombLandCoord == targetAtImpact) {
            LOG("Direct hit");

            break;
        }

        prevTargetIndex = bestTargetIndex;
        prevDirToTarget = dirToTarget; 
        ++currentStep;
        currentTime += droneConfig.simTimeStep;
    }

    freeTargets(targets, targetsCount);
    
     writeSimulationJSONFile( 
        simSteps,
        currentStep
     );

     delete[] simSteps;

    return 0;
}

