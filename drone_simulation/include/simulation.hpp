#pragma once
#include <cmath>
#include <cstring>
#include "json.hpp"

using json = nlohmann::json;

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
);

Coord getFireCoords(
    const Coord& targetCoord, const Coord& dronePos, const float& ratio
);

Coord getInterpolatedCoords(
    const float& currentTime, 
    const float& arrayTimeStep, 
    const Coord* coordInTime,
    const int& timeSteps
);

Coord getPredictedTargetCoords (
    float& currentTime, 
    float& timePeriod, 
    const Coord* coordInTime, 
    const Coord& target,
    float& totalTime,
    const int& timeSteps
 );

float normalizeAngle(float angle);

float turnDrone(
    const float& deltaAngle,
    const float& angularSpeed,
    const float& simTimeStep,
    float droneDir
);

float getTimeToStop(
    const float& attackSpeed,
    const float& acceleration,
    const float& prevDirToTarget,
    const float& angularSpeed,
    const DroneMotion& droneMotion
);

float getRatio(const float& distanceToTarget, const float& ammoHorizontalFlightDistance);


DroneState updateDroneState(
    const DroneState& droneState, 
    const float& deltaAngle, 
    const float& turnThreshold,
    const float& droneSpeed,
    const float& attackSpeed
);

DroneMotion updateDroneVelocity(
    const float& acceleration,
    const float& simTimeStep,
    const float& attackSpeed,
    const float& angularSpeed,
    const float& deltaAngle,
    DroneMotion droneMotion
);


DroneMotion updateDroneMotion(
    const float& acceleration,
    const float& simTimeStep,
    const float& attackSpeed,
    const float& angularSpeed,
    const float& dirToTarget,
    const float& turnThreshold,
    DroneMotion droneMotion
);

inline float getManeuveringTime(
    const float& distanceToTarget, 
    const float& ammoHorizontalFlightDistance,
    const float& attackSpeed,
    const float& acceleration,
    const float& angularSpeed
);

Coord getBombLandCoord(
    DroneMotion& droneMotion,
    const float& ammoHorizontalFlightDistance
);

void writeSimulationJSONFile(
    SimulationStep simSteps[10001],
    int currentStep
);

float getAmmoHorizontalFlightDistance(
    const float& attackSpeed,
    const float& ammoFlightTime,
    const AmmoParams& ammo,
    const float& g
);

json parseJSONfile(const std::string& path);

DroneConfig readDroneConfig(const std::string& path);

AmmoParams* readAmmo(json& ammoJSON);

AmmoParams findAmmo(const AmmoParams* ammoArr, const char ammoName[32], const int& ammoCount);

Coord** fillTargets(json& targetsJSON);

float getAmmoFlightTime(
    const DroneConfig& droneConfig,
    const AmmoParams& ammo,
    const float& g
);

void freeTargets(
    Coord**& targets,
    const int& targetsCount
);

float length(const Coord& coord);

Coord normalize(const Coord& coord);