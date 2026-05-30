#pragma once
#include <cmath>
#include <cstring>
#include <vector>
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
    const std::vector<Coord>& coordInTime
);

Coord getPredictedTargetCoords (
    float& currentTime, 
    float& timePeriod, 
    const std::vector<Coord>& coordInTime, 
    const Coord& target,
    float& totalTime
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

float getManeuveringTime(
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

void writeSimulationJSONFile(const std::vector<SimulationStep>& simSteps);

float getAmmoHorizontalFlightDistance(
    const float& attackSpeed,
    const float& ammoFlightTime,
    const AmmoParams& ammo
);

json parseJSONfile(const std::string& path);

DroneConfig readDroneConfig(const std::string& path);

std::vector<AmmoParams> readAmmo(json& ammoJSON);

AmmoParams findAmmo(const std::vector<AmmoParams>& ammoArr, const std::string ammoName);

std::vector<std::vector<Coord>> fillTargets(json& targetsJSON);

float getAmmoFlightTime(
    const DroneConfig& droneConfig,
    const AmmoParams& ammo
);

float length(const Coord& coord);

Coord normalize(const Coord& coord);