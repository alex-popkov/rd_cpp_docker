#pragma once

// Holds parameters of one ammunition type.
struct Ammo {
  float mass;
  float drag;
  float lift;
};

// 2D point in the horizontal plane (drone position or fire point).
struct Point2D {
  float x;
  float y;
};

// Result of the fire-plan computation.
// If `has_maneuver` is true, the drone must first move to `maneuver`
// before having enough room for the horizontal travel + acceleration path.
// `fire` is the point at which the ammo is released.
struct FirePlan {
  bool has_maneuver;
  Point2D maneuver;
  Point2D fire;
};

// Returns ammo parameters by its name.
// Throws std::runtime_error if the name is not in the supported list.
auto find_ammo(const char* name) -> Ammo;

// Computes how long the ammo falls until it reaches the target's altitude.
// Throws std::runtime_error if the trajectory is not physically possible
// (target too high, or computed time is not positive).
auto compute_fall_time(float zd, float attack_speed, const Ammo& ammo) -> float;

// Computes how far the ammo travels horizontally during its fall.
// Throws std::runtime_error if the computed distance is not positive.
auto compute_horizontal_travel(float t, float attack_speed, const Ammo& ammo) -> float;

// Computes the (optional) maneuver point and the fire point.
// Throws std::runtime_error if the distance to the target is not positive.
auto compute_fire_plan(Point2D drone, Point2D target, float h, float acceleration_path) -> FirePlan;