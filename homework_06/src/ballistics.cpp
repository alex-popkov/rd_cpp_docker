#include "ballistics.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

Ammo find_ammo(const char* name)
{
  if (strcmp(name, "VOG-17") == 0) {
    return Ammo{0.35f, 0.07f, 0.0f};
  }
  if (strcmp(name, "M67") == 0) {
    return Ammo{0.6f, 0.1f, 0.0f};
  }
  if (strcmp(name, "RKG-3") == 0) {
    return Ammo{1.2f, 0.1f, 0.0f};
  }
  if (strcmp(name, "GLIDING-VOG") == 0) {
    return Ammo{0.45f, 0.1f, 1.0f};
  }
  if (strcmp(name, "GLIDING-RKG") == 0) {
    return Ammo{1.4f, 0.1f, 1.0f};
  }

  throw std::runtime_error(std::string("balistics.cpp: find_ammo: unknown ammo name: ") + name);
}

float compute_fall_time(float zd, float attack_speed, const Ammo& ammo)
{
  const float g = 9.81f;
  const float m = ammo.mass;
  const float d = ammo.drag;
  const float l = ammo.lift;

  const float a = d * g * m - 2 * pow(d, 2) * l * attack_speed;
  const float b = -3 * g * pow(m, 2) + 3 * d * l * m * attack_speed;
  const float c = 6 * pow(m, 2) * zd;
  const float p = -pow(b, 2) / (3 * pow(a, 2));
  const float q = 2 * pow(b, 3) / (27 * pow(a, 3)) + c / a;
  const float acos_arg = 3 * q / (2 * p) * sqrt(-3 / p);

  if (acos_arg > 1 || acos_arg < -1) {
    throw std::runtime_error("balistics.cpp: compute_fall_time: target is too high");
  }

  const float phi = acos(acos_arg);
  const float t = 2 * sqrt(-p / 3) * cos((phi + 4 * M_PI) / 3) - b / (3 * a);

  if (t <= 0) {
    throw std::runtime_error("balistics.cpp: compute_fall_time: time must be greater than 0");
  }

  return t;
}

float compute_horizontal_travel(float t, float attack_speed, const Ammo& ammo)
{
  const float g = 9.81f;
  const float m = ammo.mass;
  const float d = ammo.drag;
  const float l = ammo.lift;

  const float h = attack_speed * t - pow(t, 2) * d * attack_speed / (2 * m) +
                  pow(t, 3) * (6 * d * g * l * m - 6 * pow(d, 2) * (pow(l, 2) - 1) * attack_speed) / (36 * pow(m, 2)) +
                  pow(t, 4) *
                    (-6 * pow(d, 2) * g * l * (1 + pow(l, 2) + pow(l, 4)) * m + 3 * pow(d, 3) * pow(l, 2) * (1 + pow(l, 2)) * attack_speed +
                     6 * pow(d, 3) * pow(l, 4) * (1 + pow(l, 2)) * attack_speed) /
                    (36 * pow(1 + pow(l, 2), 2) * pow(m, 3)) +
                  pow(t, 5) * (3 * pow(d, 3) * g * pow(l, 3) * m - 3 * pow(d, 4) * pow(l, 2) * (1 + pow(l, 2)) * attack_speed) /
                    (36 * (1 + pow(l, 2)) * pow(m, 4));

  if (h <= 0) {
    throw std::runtime_error("balistics.cpp: compute_horizontal_travel: horizontal distance must be greater than 0");
  }

  return h;
}

FirePlan compute_fire_plan(Point2D drone, Point2D target, float h, float acceleration_path)
{
  float distance_to_target = sqrt(pow(target.x - drone.x, 2) + pow(target.y - drone.y, 2));

  if (distance_to_target <= 0) {
    throw std::runtime_error("balistics.cpp: compute_fire_plan: distance to target must be greater than 0");
  }

  FirePlan plan;
  plan.has_maneuver = false;
  plan.maneuver = Point2D{0.0f, 0.0f};

  if (h + acceleration_path > distance_to_target) {
    drone.x = target.x - (target.x - drone.x) * (h + acceleration_path) / distance_to_target;
    drone.y = target.y - (target.y - drone.y) * (h + acceleration_path) / distance_to_target;
    distance_to_target = sqrt(pow(target.x - drone.x, 2) + pow(target.y - drone.y, 2));

    plan.has_maneuver = true;
    plan.maneuver = Point2D{drone.x, drone.y};
  }

  const float ratio = (distance_to_target - h) / distance_to_target;
  plan.fire.x = drone.x + (target.x - drone.x) * ratio;
  plan.fire.y = drone.y + (target.y - drone.y) * ratio;

  return plan;
}