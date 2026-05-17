#include "ballistics.hpp"

#include <cmath>
#include <cstring>
#include <numbers>
#include <stdexcept>
#include <string>

auto find_ammo(const char* name) -> Ammo
{
  if (strcmp(name, "VOG-17") == 0) {
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers) — it's an ammo configuration
    return Ammo{0.35f, 0.07f, 0.0f};
  }
  if (strcmp(name, "M67") == 0) {
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers) — it's an ammo configuration
    return Ammo{0.6f, 0.1f, 0.0f};
  }
  if (strcmp(name, "RKG-3") == 0) {
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers) — it's an ammo configuration
    return Ammo{1.2f, 0.1f, 0.0f};
  }
  if (strcmp(name, "GLIDING-VOG") == 0) {
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers) — it's an ammo configuration
    return Ammo{0.45f, 0.1f, 1.0f};
  }
  if (strcmp(name, "GLIDING-RKG") == 0) {
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers) — it's an ammo configuration
    return Ammo{1.4f, 0.1f, 1.0f};
  }

  throw std::runtime_error(std::string("balistics.cpp: find_ammo: unknown ammo name: ") + name);
}

auto compute_fall_time(float zd, float attack_speed, const Ammo& ammo) -> float
{
  const float g = 9.81f;
  const float m = ammo.mass;
  const float d = ammo.drag;
  const float l = ammo.lift;

  const float a = d * g * m - 2 * (d * d) * l * attack_speed;
  const float b = -3 * g * (m * m) + 3 * d * l * m * attack_speed;
  const float c = 6 * (m * m) * zd;
  const float p = -(b * b) / (3 * (a * a));
  const float q = 2 * (b * b * b) / (27 * (a * a * a)) + c / a;
  const float acos_arg = 3 * q / (2 * p) * std::sqrt(-3 / p);

  if (acos_arg > 1 || acos_arg < -1) {
    throw std::runtime_error("balistics.cpp: compute_fall_time: target is too high");
  }

  const float phi = std::acos(acos_arg);
  const float t = 2 * std::sqrt(-p / 3) * std::cos((phi + 4 * std::numbers::pi_v<float>) / 3) - b / (3 * a);

  if (t <= 0) {
    throw std::runtime_error("balistics.cpp: compute_fall_time: time must be greater than 0");
  }

  return t;
}

auto compute_horizontal_travel(float t, float attack_speed, const Ammo& ammo) -> float
{
  const float g = 9.81f;
  const float m = ammo.mass;
  const float d = ammo.drag;
  const float l = ammo.lift;

  const float h = attack_speed * t - (t * t) * d * attack_speed / (2 * m) +
                  (t * t * t) * (6 * d * g * l * m - 6 * (d * d) * ((l * l) - 1) * attack_speed) / (36 * (m * m)) +
                  (t * t * t * t) *
                    (-6 * (d * d) * g * l * (1 + (l * l) + (l * l * l * l)) * m + 3 * (d * d * d) * (l * l) * (1 + (l * l)) * attack_speed +
                     6 * (d * d * d) * (l * l * l * l) * (1 + (l * l)) * attack_speed) /
                    (36 * (1 + (l * l)) * (1 + (l * l)) * (m * m * m)) +
                  (t * t * t * t * t) *
                    (3 * (d * d * d) * g * (l * l * l) * m - 3 * (d * d * d * d) * (l * l) * (1 + (l * l)) * attack_speed) /
                    (36 * (1 + (l * l)) * (m * m * m * m));

  if (h <= 0) {
    throw std::runtime_error("balistics.cpp: compute_horizontal_travel: horizontal distance must be greater than 0");
  }

  return h;
}

auto compute_fire_plan(Point2D drone, Point2D target, float h, float acceleration_path) -> FirePlan
{
  float distance_to_target = std::sqrt(((target.x - drone.x) * (target.x - drone.x)) + ((target.y - drone.y) * (target.y - drone.y)));

  if (distance_to_target <= 0) {
    throw std::runtime_error("balistics.cpp: compute_fire_plan: distance to target must be greater than 0");
  }

  FirePlan plan{};
  plan.has_maneuver = false;
  plan.maneuver = Point2D{0.0f, 0.0f};

  if (h + acceleration_path > distance_to_target) {
    drone.x = target.x - (target.x - drone.x) * (h + acceleration_path) / distance_to_target;
    drone.y = target.y - (target.y - drone.y) * (h + acceleration_path) / distance_to_target;
    distance_to_target = std::sqrt(((target.x - drone.x) * (target.x - drone.x)) + ((target.y - drone.y) * (target.y - drone.y)));

    plan.has_maneuver = true;
    plan.maneuver = Point2D{drone.x, drone.y};
  }

  const float ratio = (distance_to_target - h) / distance_to_target;
  plan.fire.x = drone.x + (target.x - drone.x) * ratio;
  plan.fire.y = drone.y + (target.y - drone.y) * ratio;

  return plan;
}