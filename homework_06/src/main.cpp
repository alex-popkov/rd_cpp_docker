#include "ballistics.hpp"

#include <fstream>
#include <iostream>
#include <string>

const int kMaxAmmoNameLength = 15;

namespace {
void log_error(const std::string& msg)
{
  std::cerr << "[ERROR] " << msg << "\n";
}
}  // namespace

auto main(int argc, char** argv) -> int
{
  // The executable expects an input file path and an output file path.
  if (argc != 3) {
    log_error("usage: balistics_check <input_path> <output_path>");

    return 1;
  }

  try {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic) — argv is a raw C pointer
    std::ifstream input(argv[1]);

    if (!input.is_open()) {
      log_error("could not open input file");

      return 1;
    }

    float xd = 0.0f;
    float yd = 0.0f;
    float zd = 0.0f;
    float target_x = 0.0f;
    float target_y = 0.0f;
    float attack_speed = 0.0f;
    float acceleration_path = 0.0f;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) — fixed-size buffer is sufficient for known ammo names
    char ammo_name[kMaxAmmoNameLength];

    input >> xd >> yd >> zd >> target_x >> target_y >> attack_speed >> acceleration_path >> ammo_name;
    input.close();

    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay) — find_ammo takes a C string
    const Ammo ammo = find_ammo(ammo_name);
    const float t = compute_fall_time(zd, attack_speed, ammo);
    const float h = compute_horizontal_travel(t, attack_speed, ammo);

    const Point2D drone = {xd, yd};
    const Point2D target = {target_x, target_y};
    const FirePlan plan = compute_fire_plan(drone, target, h, acceleration_path);

    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic) — argv is a raw C pointer
    std::ofstream output(argv[2], std::ios::app);
    if (!output.is_open()) {
      log_error("could not open output file");

      return 1;
    }

    if (plan.has_maneuver) {
      output << plan.maneuver.x << " " << plan.maneuver.y << "\n";
    }
    output << plan.fire.x << " " << plan.fire.y << "\n";
    output.close();

    return 0;
  }
  catch (const std::exception& error) {
    log_error(error.what());

    return 1;
  }
}