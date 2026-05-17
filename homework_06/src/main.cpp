#include "ballistics.hpp"

#include <iostream>
#include <fstream>

#define ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl

int main(int argc, char** argv)
{
  // The executable expects an input file path and an output file path.
  if (argc != 3) {
    ERROR("usage: balistics_check <input_path> <output_path>");

    return 1;
  }

  try {
    std::ifstream input(argv[1]);

    if (!input.is_open()) {
      ERROR("could not open input file");

      return 1;
    }

    float xd, yd, zd, targetX, targetY, attackSpeed, accelerationPath;
    char ammo_name[15];

    input >> xd >> yd >> zd >> targetX >> targetY >> attackSpeed >> accelerationPath >> ammo_name;
    input.close();

    const Ammo ammo = find_ammo(ammo_name);
    const float t = compute_fall_time(zd, attackSpeed, ammo);
    const float h = compute_horizontal_travel(t, attackSpeed, ammo);

    const Point2D drone = {xd, yd};
    const Point2D target = {targetX, targetY};
    const FirePlan plan = compute_fire_plan(drone, target, h, accelerationPath);

    std::ofstream output(argv[2], std::ios::app);
    if (!output.is_open()) {
      ERROR("could not open output file");

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
    ERROR(error.what());

    return 1;
  }
}