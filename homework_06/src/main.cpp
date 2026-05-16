#include <iostream>
#include <cmath>
#include <cstring>
#include <fstream>

int main()
{
  std::ifstream input("input.txt");

  if (!input.is_open()) {
    std::cout << "Could not open input file\n";
    return 1;
  }

  const float g = 9.81f;

  float xd, yd, zd, targetX, targetY, attackSpeed, accelerationPath, m, d, l;
  char ammo_name[15];

  input >> xd >> yd >> zd >> targetX >> targetY >> attackSpeed >> accelerationPath >> ammo_name;
  input.close();

  if (strcmp(ammo_name, "VOG-17") == 0) {
    m = 0.35f;
    d = 0.07f;
    l = 0.0f;
  }
  else if (strcmp(ammo_name, "M67") == 0) {
    m = 0.6f;
    d = 0.1f;
    l = 0.0f;
  }
  else if (strcmp(ammo_name, "RKG-3") == 0) {
    m = 1.2f;
    d = 0.1f;
    l = 0.0f;
  }
  else if (strcmp(ammo_name, "GLIDING-VOG") == 0) {
    m = 0.45f;
    d = 0.1f;
    l = 1.0f;
  }
  else if (strcmp(ammo_name, "GLIDING-RKG") == 0) {
    m = 1.4f;
    d = 0.1f;
    l = 1.0f;
  }
  else {
    std::cout << "Unknown ammo\n";
    return 1;
  }

  float a = d * g * m - 2 * pow(d, 2) * l * attackSpeed;
  float b = -3 * g * pow(m, 2) + 3 * d * l * m * attackSpeed;
  float c = 6 * pow(m, 2) * zd;
  float p = -pow(b, 2) / (3 * pow(a, 2));
  float q = 2 * pow(b, 3) / (27 * pow(a, 3)) + c / a;
  float acos_arg = 3 * q / (2 * p) * sqrt(-3 / p);

  if (acos_arg > 1 || acos_arg < -1) {
    std::cout << "Too hight\n";
    return 1;
  }

  float phi = acos(acos_arg);
  float t = 2 * sqrt(-p / 3) * cos((phi + 4 * M_PI) / 3) - b / (3 * a);

  if (t <= 0) {
    std::cout << "Time should be more than 0\n";
    return 1;
  }

  float h = attackSpeed * t - pow(t, 2) * d * attackSpeed / (2 * m) +
            pow(t, 3) * (6 * d * g * l * m - 6 * pow(d, 2) * (pow(l, 2) - 1) * attackSpeed) / (36 * pow(m, 2)) +
            pow(t, 4) *
              (-6 * pow(d, 2) * g * l * (1 + pow(l, 2) + pow(l, 4)) * m + 3 * pow(d, 3) * pow(l, 2) * (1 + pow(l, 2)) * attackSpeed +
               6 * pow(d, 3) * pow(l, 4) * (1 + pow(l, 2)) * attackSpeed) /
              (36 * pow(1 + pow(l, 2), 2) * pow(m, 3)) +
            pow(t, 5) * (3 * pow(d, 3) * g * pow(l, 3) * m - 3 * pow(d, 4) * pow(l, 2) * (1 + pow(l, 2)) * attackSpeed) /
              (36 * (1 + pow(l, 2)) * pow(m, 4));

  if (h <= 0) {
    std::cout << "Horizontal distance to the target should be more than 0\n";
    return 1;
  }

  float distanceToTarget = sqrt(pow((targetX - xd), 2) + pow((targetY - yd), 2));

  if (distanceToTarget <= 0) {
    std::cout << "Distance to the target should be more than 0\n";
    return 1;
  }

  std::ofstream output("output.txt", std::ios::app);

  if (!output.is_open()) {
    std::cerr << "Could not open output file\n";
    return 1;
  }

  // check if distanceToTarget < 1e-7

  if (h + accelerationPath > distanceToTarget) {
    xd = targetX - (targetX - xd) * (h + accelerationPath) / distanceToTarget;
    yd = targetY - (targetY - yd) * (h + accelerationPath) / distanceToTarget;
    distanceToTarget = sqrt(pow(targetX - xd, 2) + pow(targetY - yd, 2));
    output << xd << " " << yd << "\n";
  }

  float ratio = (distanceToTarget - h) / distanceToTarget;
  float fireX = xd + (targetX - xd) * ratio;
  float fireY = yd + (targetY - yd) * ratio;

  output << fireX << " " << fireY << "\n";
  output.close();

  return 0;
}
