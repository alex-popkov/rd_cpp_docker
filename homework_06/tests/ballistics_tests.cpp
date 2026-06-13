#include "ballistics.hpp"

#include <gtest/gtest.h>

TEST(Ballistics_find_ammo, KnownAmmoReturnsCorrectParams)
{
  const Ammo vog = find_ammo("VOG-17");

  EXPECT_FLOAT_EQ(vog.mass, 0.35f);
  EXPECT_FLOAT_EQ(vog.drag, 0.07f);
  EXPECT_FLOAT_EQ(vog.lift, 0.0f);
}

TEST(Ballistics_find_ammo, UnknownNameThrows)
{
  EXPECT_THROW(find_ammo("UNKNOWN"), std::runtime_error);
}

TEST(Ballistics_find_ammo, EmptyNameThrows)
{
  EXPECT_THROW(find_ammo(""), std::runtime_error);
}

TEST(Ballistics_find_ammo, CaseSensitiveLookup)
{
  EXPECT_THROW(find_ammo("vog-17"), std::runtime_error);
}

TEST(Ballistics_compute_fall_time, VOG17_From100m_At10mps)
{
  const Ammo ammo = {0.35f, 0.07f, 0.0f};
  const float t = compute_fall_time(100.0f, 10.0f, ammo);
  EXPECT_NEAR(t, 5.7497f, 0.01f);
}

TEST(Ballistics, ReferenceVog17DropPoint)
{
  const Ammo ammo = find_ammo("VOG-17");

  const float zd = 100.0f;
  const float attack_speed = 10.0f;
  const float acceleration_path = 10.0f;
  const Point2D drone = {100.0f, 100.0f};
  const Point2D target = {200.0f, 200.0f};

  const float t = compute_fall_time(zd, attack_speed, ammo);
  const float h = compute_horizontal_travel(t, attack_speed, ammo);
  const FirePlan plan = compute_fire_plan(drone, target, h, acceleration_path);

  EXPECT_FALSE(plan.has_maneuver);
  EXPECT_NEAR(plan.fire.x, 173.759f, 0.01f);
  EXPECT_NEAR(plan.fire.y, 173.759f, 0.01f);
}