#include "balistics.hpp"

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

TEST(Ballistics__compute_fall_time, VOG17_From100m_At10mps)
{
  const Ammo ammo = {0.35f, 0.07f, 0.0f};
  const float t = compute_fall_time(100.0f, 10.0f, ammo);
  EXPECT_NEAR(t, 5.7497f, 0.001f);
}
