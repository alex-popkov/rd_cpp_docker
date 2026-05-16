#include "balistics.hpp"

#include <gtest/gtest.h>

TEST(find_ammo, KnownAmmoReturnsCorrectParams)
{
  const Ammo vog = find_ammo("VOG-17");

  EXPECT_FLOAT_EQ(vog.mass, 0.35f);
  EXPECT_FLOAT_EQ(vog.drag, 0.07f);
  EXPECT_FLOAT_EQ(vog.lift, 0.0f);
}