#include "fms_robot_agent/battery_model.hpp"

#include <gtest/gtest.h>

namespace fms_robot_agent {

TEST(BatteryModel, StartsAtRequestedSoc)
{
  BatteryModel battery(75.0);
  EXPECT_DOUBLE_EQ(battery.soc(), 75.0);
}

TEST(BatteryModel, ClampsInitialSocToValidRange)
{
  EXPECT_DOUBLE_EQ(BatteryModel(150.0).soc(), 100.0);
  EXPECT_DOUBLE_EQ(BatteryModel(-10.0).soc(), 0.0);
}

TEST(BatteryModel, DrainsFasterWhileMovingThanIdle)
{
  BatteryModel idle_battery(100.0);
  idle_battery.update(/*dt=*/10.0, /*is_moving=*/false, /*is_charging=*/false);

  BatteryModel moving_battery(100.0);
  moving_battery.update(/*dt=*/10.0, /*is_moving=*/true, /*is_charging=*/false);

  EXPECT_LT(moving_battery.soc(), idle_battery.soc());
}

TEST(BatteryModel, ChargesWhileDocked)
{
  BatteryModel battery(50.0);
  battery.update(/*dt=*/5.0, /*is_moving=*/false, /*is_charging=*/true);
  EXPECT_GT(battery.soc(), 50.0);
}

TEST(BatteryModel, SocNeverDropsBelowZero)
{
  BatteryModel battery(1.0);
  battery.update(/*dt=*/1000.0, /*is_moving=*/true, /*is_charging=*/false);
  EXPECT_DOUBLE_EQ(battery.soc(), 0.0);
}

TEST(BatteryModel, SocNeverExceedsHundred)
{
  BatteryModel battery(99.0);
  battery.update(/*dt=*/1000.0, /*is_moving=*/false, /*is_charging=*/true);
  EXPECT_DOUBLE_EQ(battery.soc(), 100.0);
}

TEST(BatteryModel, IsLowBelowThresholdOnly)
{
  BatteryModel low_battery(BatteryModel::LOW_THRESHOLD - 1.0);
  EXPECT_TRUE(low_battery.is_low());

  BatteryModel ok_battery(BatteryModel::LOW_THRESHOLD);
  EXPECT_FALSE(ok_battery.is_low());
}

TEST(BatteryModel, IsFullAtOrAboveThresholdOnly)
{
  BatteryModel full_battery(BatteryModel::FULL_THRESHOLD);
  EXPECT_TRUE(full_battery.is_full());

  BatteryModel not_full_battery(BatteryModel::FULL_THRESHOLD - 1.0);
  EXPECT_FALSE(not_full_battery.is_full());
}

TEST(BatteryModel, SetSocClampsToValidRange)
{
  BatteryModel battery;
  battery.set_soc(200.0);
  EXPECT_DOUBLE_EQ(battery.soc(), 100.0);

  battery.set_soc(-5.0);
  EXPECT_DOUBLE_EQ(battery.soc(), 0.0);
}

}  // namespace fms_robot_agent
