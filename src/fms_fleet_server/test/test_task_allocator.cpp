#include "fms_fleet_server/task_allocator.hpp"

#include <gtest/gtest.h>

namespace fms_fleet_server {
namespace {

fms_msgs::msg::RobotStatus make_robot(const std::string& robot_id, double x, double y,
                                       float soc, uint8_t state = fms_msgs::msg::RobotStatus::STATE_IDLE)
{
  fms_msgs::msg::RobotStatus status;
  status.robot_id = robot_id;
  status.state = state;
  status.battery_soc = soc;
  status.pose.position.x = x;
  status.pose.position.y = y;
  return status;
}

fms_msgs::msg::TaskAssignment make_request(double pick_x, double pick_y)
{
  fms_msgs::msg::TaskAssignment request;
  request.pick_pose.position.x = pick_x;
  request.pick_pose.position.y = pick_y;
  return request;
}

}  // namespace

TEST(TaskAllocator, ReturnsNulloptWhenNoRobotsTracked)
{
  std::map<std::string, fms_msgs::msg::RobotStatus> robots;
  auto result = select_best_robot(robots, make_request(0.0, 0.0), 0.5);
  EXPECT_FALSE(result.has_value());
}

TEST(TaskAllocator, ReturnsNulloptWhenNoRobotIsIdle)
{
  std::map<std::string, fms_msgs::msg::RobotStatus> robots;
  robots["robot_1"] = make_robot("robot_1", 0.0, 0.0, 100.0, fms_msgs::msg::RobotStatus::STATE_NAVIGATING);
  robots["robot_2"] = make_robot("robot_2", 1.0, 1.0, 100.0, fms_msgs::msg::RobotStatus::STATE_CHARGING);

  auto result = select_best_robot(robots, make_request(0.0, 0.0), 0.5);
  EXPECT_FALSE(result.has_value());
}

TEST(TaskAllocator, PicksTheCloserIdleRobotWhenSocIsEqual)
{
  std::map<std::string, fms_msgs::msg::RobotStatus> robots;
  robots["robot_far"]   = make_robot("robot_far", 100.0, 0.0, 80.0);
  robots["robot_close"] = make_robot("robot_close", 1.0, 0.0, 80.0);

  auto result = select_best_robot(robots, make_request(0.0, 0.0), 0.5);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->first, "robot_close");
}

TEST(TaskAllocator, PicksTheHigherSocRobotWhenDistanceIsEqual)
{
  std::map<std::string, fms_msgs::msg::RobotStatus> robots;
  robots["robot_low_soc"]  = make_robot("robot_low_soc", 5.0, 0.0, 20.0);
  robots["robot_high_soc"] = make_robot("robot_high_soc", 5.0, 0.0, 90.0);

  auto result = select_best_robot(robots, make_request(0.0, 0.0), 0.5);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->first, "robot_high_soc");
}

TEST(TaskAllocator, IgnoresNonIdleRobotsEvenIfCloser)
{
  std::map<std::string, fms_msgs::msg::RobotStatus> robots;
  robots["robot_close_busy"] = make_robot(
      "robot_close_busy", 1.0, 0.0, 100.0, fms_msgs::msg::RobotStatus::STATE_EXECUTING);
  robots["robot_far_idle"] = make_robot("robot_far_idle", 50.0, 0.0, 100.0);

  auto result = select_best_robot(robots, make_request(0.0, 0.0), 0.5);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->first, "robot_far_idle");
}

TEST(TaskAllocator, ScoreMatchesDistancePlusSocWeightedTerm)
{
  std::map<std::string, fms_msgs::msg::RobotStatus> robots;
  // dx=-3, dy=4 -> distance=5 (3-4-5 triangle), soc=70 -> (100-70)*0.5=15
  robots["robot_1"] = make_robot("robot_1", 3.0, 0.0, 70.0);

  auto result = select_best_robot(robots, make_request(0.0, 4.0), 0.5);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->second, 5.0 + 15.0);
}

}  // namespace fms_fleet_server
