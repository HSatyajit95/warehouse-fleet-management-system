#include "fms_robot_agent/robot_fsm.hpp"

#include <gtest/gtest.h>

namespace fms_robot_agent {

TEST(RobotFSM, StartsIdle)
{
  RobotFSM fsm;
  EXPECT_EQ(fsm.state(), RobotState::IDLE);
}

TEST(RobotFSM, FullHappyPathCycleCoversBothPickAndDropLegs)
{
  // A task has two navigation legs (pick, then drop), each followed by an
  // EXECUTING phase: NAVIGATING/EXECUTING are visited twice before IDLE.
  RobotFSM fsm;
  EXPECT_TRUE(fsm.process(FsmEvent::TASK_RECEIVED));
  EXPECT_EQ(fsm.state(), RobotState::ASSIGNED);

  EXPECT_TRUE(fsm.process(FsmEvent::NAV_STARTED));
  EXPECT_EQ(fsm.state(), RobotState::NAVIGATING);

  EXPECT_TRUE(fsm.process(FsmEvent::NAV_SUCCEEDED));  // arrived at pick
  EXPECT_EQ(fsm.state(), RobotState::EXECUTING);

  EXPECT_TRUE(fsm.process(FsmEvent::EXECUTION_DONE));  // pick done, head to drop
  EXPECT_EQ(fsm.state(), RobotState::NAVIGATING);

  EXPECT_TRUE(fsm.process(FsmEvent::NAV_SUCCEEDED));  // arrived at drop
  EXPECT_EQ(fsm.state(), RobotState::EXECUTING);

  EXPECT_TRUE(fsm.process(FsmEvent::REPORT_SENT));  // drop done, report
  EXPECT_EQ(fsm.state(), RobotState::IDLE);
}

TEST(RobotFSM, NavFailureGoesToRecovering)
{
  RobotFSM fsm;
  fsm.process(FsmEvent::TASK_RECEIVED);
  fsm.process(FsmEvent::NAV_STARTED);
  ASSERT_EQ(fsm.state(), RobotState::NAVIGATING);

  EXPECT_TRUE(fsm.process(FsmEvent::NAV_FAILED));
  EXPECT_EQ(fsm.state(), RobotState::RECOVERING);
}

TEST(RobotFSM, FaultInjectedWhileNavigatingGoesToRecovering)
{
  RobotFSM fsm;
  fsm.process(FsmEvent::TASK_RECEIVED);
  fsm.process(FsmEvent::NAV_STARTED);
  ASSERT_EQ(fsm.state(), RobotState::NAVIGATING);

  EXPECT_TRUE(fsm.process(FsmEvent::FAULT_INJECTED));
  EXPECT_EQ(fsm.state(), RobotState::RECOVERING);
}

TEST(RobotFSM, RecoveryDoneReturnsToAssigned)
{
  RobotFSM fsm;
  fsm.process(FsmEvent::TASK_RECEIVED);
  fsm.process(FsmEvent::NAV_STARTED);
  fsm.process(FsmEvent::NAV_FAILED);
  ASSERT_EQ(fsm.state(), RobotState::RECOVERING);

  EXPECT_TRUE(fsm.process(FsmEvent::RECOVERY_DONE));
  EXPECT_EQ(fsm.state(), RobotState::ASSIGNED);
}

TEST(RobotFSM, RecoveryFailedReturnsToIdle)
{
  RobotFSM fsm;
  fsm.process(FsmEvent::TASK_RECEIVED);
  fsm.process(FsmEvent::NAV_STARTED);
  fsm.process(FsmEvent::NAV_FAILED);
  ASSERT_EQ(fsm.state(), RobotState::RECOVERING);

  EXPECT_TRUE(fsm.process(FsmEvent::RECOVERY_FAILED));
  EXPECT_EQ(fsm.state(), RobotState::IDLE);
}

TEST(RobotFSM, LowBatteryGoesToChargingFromAnyInterruptibleState)
{
  for (FsmEvent setup_event : {FsmEvent::TASK_RECEIVED}) {
    RobotFSM fsm;
    fsm.process(setup_event);
    ASSERT_EQ(fsm.state(), RobotState::ASSIGNED);
    EXPECT_TRUE(fsm.process(FsmEvent::LOW_BATTERY));
    EXPECT_EQ(fsm.state(), RobotState::CHARGING);
  }

  RobotFSM idle_fsm;
  EXPECT_TRUE(idle_fsm.process(FsmEvent::LOW_BATTERY));
  EXPECT_EQ(idle_fsm.state(), RobotState::CHARGING);
}

TEST(RobotFSM, BatteryFullReturnsToIdleFromCharging)
{
  RobotFSM fsm;
  fsm.process(FsmEvent::LOW_BATTERY);
  ASSERT_EQ(fsm.state(), RobotState::CHARGING);

  EXPECT_TRUE(fsm.process(FsmEvent::BATTERY_FULL));
  EXPECT_EQ(fsm.state(), RobotState::IDLE);
}

TEST(RobotFSM, InvalidTransitionIsIgnoredAndReturnsFalse)
{
  RobotFSM fsm;
  ASSERT_EQ(fsm.state(), RobotState::IDLE);

  // IDLE has no transition for NAV_SUCCEEDED.
  EXPECT_FALSE(fsm.process(FsmEvent::NAV_SUCCEEDED));
  EXPECT_EQ(fsm.state(), RobotState::IDLE);
}

TEST(RobotFSM, SecondNavSegmentSelfLoopReportsNoChange)
{
  RobotFSM fsm;
  fsm.process(FsmEvent::TASK_RECEIVED);
  fsm.process(FsmEvent::NAV_STARTED);
  ASSERT_EQ(fsm.state(), RobotState::NAVIGATING);

  // NAV_STARTED while already NAVIGATING is a valid self-loop (2nd leg of
  // a pick->drop task) but doesn't change state, so process() reports no
  // transition occurred even though the event was accepted.
  EXPECT_FALSE(fsm.process(FsmEvent::NAV_STARTED));
  EXPECT_EQ(fsm.state(), RobotState::NAVIGATING);
}

TEST(RobotFSM, StateNameMatchesEachState)
{
  RobotFSM fsm;
  EXPECT_STREQ(fsm.state_name(), "IDLE");

  fsm.process(FsmEvent::TASK_RECEIVED);
  EXPECT_STREQ(fsm.state_name(), "ASSIGNED");

  fsm.process(FsmEvent::NAV_STARTED);
  EXPECT_STREQ(fsm.state_name(), "NAVIGATING");

  fsm.process(FsmEvent::NAV_SUCCEEDED);
  EXPECT_STREQ(fsm.state_name(), "EXECUTING");
}

}  // namespace fms_robot_agent
