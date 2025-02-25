#include <Arduino.h>

#include "common.h"
#include "joint_state.h"
#include "control_law.h"

// NOTE call these functions in order
void setup() {
  init_utils();
  set_microros_transports();
  delay(2000); // wait for ROS init

  init_joint_state();
  init_joint_state_kalman();

  init_control_law();

  init_micro_ros_nodes();

  init_joint_state_values();
}

void loop() {
  // Increment by a common divisor of timeouts for timers in nodes
  delay(100);

  // Spin _control_law_executor to update control law from subscription and send to drivers
  RCSOFTCHECK(rclc_executor_spin_some(&_control_law_executor, RCL_MS_TO_NS(100)));

  // Spin _joint_state_executor to state (with Kalman belief) and publish
  RCSOFTCHECK(rclc_executor_spin_some(&_joint_state_executor, RCL_MS_TO_NS(100)));
}