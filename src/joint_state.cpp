#include "joint_state.h"

joint_state take_measurement_encoders() {
  joint_state temp_state;
  // TODO take measurement of encoders and translate to joint state (pos)
  return temp_state;
}

void init_joint_state_kalman() {
  // Set initial belief of joint state to a normal dist.

  // Update u and cov with these measurements
}

void joint_state_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  _measured_joint_state = take_measurement_encoders();

  // Update position
  _estimated_joint_state.joint1_pos =
      joint_state_pos_kalman_filter.updateEstimate(0);
  _estimated_joint_state.joint2_pos =
      joint_state_pos_kalman_filter.updateEstimate(0);

  // Update position velocity
  _estimated_joint_state.joint1_vel =
      joint_state_vel_kalman_filter.updateEstimate(0);
  _estimated_joint_state.joint2_vel =
      joint_state_vel_kalman_filter.updateEstimate(0);

  // Update ROS msg and publish
  _joint_state_msg.data.data[0] = _estimated_joint_state.joint1_pos;
  _joint_state_msg.data.data[1] = _estimated_joint_state.joint2_pos;
  _joint_state_msg.data.data[2] = _estimated_joint_state.joint1_vel;
  _joint_state_msg.data.data[3] = _estimated_joint_state.joint1_vel;

  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&_joint_state_publisher, &_joint_state_msg, NULL));
  }
}

void init_joint_state_values() {
  // Initially both motors at 90deg and stationary
  _estimated_joint_state.joint1_pos = 90;
  _estimated_joint_state.joint2_pos = 90;

  _estimated_joint_state.joint1_vel = 0;
  _estimated_joint_state.joint2_vel = 0;

  // Init value of msg in and out
  _joint_state_msg.data.data[0] = _estimated_joint_state.joint1_pos;
  _joint_state_msg.data.data[1] = _estimated_joint_state.joint2_pos;
  _joint_state_msg.data.data[2] = _estimated_joint_state.joint1_vel;
  _joint_state_msg.data.data[3] = _estimated_joint_state.joint1_vel;
}
