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

void init_encoder_interrupt(motor m) {
  attachInterrupt(digitalPinToInterrupt(motor1.encoder_a), update_encoder_motor1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor2.encoder_a), update_encoder_motor2, CHANGE);
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

void update_encoder_motor1() {
  int encoder_a_state = digitalRead(motor1.encoder_a);
  int encoder_b_state = digitalRead(motor1.encoder_b);

  // Determine direction of rotation
  if (encoder_a_state == HIGH) {
    if (encoder_b_state == LOW) {
      motor1.encoder_pos++;
    }
    else {
      motor1.encoder_pos--;
    }
  }
  else {
    if (encoder_b_state == LOW) {
      motor1.encoder_pos--;
    }
    else {
      motor1.encoder_pos++;
    }
  }
}

void update_encoder_motor2() {
  int encoder_a_state = digitalRead(motor2.encoder_a);
  int encoder_b_state = digitalRead(motor2.encoder_b);

  // Determine direction of rotation
  if (encoder_a_state == HIGH) {
    if (encoder_b_state == LOW) {
      motor2.encoder_pos++;
    }
    else {
      motor2.encoder_pos--;
    }
  }
  else {
    if (encoder_b_state == LOW) {
      motor2.encoder_pos--;
    }
    else {
      motor2.encoder_pos++;
    }
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

void init_joint_state() {
  // Create node
  RCCHECK(rclc_node_init_default(&_joint_state_node, "joint_state_node", "",
                                 &_support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
      &_joint_state_publisher, &_joint_state_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "joint_state_node_publisher"));

  // Create timer
  RCCHECK(rclc_timer_init_default(&_joint_state_timer, &_support,
                                  RCL_MS_TO_NS(JOINT_STATE_TIMEOUT_MS),
                                  joint_state_timer_callback));

  RCCHECK(rclc_executor_add_timer(&_joint_state_executor, &_joint_state_timer));
}
