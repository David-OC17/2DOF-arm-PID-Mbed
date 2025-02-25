#include <Arduino.h>

#include <micro_ros_arduino.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdio.h>

#include <std_msgs/msg/int32.h>

#include "utils.h"

rcl_publisher_t _joint_state_publisher;
rcl_node_t _joint_state_node;
rcl_timer_t _joint_state_timer;
std_msgs__msg__Int32 _joint_state_msg; // TODO change msg type to fit appropriate precision and dimension

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

#define JOINT_STATE_TIMEOUT 1000 // How often joint state is published

void joint_state_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    // TODO change what is done in this callback
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&_joint_state_publisher, &_joint_state_msg, NULL));
    _joint_state_msg.data++;
  }
}

void init_joint_state() {
  // Create node
  RCCHECK(rclc_node_init_default(&_joint_state_node, "joint_state_node", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
      &_joint_state_publisher, &_joint_state_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "joint_state_node_publisher"));

  allocator = rcl_get_default_allocator();

  // Create timer
  RCCHECK(rclc_timer_init_default(&_joint_state_timer, &support,
                                  RCL_MS_TO_NS(JOINT_STATE_TIMEOUT),
                                  joint_state_timer_callback));

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
}

void init_micro_ros_extras() {

  // Create executor and add the specified timers
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &_joint_state_timer));
}

void setup() {
  // NOTE call these functions in order
  set_microros_transports();
  delay(2000);

  init_micro_ros_extras();
  init_utils();
  init_joint_state();

  // Init value of msg in and out
  _joint_state_msg.data = 0;
}

void loop() {
  // Increment by a common divisor of timeouts for timers in nodes
  delay(100);

  // Spin nodes to allow for update of topics _joint_state and _control_law
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}