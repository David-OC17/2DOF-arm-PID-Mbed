#include <Arduino.h>

#include <micro_ros_arduino.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdio.h>

#include <std_msgs/msg/int32.h>

#include "utils.h"

rcl_subscription_t _control_law_subscriber;
std_msgs__msg__Int32 _control_law_msg; // TODO change msg type to fit
                                       // appropriate precision and dimension
rcl_node_t _control_law_node;
rcl_timer_t _control_law_timer;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

void control_law_callback(const void *msgin) {
  // TODO change what is done in this callback
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(ERROR_LED_PIN, (msg->data == 0) ? LOW : HIGH);
}

void init_control_law() {
  // Create node
  RCCHECK(rclc_node_init_default(&_control_law_node, "control_law_node", "",
                                 &support));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
      &_control_law_subscriber, &_control_law_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "control_law_node_subscriber"));
}

void setup() {

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &_control_law_subscriber,
                                         &_control_law_msg,
                                         &control_law_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}