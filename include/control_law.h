#include <Arduino.h>

#include <std_msgs/msg/int32.h>

#include "driver.h"
#include "common.h"

rcl_subscription_t _control_law_subscriber;
rcl_node_t _control_law_node;
rcl_timer_t _control_law_timer;

size_t control_law_size = 4; // 2 position, 2 velocity
std_msgs__msg__Float32MultiArray
    _control_law_msg;


void control_law_callback(const void *msgin) {
  // TODO change what is done in this callback
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(ERROR_LED_PIN, (msg->data == 0) ? LOW : HIGH);
}

void init_control_law() {
  // Create node
  RCCHECK(rclc_node_init_default(&_control_law_node, "control_law_node", "",
                                 &_support));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
      &_control_law_subscriber, &_control_law_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "control_law_node_subscriber"));

  // Create callback on receive to topic
  RCCHECK(rclc_executor_add_subscription(&_control_law_executor, &_control_law_subscriber,
                                         &_control_law_msg,
                                         &control_law_callback, ON_NEW_DATA));
}