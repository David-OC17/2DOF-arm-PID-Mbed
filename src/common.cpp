#include "common.h"

rclc_executor_t _joint_state_executor;
rclc_executor_t _control_law_executor;

rclc_support_t _support;
rcl_allocator_t _allocator;

void init_micro_ros_nodes() {
  // TODO check if one needs only one allocator and support for both, or if one is OK
  _allocator = rcl_get_default_allocator();

  rcl_ret_t ret = rclc_support_init(&_support, 0, NULL, &_allocator);
  if (ret != RCL_RET_OK) {
    error_loop();
  }

  // Create init_options
  RCCHECK(rclc_support_init(&_support, 0, NULL, &_allocator));

  // Create executor and add the specified timers
  RCCHECK(rclc_executor_init(&_joint_state_executor, &_support.context, 1,
                             &_allocator));

  RCCHECK(rclc_executor_init(&_control_law_executor, &_support.context, 1,
                             &_allocator));
}

void init_utils() {
  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(ERROR_LED_PIN, HIGH);
}

void ros_init() {
  set_microros_transports();

  _joint_state_executor = rclc_executor_get_zero_initialized_executor();
  _control_law_executor = rclc_executor_get_zero_initialized_executor();
}

void error_loop() {
  while (1) {
    digitalWrite(ERROR_LED_PIN, !digitalRead(ERROR_LED_PIN));
    delay(100);
  }
}
