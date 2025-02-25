#include <Arduino.h>

#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define ERROR_LED_PIN 13

rclc_executor_t _joint_state_executor;
rclc_executor_t _control_law_executor;

rclc_support_t _support;
rcl_allocator_t _allocator = rcl_get_default_allocator();

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      error_loop();                                                            \
    }                                                                          \
  }

#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
    }                                                                          \
  }

void init_micro_ros_nodes();
void init_utils();
void error_loop();

void init_micro_ros_nodes() {
  // Create init_options
  RCCHECK(rclc_support_init(&_support, 0, NULL, &_allocator));

  // Create executor and add the specified timers
  RCCHECK(rclc_executor_init(&_joint_state_executor, &_support.context, 2, &_allocator));
}

void init_utils() {
  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(ERROR_LED_PIN, HIGH);
}

void error_loop() {
  while (1) {
    digitalWrite(ERROR_LED_PIN, !digitalRead(ERROR_LED_PIN));
    delay(100);
  }
}