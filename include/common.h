#ifndef __COMMON_H__
#define __COMMON_H__

#include <Arduino.h>

#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#define ERROR_LED_PIN 13

// TODO define pins for motor
#define MOTOR1_PWM 0
#define MOTOR1_DIR1 0
#define MOTOR1_DIR2 0
#define MOTOR1_ENCODERA 0
#define MOTOR1_ENCODERB 0

// TODO define pins for motor
#define MOTOR2_PWM 0
#define MOTOR2_DIR1 0
#define MOTOR2_DIR2 0
#define MOTOR2_ENCODERA 0
#define MOTOR2_ENCODERB 0

typedef struct {
  uint8_t pwm;
  uint8_t dir1;
  uint8_t dir2;

  uint8_t encoder_a;
  uint8_t encoder_b;

  volatile uint32_t encoder_pos;
} motor;

typedef struct {
  float joint1_pos;
  float joint2_pos;

  float joint1_vel;
  float joint2_vel;
} joint_state;

rclc_executor_t _joint_state_executor;
rclc_executor_t _control_law_executor;

rclc_support_t _support;
rcl_allocator_t _allocator;

motor motor1;
motor motor2;

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
  _allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&_support, 0, NULL, &_allocator));

  // Create executor and add the specified timers
  RCCHECK(rclc_executor_init(&_joint_state_executor, &_support.context, 2,
                             &_allocator));
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

#endif // __COMMON_H__