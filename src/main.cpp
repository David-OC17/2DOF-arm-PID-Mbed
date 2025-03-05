#include <Arduino.h>

#include "common.h"
#include "joint_state.h"
#include "control_law.h"

#define WAIT_ROS_INIT 2000
#define LOOP_SPIN_CHECK_TIMEOUT_MS 20
#define CONTROL_LAW_EXECUTOR_SPIN_TIMEOUT_MS 5 * LOOP_SPIN_CHECK_TIMEOUT_MS
#define JOINT_STATE_EXECUTOR_SPIN_TIMEOUT_MS 5 * LOOP_SPIN_CHECK_TIMEOUT_MS

void setup() {
  init_utils(); // Currently disconnected / LED unused
  ros_init();
  delay(WAIT_ROS_INIT); // wait for ROS init

  motor1.pwm = MOTOR1_PWM;
  motor1.dir1 = MOTOR1_DIR1;
  motor1.dir2 = MOTOR1_DIR2;
  motor1.encoder_a = MOTOR1_ENCODERA;
  motor1.encoder_b = MOTOR1_ENCODERB;
  init_motor(motor1);

  motor2.pwm = MOTOR2_PWM;
  motor2.dir1 = MOTOR2_DIR1;
  motor2.dir2 = MOTOR2_DIR2;
  motor2.encoder_a = MOTOR2_ENCODERA;
  motor2.encoder_b = MOTOR2_ENCODERB;
  init_motor(motor2);

  // Set to default values
  init_joint_state_values(LINK1_LENGTH_MM, LINK2_LENGTH_MM);
  init_joint_state_kalman(ENCODER1_INITAL_POS_DEG, ENCODER2_INITAL_POS_DEG);

  // Control law config node and timer
  init_control_law();

  // Joint state config node and timer
  init_joint_state();

  // Call executors to start ROS2 nodes
  init_micro_ros_nodes();

  // For both motors in encoder_a
  init_encoder_interrupt();
}

void loop() {
  // Increment by a common divisor of timeouts for timers in nodes
  delay(LOOP_SPIN_CHECK_TIMEOUT_MS);

  // Spin _control_law_executor to update control law from subscription and send to drivers
  RCSOFTCHECK(rclc_executor_spin_some(&_control_law_executor, RCL_MS_TO_NS(CONTROL_LAW_EXECUTOR_SPIN_TIMEOUT_MS)));

  // Spin _joint_state_executor to state (with Kalman belief) and publish
  RCSOFTCHECK(rclc_executor_spin_some(&_joint_state_executor, RCL_MS_TO_NS(JOINT_STATE_EXECUTOR_SPIN_TIMEOUT_MS)));
}