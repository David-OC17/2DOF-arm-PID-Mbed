#include <Arduino.h>
#include <stdio.h>

#include <micro_ros_arduino.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "kalman_filter.h"
#include "common.h"

typedef struct {
  float joint1_pos;
  float joint2_pos;

  float joint1_vel;
  float joint2_vel;
} joint_state;

rcl_publisher_t _joint_state_publisher;
rcl_node_t _joint_state_node;
rcl_timer_t _joint_state_timer;

size_t joint_state_size = 4; // 2 position, 2 velocity
std_msgs__msg__Float32MultiArray
    _joint_state_msg; // TODO change msg type to fit appropriate precision and
                      // dimension

#define JOINT_STATE_TIMEOUT_MS 1000 // How often joint state is published

float E_MEA_POS = 0.1; // Measurement uncertainty for position
float E_EST_POS = 1.0; // Estimation uncertainty for position
float Q_POS = 0.001;   // Process noise for position

float E_MEA_VEL = 0.05; // Measurement uncertainty for velocity
float E_EST_VEL = 0.5;  // Estimation uncertainty for velocity
float Q_VEL = 0.005;    // Process noise for velocity

KalmanFilter<float> joint_state_pos_kalman_filter(E_MEA_POS, E_EST_POS, Q_POS);
KalmanFilter<float> joint_state_vel_kalman_filter(E_MEA_VEL, E_EST_VEL, Q_VEL);

joint_state _measured_joint_state;
joint_state _estimated_joint_state;

joint_state take_measurement_encoders();
void init_joint_state_kalman();
void joint_state_timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void init_joint_state();
void init_joint_state_values();

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
