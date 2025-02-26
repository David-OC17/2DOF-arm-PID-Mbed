#ifndef __JOINT_STATE_H__
#define __JOINT_STATE_H__

#include <stdio.h>

#include <Arduino.h>

#include <micro_ros_arduino.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "kalman_filter.h"
#include "common.h"

rcl_publisher_t _joint_state_publisher;
rcl_node_t _joint_state_node;
rcl_timer_t _joint_state_timer;

size_t joint_state_size = 4; // 2 position, 2 velocity
std_msgs__msg__Float32MultiArray
    _joint_state_msg;

#define JOINT_STATE_TIMEOUT_MS 1000 // How often joint state is published
#define ENCODER_INTERRUPT_INTERVAL 1000

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

void init_joint_state();
void init_joint_state_values();
void init_joint_state_kalman();

void init_encoder_interrupt(motor m);

void joint_state_timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void update_encoder_motor1();
void update_encoder_motor1();

#endif // __JOINT_STATE_H__