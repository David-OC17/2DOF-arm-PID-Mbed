#ifndef __JOINT_STATE_H__
#define __JOINT_STATE_H__

#include <stdio.h>

#include <Arduino.h>

#include <micro_ros_arduino.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "kf_2dof.h"
#include "common.h"

rcl_publisher_t _joint_state_publisher;
rcl_node_t _joint_state_node;
rcl_timer_t _joint_state_timer;

size_t joint_state_size = 4; // 2 position, 2 velocity
std_msgs__msg__Float32MultiArray
    _joint_state_msg;

#define JOINT_STATE_TIMEOUT_MS 1000 // How often joint state is published
#define ENCODER_INTERRUPT_INTERVAL 1000

float KALMAN_DT_MS = 0.01; // Time step in ms
float KALMAN_Q_NOISE = 0.001; // Process noise (control - motor driver)
float KALMAN_R_NOISE = 0.1; // Measurement noise (encoders)

float JOINT1_LEN = 1.0;
float JOINT2_LEN = 1.0;

KF_2DOF joint_state_kalman_filter(KALMAN_DT_MS, JOINT1_LEN, JOINT2_LEN, KALMAN_Q_NOISE, KALMAN_R_NOISE);

joint_state _measured_joint_state;
end_efector_state _estimated_end_efector_state;

joint_state take_measurement_encoders();

void init_joint_state();
void init_joint_state_values();
void init_joint_state_kalman();

void init_encoder_interrupt(motor m);

void joint_state_timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void update_encoder_motor1();
void update_encoder_motor1();

#endif // __JOINT_STATE_H__