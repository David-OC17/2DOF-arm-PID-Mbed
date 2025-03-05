#ifndef __CONTROL_LAW_H__
#define __CONTROL_LAW_H__

#include <Arduino.h>

#include <std_msgs/msg/int32.h>

#include "common.h"

#define MAX_MOTOR_VOLT 12.0
#define MIN_2MOVE_MOTOR_VOLT float(MAX_MOTOR_VOLT * 0.2)

extern rcl_subscription_t _control_law_subscriber;
extern rcl_node_t _control_law_node;
extern rcl_timer_t _control_law_timer;

extern std_msgs__msg__Float32MultiArray _control_law_msg;

void init_motor(motor m);
void init_control_law();

void control_law_callback(const void *msgin);
void control_motor(motor m, uint8_t volt);

#endif // __CONTROL_LAW_H__ 