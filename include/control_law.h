#ifndef __CONTROL_LAW_H__
#define __CONTROL_LAW_H__

#include <Arduino.h>
#include <TimerOne.h>

#include <std_msgs/msg/int32.h>

#include "common.h"

rcl_subscription_t _control_law_subscriber;
rcl_node_t _control_law_node;
rcl_timer_t _control_law_timer;

size_t control_law_size = 2; // 2 voltages
std_msgs__msg__Float32MultiArray _control_law_msg;

void init_motor(motor m);
void init_control_law();

void control_law_callback(const void *msgin);
void control_motor(motor m, uint8_t volt);

#endif // __CONTROL_LAW_H__ 