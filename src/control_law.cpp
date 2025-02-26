#include <std_msgs/msg/float32_multi_array.h>

#include "control_law.h"

void init_motor(motor m) {
  pinMode(m.pwm, OUTPUT);
  pinMode(m.dir1, OUTPUT);
  pinMode(m.dir2, OUTPUT);

  pinMode(m.encoder_a, INPUT_PULLUP);
  pinMode(m.encoder_b, INPUT_PULLUP);

  m.encoder_pos = 0;
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
  RCCHECK(rclc_executor_add_subscription(
      &_control_law_executor, &_control_law_subscriber, &_control_law_msg,
      &control_law_callback, ON_NEW_DATA));
}

void control_law_callback(const void *msgin) {
  auto control_voltages = (const std_msgs__msg__Float32MultiArray *)msgin;

  // Apply new voltages (new PWM)
  control_motor(motor1, control_voltages->data.data[0]);
  control_motor(motor2, control_voltages->data.data[1]);
}

void control_motor(motor m, uint8_t volt) {
  // Positive voltage
  if (volt > 0) {
    digitalWrite(m.dir1, HIGH);
    digitalWrite(m.dir2, LOW);
    volt = volt;
  } else {
    // Negative voltage
    digitalWrite(m.dir1, LOW);
    digitalWrite(m.dir2, HIGH);
    volt = -volt;
  }

  analogWrite(m.pwm, constrain(volt, 0, 255));
}