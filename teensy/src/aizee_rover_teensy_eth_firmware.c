/******************************************************************************
 * Teensy 4.1 Script for Aizee Rover
 ******************************************************************************/

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <Ethernet.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

// ------------------ Drive ESC Motor definitions -------------------------
// Define wheel motor pins for ESC PWM, direction, brake, and encoders.
//#define ESC_FR_PIN 10    // Originally Front Right ESC PWM
#define ESC_FR_PIN 5

#define ESC_FL_PIN 24    // Front Left ESC PWM
#define ESC_BR_PIN 4    // Back Right ESC PWM
//#define ESC_BL_PIN 5    // Back Left ESC PWM
#define ESC_BL_PIN 10

//#define DIR_FR_PIN 6    // Front Right Direction
#define DIR_FR_PIN 22

#define DIR_FL_PIN 7    // Front Left Direction
#define DIR_BR_PIN 8    // Back Right Direction
//#define DIR_BL_PIN 22    // Back Left Direction
#define DIR_BL_PIN 6

//#define BRAKE_FR_PIN 20 // Front Right Brake
#define BRAKE_FR_PIN 13

#define BRAKE_FL_PIN 11 // Front Left Brake
#define BRAKE_BR_PIN 12 // Back Right Brake
//#define BRAKE_BL_PIN 13 // Back Left Brake
#define BRAKE_BL_PIN 20

//#define ENC_FR_PIN 14   // Front Right Encoder
#define ENC_FR_PIN 17

#define ENC_FL_PIN 15   // Front Left Encoder
#define ENC_BR_PIN 16   // Back Right Encoder
//#define ENC_BL_PIN 17   // Back Left Encoder
#define ENC_BL_PIN 14

// Variables to store encoder counts for each wheel
volatile int32_t pulse_count_fr = 0;
volatile int32_t pulse_count_fl = 0;
volatile int32_t pulse_count_br = 0;
volatile int32_t pulse_count_bl = 0;

// Variables to hold the last direction for each wheel
volatile bool last_direction_fr = true;
volatile bool last_direction_fl = true;
volatile bool last_direction_br = true;
volatile bool last_direction_bl = true;

// ------------------ Micro-ROS Objects ---------------------------
rcl_node_t node;

rcl_subscription_t cmd_vel_sub;       // Drive system subscription for Twist commands
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_publisher_t encoder_pub_fr;
rcl_publisher_t encoder_pub_fl;
rcl_publisher_t encoder_pub_br;
rcl_publisher_t encoder_pub_bl;
std_msgs__msg__Int32MultiArray encoder_msg_fr;
std_msgs__msg__Int32MultiArray encoder_msg_fl;
std_msgs__msg__Int32MultiArray encoder_msg_br;
std_msgs__msg__Int32MultiArray encoder_msg_bl;

rcl_timer_t timer_fr;
rcl_timer_t timer_fl;
rcl_timer_t timer_br;
rcl_timer_t timer_bl;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

// ------------------ Function Prototypes ---------------------------
void cmd_vel_callback(const void *msgin);
void publish_wheel_encoder_fr(rcl_timer_t * timer, int64_t last_call_time);
void publish_wheel_encoder_fl(rcl_timer_t * timer, int64_t last_call_time);
void publish_wheel_encoder_br(rcl_timer_t * timer, int64_t last_call_time);
void publish_wheel_encoder_bl(rcl_timer_t * timer, int64_t last_call_time);
void setPWM_ESC(uint8_t pin, int speed);
void setDirection(uint8_t pin, bool forward);
void setBrake(uint8_t pin, bool brake);

// Encoder ISR prototypes for the drive wheels
void encoder_ISR_fr();
void encoder_ISR_fl();
void encoder_ISR_br();
void encoder_ISR_bl();

// --- Callback for processing drive commands from "cmd_vel" ---
// Receives a geometry_msgs/Twist message.
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *) msgin;
  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;
  
  // Calculate wheel speeds (example scaling factors)
  int speed_fr = linear_x * 100 + angular_z * 75;
  int speed_fl = linear_x * 100 - angular_z * 75;
  int speed_br = linear_x * 100 + angular_z * 75;
  int speed_bl = linear_x * 100 - angular_z * 75;
  
  bool direction_fr = speed_fr != 0 ? (speed_fr < 0) : last_direction_fr;
  bool direction_fl = speed_fl != 0 ? (speed_fl >= 0) : last_direction_fl;
  bool direction_br = speed_br != 0 ? (speed_br < 0) : last_direction_br;
  bool direction_bl = speed_bl != 0 ? (speed_bl >= 0) : last_direction_bl;

  // Determine direction for each wheel
  // bool direction_fr = (speed_fr >= 0);
  // bool direction_fl = (speed_fl >= 0);
  // bool direction_br = (speed_br >= 0);
  // bool direction_bl = (speed_bl >= 0);
  
  setDirection(DIR_FR_PIN, direction_fr);
  setDirection(DIR_FL_PIN, direction_fl);
  setDirection(DIR_BR_PIN, direction_br);
  setDirection(DIR_BL_PIN, direction_bl);
  
  // Update last directions for encoder counting
  last_direction_fr = direction_fr;
  last_direction_fl = direction_fl;
  last_direction_br = direction_br;
  last_direction_bl = direction_bl;
  
  // Engage brakes if a wheel's speed is zero
  setBrake(BRAKE_FR_PIN, speed_fr == 0);
  setBrake(BRAKE_FL_PIN, speed_fl == 0);
  setBrake(BRAKE_BR_PIN, speed_br == 0);
  setBrake(BRAKE_BL_PIN, speed_bl == 0);
  
  // Set ESC PWM outputs (using absolute value of speed)
  setPWM_ESC(ESC_FR_PIN, abs(speed_fr));
  setPWM_ESC(ESC_FL_PIN, abs(speed_fl));
  setPWM_ESC(ESC_BR_PIN, abs(speed_br));
  setPWM_ESC(ESC_BL_PIN, abs(speed_bl));
  
  // Serial.print("cmd_vel: linear=");
  // Serial.print(linear_x);
  // Serial.print(" angular=");
  // Serial.println(angular_z);
}

// --- Timer callbacks for publishing wheel encoder data ---
// Each timer callback accepts (rcl_timer_t*, int64_t) as required.
void publish_wheel_encoder_fr(rcl_timer_t * timer, int64_t last_call_time) {
  (void) timer;
  (void) last_call_time;
  int32_t data = pulse_count_fr;
  int32_t data_array[1] = {data};
  encoder_msg_fr.data.data = data_array;
  encoder_msg_fr.data.size = 1;
  encoder_msg_fr.data.capacity = 1;
  rcl_publish(&encoder_pub_fr, &encoder_msg_fr, NULL);
}

void publish_wheel_encoder_fl(rcl_timer_t * timer, int64_t last_call_time) {
  (void) timer;
  (void) last_call_time;
  int32_t data = pulse_count_fl;
  int32_t data_array[1] = {data};
  encoder_msg_fl.data.data = data_array;
  encoder_msg_fl.data.size = 1;
  encoder_msg_fl.data.capacity = 1;
  rcl_publish(&encoder_pub_fl, &encoder_msg_fl, NULL);
}

void publish_wheel_encoder_br(rcl_timer_t * timer, int64_t last_call_time) {
  (void) timer;
  (void) last_call_time;
  int32_t data = pulse_count_br;
  int32_t data_array[1] = {data};
  encoder_msg_br.data.data = data_array;
  encoder_msg_br.data.size = 1;
  encoder_msg_br.data.capacity = 1;
  rcl_publish(&encoder_pub_br, &encoder_msg_br, NULL);
}

void publish_wheel_encoder_bl(rcl_timer_t * timer, int64_t last_call_time) {
  (void) timer;
  (void) last_call_time;
  int32_t data = pulse_count_bl;
  int32_t data_array[1] = {data};
  encoder_msg_bl.data.data = data_array;
  encoder_msg_bl.data.size = 1;
  encoder_msg_bl.data.capacity = 1;
  rcl_publish(&encoder_pub_bl, &encoder_msg_bl, NULL);
}

// --- Helper functions for ESC control ---
// Maps a speed (0–100) to a PWM value (0–255) and writes to the given pin.
void setPWM_ESC(uint8_t pin, int speed) {
  int pwm_value = map(speed, 0, 100, 0, 255);
  analogWrite(pin, pwm_value);
}

void setDirection(uint8_t pin, bool forward) {
  digitalWrite(pin, forward ? HIGH : LOW);
}

void setBrake(uint8_t pin, bool brake) {
  digitalWrite(pin, brake ? HIGH : LOW);
}

// --- Encoder Interrupt Service Routines for drive wheels ---
void encoder_ISR_fr() { pulse_count_fr += (last_direction_fr ? 1 : -1); }
void encoder_ISR_fl() { pulse_count_fl += (last_direction_fl ? 1 : -1); }
void encoder_ISR_br() { pulse_count_br += (last_direction_br ? 1 : -1); }
void encoder_ISR_bl() { pulse_count_bl += (last_direction_bl ? 1 : -1); }

void setup() {
  //Serial.begin(115200);
  //while (!Serial) { }

  // Initialize micro-ROS transport for Teensy.
  set_microros_transports();

  byte my_mac[]   = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
  IPAddress my_ip(192, 168, 1, 50); 
  IPAddress agent_ip(192, 168, 1, 100);

  Ethernet.init(10);               // CS pin for W5500 (adjust if yours is different)
  Ethernet.begin(my_mac, my_ip);   // bring up the link :contentReference[oaicite:0]{index=0}
  delay(500);

  set_microros_native_ethernet_udp_transports(
    my_mac,
    my_ip,
    agent_ip,
    8888
  );
  
  // ---- Drive System Initialization ----
  // Set ESC outputs
  pinMode(ESC_FR_PIN, OUTPUT);
  pinMode(ESC_FL_PIN, OUTPUT);
  pinMode(ESC_BR_PIN, OUTPUT);
  pinMode(ESC_BL_PIN, OUTPUT);
  // Set direction outputs
  pinMode(DIR_FR_PIN, OUTPUT);
  pinMode(DIR_FL_PIN, OUTPUT);
  pinMode(DIR_BR_PIN, OUTPUT);
  pinMode(DIR_BL_PIN, OUTPUT);
  // Set brake outputs
  pinMode(BRAKE_FR_PIN, OUTPUT);
  pinMode(BRAKE_FL_PIN, OUTPUT);
  pinMode(BRAKE_BR_PIN, OUTPUT);
  pinMode(BRAKE_BL_PIN, OUTPUT);
  // Initialize wheel encoder pins
  pinMode(ENC_FR_PIN, INPUT_PULLUP);
  pinMode(ENC_FL_PIN, INPUT_PULLUP);
  pinMode(ENC_BR_PIN, INPUT_PULLUP);
  pinMode(ENC_BL_PIN, INPUT_PULLUP);
  
  // Attach interrupts for wheel encoders
  attachInterrupt(digitalPinToInterrupt(ENC_FR_PIN), encoder_ISR_fr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FL_PIN), encoder_ISR_fl, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_BR_PIN), encoder_ISR_br, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_BL_PIN), encoder_ISR_bl, RISING);
  
  // ---- Micro-ROS Node Initialization ----
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "teensy_combined_node", "", &support);
  
  // Subscription for drive control Twist messages.
  rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  );
  
  // Publishers for wheel encoder data (one publisher per wheel)
  rclc_publisher_init_default(
    &encoder_pub_fr,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "encoder_data_fr"
  );
  rclc_publisher_init_default(
    &encoder_pub_fl,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "encoder_data_fl"
  );
  rclc_publisher_init_default(
    &encoder_pub_br,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "encoder_data_br"
  );
  rclc_publisher_init_default(
    &encoder_pub_bl,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "encoder_data_bl"
  );
  
  // Initialize timers for wheel encoder data publishing (every 100ms)
  rclc_timer_init_default(
    &timer_fr,
    &support,
    RCL_MS_TO_NS(100),
    publish_wheel_encoder_fr
  );
  rclc_timer_init_default(
    &timer_fl,
    &support,
    RCL_MS_TO_NS(100),
    publish_wheel_encoder_fl
  );
  rclc_timer_init_default(
    &timer_br,
    &support,
    RCL_MS_TO_NS(100),
    publish_wheel_encoder_br
  );
  rclc_timer_init_default(
    &timer_bl,
    &support,
    RCL_MS_TO_NS(100),
    publish_wheel_encoder_bl
  );
  
  // Initialize executor with 6 handles:
  rclc_executor_init(&executor, &support.context, 6, &allocator);
  rclc_executor_add_timer(&executor, &timer_fr);
  rclc_executor_add_timer(&executor, &timer_fl);
  rclc_executor_add_timer(&executor, &timer_br);
  rclc_executor_add_timer(&executor, &timer_bl);
}

void loop() {
  // Process incoming micro-ROS messages and timer events.
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  
}
