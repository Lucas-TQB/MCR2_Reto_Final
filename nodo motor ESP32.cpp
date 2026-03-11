#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>

// --- Hardware Pinout ---
#define PHASEA_GPIO 16
#define PHASEB_GPIO 17
#define IN1_GPIO    5  
#define IN2_GPIO    2  
#define PWM_GPIO    15 

// --- PID and Motor Constants ---
const float PPR_GEAR = 495.0; 
const float RPM_FULL_SCALE = 140.0;

// PID gains
float Kp = 2.4701;
float Ki = 0.014;
float Kd = 2.8;

// Error terms (matching your difference equation style)
float et0  = 0; // e[k]
float et_1 = 0; // e[k-1]
float et_2 = 0; // e[k-2]

float setpoint_rpm = 0.0;
float current_rpm  = 0.0;
volatile int pulses = 0;

// --- micro-ROS Objects ---
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg_sub;
std_msgs__msg__Float32 msg_pub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// --- ISR ---
void IRAM_ATTR isr() {
  if(digitalRead(PHASEB_GPIO) == HIGH) {
    pulses++;
  } else {
    pulses--;
  }
}

// --- PID Difference Equation Function ---
float accionControlPID(float Ts_ms) {
  static float controlVariable = 0; 

  /*
    Discrete PID Difference Equation:
    u[k] = u[k-1] + (Kp + Kd/Ts)e[k] + (-Kp + Ki*Ts - 2Kd/Ts)e[k-1] + (Kd/Ts)e[k-2]
  */
  controlVariable += (Kp + Kd / Ts_ms) * et0;
  controlVariable += (-Kp + Ki * Ts_ms - 2.0 * Kd / Ts_ms) * et_1;
  controlVariable += (Kd / Ts_ms) * et_2;

  // Clamp to 8-bit PWM range (0-255) for the motor driver
  if (controlVariable > 255.0) controlVariable = 255.0;
  if (controlVariable < -255.0) controlVariable = -255.0;

  return controlVariable;
}

// --- micro-ROS Callback ---
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  if (msg != NULL) {
    // Input is [-1, 1], scale to RPM
    setpoint_rpm = msg->data * RPM_FULL_SCALE;
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(IN1_GPIO, OUTPUT);
  pinMode(IN2_GPIO, OUTPUT);
  pinMode(PHASEA_GPIO, INPUT_PULLUP);
  pinMode(PHASEB_GPIO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PHASEA_GPIO), isr, FALLING);

  ledcAttach(PWM_GPIO, 1000, 8); // 1kHz, 8-bit

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "motor_node", "", &support); 

  rclc_subscription_init_default(
    &subscriber, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "cmd_pwm");

  rclc_publisher_init_default(
    &publisher, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor_telemetry");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA);
}

void loop() {
  // Allow micro-ROS to process incoming messages
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

  static unsigned long last_time = 0;
  const unsigned long sample_time_ms = 100;

  if (millis() - last_time >= sample_time_ms) {
    unsigned long now = millis();
    float dt = (now - last_time) / 1000.0;
    
    // 1. Calculate RPM (Atomic read not strictly required at 10Hz but good practice)
    noInterrupts();
    int pulses_copy = pulses;
    pulses = 0;
    interrupts();

    current_rpm = (pulses_copy * (1.0 / dt) * 60.0) / PPR_GEAR;

    // 2. Prepare Errors for Difference Equation
    et_2 = et_1;
    et_1 = et0;
    et0  = setpoint_rpm - current_rpm;

    // 3. Compute PID Output
    float output = accionControlPID((float)sample_time_ms);

    // 4. Actuate Motor Driver
    int pwm_val = constrain(abs((int)output), 0, 255);
    if (output >= 0) {
      digitalWrite(IN1_GPIO, HIGH);
      digitalWrite(IN2_GPIO, LOW);
    } else {
      digitalWrite(IN1_GPIO, LOW);
      digitalWrite(IN2_GPIO, HIGH);
    }
    ledcWrite(PWM_GPIO, pwm_val);

    // 5. Publish Telemetry
    msg_pub.data = current_rpm;
    rcl_publish(&publisher, &msg_pub, NULL);
    
    last_time = now;
  }
}