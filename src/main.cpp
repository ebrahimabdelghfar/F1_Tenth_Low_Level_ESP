#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <uros_config.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/error_handling.h>
#include <brushless_control.h>
#include "pins_config.h"
#include "filter_lib.h"
#include "servo_control.h"
#include "pid_lib.h"
#include "servo_config.h"
#include "filter_config.h"
#include "esp32_config.h"
/* ============== Create Threads Safe Mutexes ============== */
SemaphoreHandle_t servoSetPointMutex = NULL;
SemaphoreHandle_t steeringAngleReadingMutex = NULL;
SemaphoreHandle_t throttleCommandMutex = NULL;

// ============== FreeRTOS Task Handles ==============
TaskHandle_t controlTaskHandle = NULL;

// ============== Publisher for steering angle feedback ==============
rcl_publisher_t steering_angle_publisher;

// ============== Subscribers ==============
rcl_subscription_t steering_command_subscriber;
rcl_subscription_t throttle_subscriber;

// ============== Core ROS structures ==============
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ============== Timers ==============
rcl_timer_t steering_feedback_timer;    // 20Hz publisher timer (runs on Core 1)

// ============== Executors ==============
rclc_executor_t pub_executor;
rclc_executor_t sub_executor;

// ============== Messages ==============
std_msgs__msg__Float32 steering_angle_msg;
std_msgs__msg__Float32 steering_command_msg;
std_msgs__msg__Float32 throttle_msg;

// ============== Control variables (volatile for cross-core access) ==============
volatile float steering_command_value = 0.0f;
volatile float throttle_command_value = 0.0f;
volatile float current_steering_angle = 0.0f;
BrushlessControl brushlessControl;
AngleFilter angleFilter;
PIDController pidController;
ServoControl servoControl;
ServoPID steeringPID;
// ============== Timing for control loop ==============
const TickType_t CONTROL_PERIOD_MS = 10;  // 100Hz control loop (10ms)

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// ============== Callback for /steering_command subscriber ==============
void steering_command_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  steering_command_value = msg->data;
}

// ============== Callback for /throttle subscriber ==============
void throttle_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  throttle_command_value = msg->data;
}

// ============== Timer callback for publishing steering angle feedback at 20Hz (Core 1) ==============
void steering_feedback_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    current_steering_angle = servoControl.getCurrentAngle(); // Read current angle from servo feedback
    current_steering_angle = angleFilter.update(current_steering_angle); // Apply hybrid filter to raw angle
    steering_angle_msg.data = current_steering_angle; // Update message with current angle
    RCSOFTCHECK(rcl_publish(&steering_angle_publisher, &steering_angle_msg, NULL)); // Publish feedback
  }
}

// ============== Control Task running on Core 0 ==============
// Handles both steering and brushless motor control at 10Hz
void controlTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  for (;;) {
    // Only run control when agent is connected
    if (state == AGENT_CONNECTED) {
      // ----- Steering Control (10Hz) -----
      // TODO: Implement steering control logic
      // Use steering_command_value to control the steering servo
      // Update current_steering_angle with feedback
      
      // ----- Brushless Motor Control (10Hz) -----
      // TODO: Implement brushless motor control logic
      // Use throttle_command_value to control the brushless motor
    } else {
      // Safety: Set motors to safe state when disconnected
      brushlessControl.stop();
      // TODO: Set steering to neutral, motor to stop
    }
    
    // Precise timing using vTaskDelayUntil for consistent 10Hz loop
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}


bool create_entities(void){
  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // Create publisher for /steering_angle
  RCCHECK(rclc_publisher_init_default(
    &steering_angle_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/steering_angle")
  );

  // Create subscription for /steering_command
  RCCHECK(rclc_subscription_init_default(
    &steering_command_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/steering_command")
  );

  // Create subscription for /throttle
  RCCHECK(rclc_subscription_init_default(
    &throttle_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/throttle")
  );

  // Create steering feedback timer at 20Hz (50ms period) - runs on Core 1
  const unsigned int feedback_timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(
    &steering_feedback_timer,
    &support,
    RCL_MS_TO_NS(feedback_timer_timeout),
    steering_feedback_timer_callback)
  );

  // Create executors (all run on Core 1 with micro-ROS)
  pub_executor = rclc_executor_get_zero_initialized_executor();
  sub_executor = rclc_executor_get_zero_initialized_executor();

  // Publisher executor: 1 feedback timer = 1 handle
  RCCHECK(rclc_executor_init(&pub_executor, &support.context, 1, &allocator));
  // Subscriber executor: 2 subscriptions = 2 handles
  RCCHECK(rclc_executor_init(&sub_executor, &support.context, 2, &allocator));

  // Add subscriptions to sub_executor
  RCCHECK(rclc_executor_add_subscription(&sub_executor, &steering_command_subscriber, &steering_command_msg, steering_command_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&sub_executor, &throttle_subscriber, &throttle_msg, throttle_callback, ON_NEW_DATA));

  // Add timer to pub_executor
  RCCHECK(rclc_executor_add_timer(&pub_executor, &steering_feedback_timer));

  return true;
}

void destroy_entities(){
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&steering_angle_publisher, &node);
  rcl_subscription_fini(&steering_command_subscriber, &node);
  rcl_subscription_fini(&throttle_subscriber, &node);

  rcl_timer_fini(&steering_feedback_timer);

  rclc_executor_fini(&pub_executor);
  rclc_executor_fini(&sub_executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  /*set ESP32 Configuration*/
  setCpuFrequencyMhz(ESP32_CLOCK_SPEED_MHZ);
  analogReadResolution(ANALOG_READ_RESOLUTION);
  analogSetAttenuation(ANALOG_ATTENUATION);
  // Configure serial transport
  Serial.begin(115200);
  while(!Serial); // Wait until Serial is ready
  set_microros_serial_transports(Serial);
  
  // TODO: Initialize steering servo
  servoControl.Servo_Init(STEERING_SERVO_PIN, STEERING_ANGLE_SENSOR_PIN);
  brushlessControl.init(BRUSHLESS_PWM_PIN); // Example: Initialize brushless motor on pin 5
  angleFilter.begin(0.001f, FILTER_Q_ANGLE, FILTER_Q_VEL, FILTER_R_MEAS); // default_dt=1ms, Q_angle=0.01, Q_vel=0.001, R_meas=0.1
  pidController.PID_Init(&steeringPID, SERVO_KP, SERVO_KI, SERVO_KD, INTEGRAL_WINDUP_GUARD,OUTPUT_LIMIT); // Example PID parameters
  // create mutexes for thread-safe variable access if needed 

  // Core 1 (APP_CPU) is used for micro-ROS communication (Arduino default)
  xTaskCreatePinnedToCore(
    controlTask,           // Task function
    "ControlTask",         // Task name
    4096,                  // Stack size (bytes)
    NULL,                  // Task parameters
    configMAX_PRIORITIES - 1, // Highest priority for fastest scheduling
    &controlTaskHandle,    // Task handle
    0                      // Pin to Core 0
  );
  
  state = WAITING_AGENT;
}

void loop() {
  // This loop runs on Core 1 - handles all micro-ROS communication
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        // Spin executors - handles pub/sub on Core 1
        RCSOFTCHECK(rclc_executor_spin_some(&pub_executor, RCL_MS_TO_NS(10)));
        RCSOFTCHECK(rclc_executor_spin_some(&sub_executor, RCL_MS_TO_NS(10)));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
  // Small delay to yield to other tasks and prevent watchdog issues
  vTaskDelay(pdMS_TO_TICKS(1));
}