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
#include "rtos_utils_lib.h"
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
rcl_timer_t steering_feedback_timer; // Publisher timer (runs on Core 1)

// ============== Executors ==============
rclc_executor_t ros_executor;

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
const TickType_t CONTROL_PERIOD_MS = 10; // 100Hz control loop (10ms period)
const uint32_t STEERING_FEEDBACK_PERIOD_MS = 10; // 100Hz feedback publish period
const int64_t EXECUTOR_SPIN_TIMEOUT_NS = 0;      // Non-blocking spin for minimum latency
const uint32_t WAITING_AGENT_PING_PERIOD_MS = 250;
const uint32_t CONNECTED_AGENT_PING_PERIOD_MS = 1000;
const uint32_t AGENT_PING_TIMEOUT_MS = 10;
const uint8_t AGENT_PING_ATTEMPTS = 1;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// ============== Callback for /steering_command subscriber ==============
void steering_command_callback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  logInMutex(&servoSetPointMutex, "SteeringCommandCallback");
  steering_command_value = msg->data;
  logOutMutex(&servoSetPointMutex, "SteeringCommandCallback");
}

// ============== Callback for /throttle subscriber ==============
void throttle_callback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  logInMutex(&throttleCommandMutex, "ThrottleCallback");
  throttle_command_value = msg->data;
  logOutMutex(&throttleCommandMutex, "ThrottleCallback");
}

// ============== Timer callback for publishing steering angle feedback at 20Hz (Core 1) ==============
void steering_feedback_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    logInMutex(&steeringAngleReadingMutex, "SteeringFeedbackTimerCallback");
    current_steering_angle = servoControl.getCurrentAngle();                        // Read current angle from servo feedback
    current_steering_angle = angleFilter.update(current_steering_angle);            // Apply hybrid filter to raw angle
    current_steering_angle = servoControl.constrainAngle(current_steering_angle);   // Constrain angle to [-180, 180]
    steering_angle_msg.data = current_steering_angle-ANGLE_OFFSET;                               // Update message with current angle
    RCSOFTCHECK(rcl_publish(&steering_angle_publisher, &steering_angle_msg, NULL)); // Publish feedback
    logOutMutex(&steeringAngleReadingMutex, "SteeringFeedbackTimerCallback");
  }
}

// ============== Control Task running on Core 0 ==============
// Handles both steering and brushless motor control at 100Hz
void controlTask(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    // Only run control when agent is connected
    if (state == AGENT_CONNECTED) {
      // ----- Steering Control (100Hz) -----
      logInMutex(&servoSetPointMutex, "ControlTask");
      float raw_angle = servoControl.getCurrentAngle();
      float filtered_angle = angleFilter.update(raw_angle);
      filtered_angle = servoControl.constrainAngle(filtered_angle)-ANGLE_OFFSET; // Constrain to [-180, 180] and apply offset
      float pwm_offset = pidController.PID_Compute(&steeringPID, steering_command_value, filtered_angle);
      servoControl.setServoPulseWidth(pwm_offset);
      logOutMutex(&servoSetPointMutex, "ControlTask");
    
      logInMutex(&throttleCommandMutex, "ControlTask");
      brushlessControl.set_throttle(throttle_command_value);
      logOutMutex(&throttleCommandMutex, "ControlTask");
      

    } else {
      // Safety: Set motors to safe state when disconnected
      logInMutex(&throttleCommandMutex, "ControlTask");
      brushlessControl.stop();
      servoControl.setServoPulseWidth(0.0f); // Set steering to neutral
      logOutMutex(&throttleCommandMutex, "ControlTask");
    }
    // Precise timing using vTaskDelayUntil for consistent 100Hz loop
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

bool create_entities(void)
{
  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // Create publisher for /steering_angle
    RCCHECK(rclc_publisher_init_best_effort(
      &steering_angle_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/steering_angle"));

  // Create subscription for /steering_command
    RCCHECK(rclc_subscription_init_best_effort(
      &steering_command_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/steering_command"));

  // Create subscription for /throttle
    RCCHECK(rclc_subscription_init_best_effort(
      &throttle_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/throttle"));

    // Create steering feedback timer at 100Hz (10ms period) - runs on Core 1
  RCCHECK(rclc_timer_init_default(
      &steering_feedback_timer,
      &support,
      RCL_MS_TO_NS(STEERING_FEEDBACK_PERIOD_MS),
      steering_feedback_timer_callback));

    // Create executor (runs on Core 1 with micro-ROS)
    ros_executor = rclc_executor_get_zero_initialized_executor();

    // 1 timer + 2 subscriptions
    RCCHECK(rclc_executor_init(&ros_executor, &support.context, 3, &allocator));

    RCCHECK(rclc_executor_add_subscription(&ros_executor, &steering_command_subscriber, &steering_command_msg, steering_command_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&ros_executor, &throttle_subscriber, &throttle_msg, throttle_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&ros_executor, &steering_feedback_timer));

  return true;
}

void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&steering_angle_publisher, &node);
  rcl_subscription_fini(&steering_command_subscriber, &node);
  rcl_subscription_fini(&throttle_subscriber, &node);

  rcl_timer_fini(&steering_feedback_timer);

  rclc_executor_fini(&ros_executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup()
{
  /*set ESP32 Configuration*/
  setCpuFrequencyMhz(ESP32_CLOCK_SPEED_MHZ);
  analogReadResolution(ANALOG_READ_RESOLUTION);
  analogSetAttenuation(ANALOG_ATTENUATION);
  // Configure serial transport
  Serial.begin(115200);
  while (!Serial)
    ; // Wait until Serial is ready
  set_microros_serial_transports(Serial);

  // TODO: Initialize steering servo
  servoControl.Servo_Init(STEERING_SERVO_PIN, STEERING_ANGLE_SENSOR_PIN);
  brushlessControl.init(BRUSHLESS_PWM_PIN);                                                                // Example: Initialize brushless motor on pin 5
  angleFilter.begin(0.001f, FILTER_Q_ANGLE, FILTER_Q_VEL, FILTER_R_MEAS);                                  // default_dt=1ms, Q_angle=0.01, Q_vel=0.001, R_meas=0.1
  pidController.PID_Init(&steeringPID, SERVO_KP, SERVO_KI, SERVO_KD, INTEGRAL_WINDUP_GUARD, OUTPUT_LIMIT); // Example PID parameters
  // create mutexes for thread-safe variable access if needed
  servoSetPointMutex = xSemaphoreCreateMutex();
  steeringAngleReadingMutex = xSemaphoreCreateMutex();
  throttleCommandMutex = xSemaphoreCreateMutex();
  // Core 1 (APP_CPU) is used for micro-ROS communication (Arduino default)
  xTaskCreatePinnedToCore(
      controlTask,              // Task function
      "ControlTask",            // Task name
      4096,                     // Stack size (bytes)
      NULL,                     // Task parameters
      configMAX_PRIORITIES - 1, // Highest priority for fastest scheduling
      &controlTaskHandle,       // Task handle
      0                         // Pin to Core 0
  );

  state = WAITING_AGENT;
}

void loop()
{
  // This loop runs on Core 1 - handles all micro-ROS communication
  switch (state)
  {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(WAITING_AGENT_PING_PERIOD_MS, state = (RMW_RET_OK == rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT)
    {
      destroy_entities();
    };
    break;
  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(CONNECTED_AGENT_PING_PERIOD_MS, state = (RMW_RET_OK == rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED)
    {
      // Spin executor in non-blocking mode for low-latency pub/sub handling
      RCSOFTCHECK(rclc_executor_spin_some(&ros_executor, EXECUTOR_SPIN_TIMEOUT_NS));
    }
    break;
  case AGENT_DISCONNECTED:
    destroy_entities();
    state = WAITING_AGENT;
    break;
  default:
    break;
  }
  // Yield CPU without adding fixed millisecond latency on each loop
  taskYIELD();
}