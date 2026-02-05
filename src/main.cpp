#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <uros_config.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/error_handling.h>

// Publisher for steering angle feedback
rcl_publisher_t steering_angle_publisher;

// Subscribers
rcl_subscription_t steering_command_subscriber;
rcl_subscription_t throttle_subscriber;

// Core ROS structures
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Timers
rcl_timer_t steering_feedback_timer;    // 20Hz publisher timer
rcl_timer_t steering_control_timer;      // 10Hz steering control
rcl_timer_t brushless_control_timer;     // 10Hz brushless control

// Executors
rclc_executor_t pub_executor;
rclc_executor_t sub_executor;

// Messages
std_msgs__msg__Float32 steering_angle_msg;
std_msgs__msg__Float32 steering_command_msg;
std_msgs__msg__Float32 throttle_msg;

// Control variables
float steering_command_value = 0.0f;
float throttle_value = 0.0f;
float current_steering_angle = 0.0f;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Callback for /steering_command subscriber
void steering_command_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  steering_command_value = msg->data;
}

// Callback for /throttle subscriber
void throttle_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  throttle_value = msg->data;
}

// Timer callback for publishing steering angle feedback at 20Hz
void steering_feedback_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // TODO: Get actual steering angle from sensor/encoder
    steering_angle_msg.data = current_steering_angle;
    RCSOFTCHECK(rcl_publish(&steering_angle_publisher, &steering_angle_msg, NULL));
  }
}

// Timer callback for steering control at 10Hz
void steering_control_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // TODO: Implement steering control logic
    // Use steering_command_value to control the steering servo
    // Update current_steering_angle with feedback
  }
}

// Timer callback for brushless motor control at 10Hz
void brushless_control_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // TODO: Implement brushless motor control logic
    // Use throttle_value to control the brushless motor
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

  // Create steering feedback timer at 20Hz (50ms period)
  const unsigned int feedback_timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(
    &steering_feedback_timer,
    &support,
    RCL_MS_TO_NS(feedback_timer_timeout),
    steering_feedback_timer_callback)
  );

  // Create steering control timer at 10Hz (100ms period)
  const unsigned int steering_control_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &steering_control_timer,
    &support,
    RCL_MS_TO_NS(steering_control_timeout),
    steering_control_timer_callback)
  );

  // Create brushless control timer at 10Hz (100ms period)
  const unsigned int brushless_control_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &brushless_control_timer,
    &support,
    RCL_MS_TO_NS(brushless_control_timeout),
    brushless_control_timer_callback)
  );

  // Create executors
  pub_executor = rclc_executor_get_zero_initialized_executor();
  sub_executor = rclc_executor_get_zero_initialized_executor();

  // Publisher executor: 1 feedback timer + 2 control timers = 3 handles
  RCCHECK(rclc_executor_init(&pub_executor, &support.context, 3, &allocator));
  // Subscriber executor: 2 subscriptions = 2 handles
  RCCHECK(rclc_executor_init(&sub_executor, &support.context, 2, &allocator));

  // Add subscriptions to sub_executor
  RCCHECK(rclc_executor_add_subscription(&sub_executor, &steering_command_subscriber, &steering_command_msg, steering_command_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&sub_executor, &throttle_subscriber, &throttle_msg, throttle_callback, ON_NEW_DATA));

  // Add timers to pub_executor
  RCCHECK(rclc_executor_add_timer(&pub_executor, &steering_feedback_timer));
  RCCHECK(rclc_executor_add_timer(&pub_executor, &steering_control_timer));
  RCCHECK(rclc_executor_add_timer(&pub_executor, &brushless_control_timer));

  return true;
}

void destroy_entities(){
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&steering_angle_publisher, &node);
  rcl_subscription_fini(&steering_command_subscriber, &node);
  rcl_subscription_fini(&throttle_subscriber, &node);

  rcl_timer_fini(&steering_feedback_timer);
  rcl_timer_fini(&steering_control_timer);
  rcl_timer_fini(&brushless_control_timer);

  rclc_executor_fini(&pub_executor);
  rclc_executor_fini(&sub_executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  while(!Serial); // Wait until Serial is ready
  set_microros_serial_transports(Serial);
  
  // TODO: Initialize steering servo
  // TODO: Initialize brushless motor controller
  
  state = WAITING_AGENT;
}

void loop() {
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
        RCSOFTCHECK(rclc_executor_spin_some(&pub_executor, RCL_MS_TO_NS(10)));
        RCSOFTCHECK(rclc_executor_spin_some(&sub_executor, RCL_MS_TO_NS(20)));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
}