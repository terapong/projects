#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include "pico/stdlib.h"
#include "../pico_uart_transports.h"
#include <rmw_microros/rmw_microros.h>

rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;

rcl_timer_t timer;

#include "sensor_msgs/msg/range.h"
sensor_msgs__msg__Range range_msg;

// Ultrasonic sensor pins
#define TRIG_PIN 0
#define ECHO_PIN 1

// Timer callback for publishing sensor data
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer != NULL) {
        // Trigger the ultrasonic sensor
        gpio_put(TRIG_PIN, 1);
        sleep_us(10);
        gpio_put(TRIG_PIN, 0);
        
        // Wait for echo to go high
        while(gpio_get(ECHO_PIN) == 0);
        absolute_time_t start_time = get_absolute_time();
        
        // Wait for echo to go low
        while(gpio_get(ECHO_PIN) == 1);
        absolute_time_t end_time = get_absolute_time();
        
        // Calculate duration and distance
        int64_t duration_us = absolute_time_diff_us(start_time, end_time);
        float distance_cm = duration_us * 0.0343 / 2;  // Speed of sound in cm/us
        
        // Populate the ROS message
        range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
        range_msg.field_of_view = 0.1f;  // ~10 degree field of view
        range_msg.min_range = 0.02f;     // 2cm minimum
        range_msg.max_range = 4.0f;      // 4m maximum
        range_msg.range = distance_cm / 100.0f;  // Convert to meters
        
        // Publish the message
        rcl_publish(&publisher, &range_msg, NULL);
    }
}

int main() {
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    // Initialize GPIO for ultrasonic sensor
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    // Initialize message
    sensor_msgs__msg__Range__init(&range_msg);
    range_msg.header.frame_id.data = "ultrasonic_sensor";
    range_msg.header.frame_id.size = strlen("ultrasonic_sensor");

    // Set up Micro-ROS
    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ret != RCL_RET_OK) {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "otto_node", "", &support);
  
    // Create publisher
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "ultrasonic_distance");

    // Create timer
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    // Create executor
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        // Update LCD with message data if needed
    }

    // Cleanup
    rcl_publisher_fini(&publisher, &node);
    rcl_node_fini(&node);
    return 0;
}

//ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
//ros2 topic list
//ros2 topic echo /ultrasonic_distance
//ros2 topic list
//colcon build --symlink-install
//ros2 topic echo /ultrasonic_distance