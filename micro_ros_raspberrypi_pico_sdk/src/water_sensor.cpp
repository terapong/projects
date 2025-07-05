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

#include <std_msgs/msg/int32.h>
std_msgs__msg__Int32 water_level_msg;

#include "hardware/adc.h"

#define WATER_SENSOR_PIN 0

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        int adc_val = adc_read();  // Read from ADC pin
        water_level_msg.data = adc_val;
        rcl_publish(&publisher, &water_level_msg, NULL);
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

    // Initialize ADC
    adc_init();
    adc_gpio_init(WATER_SENSOR_PIN);  // GP26 = ADC0
    adc_select_input(WATER_SENSOR_PIN);

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
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "water_level");

    // Create timer
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    // Create executor
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}

//ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
//ros2 topic list
//ros2 topic echo /water_level
//ros2 run otto_robot water_sensor_control_gui