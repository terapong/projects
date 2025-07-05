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

#include <std_msgs/msg/string.h>
std_msgs__msg__String msg;

// #include <RtcDS1302.h>
// ThreeWire myWire(2, 15, 4);
// RtcDS1302<ThreeWire> Rtc(myWire);

int main() {
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

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
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "rtc_time");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    // rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}

// ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
// ros2 topic echo /rtc_time
// ros2 run otto_robot rtc_control_gui
// CONNECTIONS:
// DS1302 CLK/SCLK --> 15
// DS1302 DAT/IO --> 2
// DS1302 RST/CE --> 4
// DS1302 VCC --> 3.3v - 5v
// DS1302 GND --> GND