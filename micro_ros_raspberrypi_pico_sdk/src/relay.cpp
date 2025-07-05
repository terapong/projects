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

#include <std_msgs/msg/bool.h>
std_msgs__msg__Bool msg;

#define LED_PIN 0

void subscription_callback(const void* msgin) {
    const std_msgs__msg__Bool * msg_toto = (const std_msgs__msg__Bool *)msgin;
    if (msg_toto->data) {
        gpio_put(LED_PIN, 1);
    } else {
        gpio_put(LED_PIN, 0);
    }
    // Publish the LED state
    msg.data = msg_toto->data;
    rcl_publish(&publisher, &msg, NULL); //มันส่งมาแล้วนี่หว่า
}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

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
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "relay_state");

    // Create subscriber
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "relay_command");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}

//ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
//ros2 topic list
//ros2 topic echo /relay_state
//ros2 topic pub --once /relay_command std_msgs/Bool "data: true"
//ros2 topic pub --once /relay_command std_msgs/Bool "data: false"
//ros2 run otto_robot relay_control_gui