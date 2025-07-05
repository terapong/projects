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

#include <std_msgs/msg/color_rgba.h>
std_msgs__msg__ColorRGBA led_color;

#define LED_RED 0
#define LED_GREEN 1
#define LED_BLUE 2

void subscription_callback(const void *msgin) {
  const std_msgs__msg__ColorRGBA *msg_toto = (const std_msgs__msg__ColorRGBA *)msgin;

  // Set LED colors (PWM for smooth transitions) gpio_put(LED_PIN_1, 1);
    gpio_put(LED_RED, (int)msg_toto->r);
    gpio_put(LED_GREEN, (int)msg_toto->g);
    gpio_put(LED_BLUE, (int)msg_toto->b);
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

    gpio_init(LED_RED); gpio_set_dir(LED_RED, GPIO_OUT);
    gpio_init(LED_GREEN); gpio_set_dir(LED_GREEN, GPIO_OUT);
    gpio_init(LED_BLUE); gpio_set_dir(LED_BLUE, GPIO_OUT);

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

    // Create subscriber  
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA),
        "led_color");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &led_color, &subscription_callback, ON_NEW_DATA);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}

//ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
//ros2 topic list
//ros2 topic echo /led_color
//# Set to red
//ros2 topic pub --once /led_color std_msgs/msg/ColorRGBA "{r: 1.0, g: 0.0, b: 0.0, a: 1.0}"

//# Set to green
//ros2 topic pub --once /led_color std_msgs/msg/ColorRGBA "{r: 0.0, g: 1.0, b: 0.0, a: 1.0}"

//# Set to blue
//ros2 topic pub --once /led_color std_msgs/msg/ColorRGBA "{r: 0.0, g: 0.0, b: 1.0, a: 1.0}"