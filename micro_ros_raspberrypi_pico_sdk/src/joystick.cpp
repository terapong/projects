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

#include <sensor_msgs/msg/joy.h>
sensor_msgs__msg__Joy joy_msg;

#include "hardware/adc.h"

#define JOYSTICK_X_ADC 0      // GP26
#define JOYSTICK_Y_ADC 1      // GP27
#define JOYSTICK_BUTTON_GPIO 22

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer != NULL) {
        // Read joystick values
        adc_select_input(JOYSTICK_X_ADC);
        float x = adc_read() / 4095.0f * 2.0f - 1.0f; // Convert to -1.0 to 1.0
        
        adc_select_input(JOYSTICK_Y_ADC);
        float y = adc_read() / 4095.0f * 2.0f - 1.0f; // Convert to -1.0 to 1.0
        
        bool button = !gpio_get(JOYSTICK_BUTTON_GPIO); // Assuming active low
        
        // Prepare message
        joy_msg.axes.data[0] = x;
        joy_msg.axes.data[1] = y;
        joy_msg.buttons.data[0] = button ? 1 : 0;
        
        // Publish message
        rcl_ret_t ret = rcl_publish(&publisher, &joy_msg, NULL);
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

    // Initialize ADC for joystick
    adc_init();
    adc_gpio_init(26); // X axis
    adc_gpio_init(27); // Y axis
    
    // Initialize button GPIO
    gpio_init(JOYSTICK_BUTTON_GPIO);
    gpio_set_dir(JOYSTICK_BUTTON_GPIO, GPIO_IN);
    gpio_pull_up(JOYSTICK_BUTTON_GPIO);

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
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
        "joystick_input");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100), // 100ms
        timer_callback);

    // Create executor
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    
    // Initialize joy message
    joy_msg.axes.data = (float*)malloc(2 * sizeof(float));
    joy_msg.axes.size = 2;
    joy_msg.buttons.data = (int32_t*)malloc(1 * sizeof(int32_t));
    joy_msg.buttons.size = 1;

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    // Cleanup (though we'll never get here)
    free(joy_msg.axes.data);
    free(joy_msg.buttons.data);
    return 0;
}

//ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
//ros2 topic list
//ros2 topic echo /joy
//ros2 run otto_robot joystick_control_gui 