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
std_msgs__msg__Int32 msg;

#include "hardware/pwm.h"

#define SERVO_PIN 0

void pwm_init() {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 64.f);
    pwm_config_set_wrap(&config, 39062.f); // 50Hz frequency
    
    pwm_init(slice_num, &config, true);
}

void set_servo_angle(int angle) {
    // Convert angle (0-180) to pulse width (1000-2000µs)
    float pulse_width = 1000 + (angle / 180.0) * 1000;
    float wrap = 39062.f; // Corresponds to 20ms period
    float level = (pulse_width / 20000.f) * wrap;
    
    pwm_set_gpio_level(SERVO_PIN, (uint16_t)level);
}

void subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    printf("Received angle: %d\n", msg->data);
    set_servo_angle(msg->data);
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

    gpio_init(SERVO_PIN);
    gpio_set_dir(SERVO_PIN, GPIO_OUT);

    pwm_init();

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

    // // Create publisher
    // rclc_publisher_init_default(
    //     &publisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    //     "led_state");

    // Create subscriber
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "servo_angle");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    rcl_subscription_fini(&subscriber, &node);
    rcl_node_fini(&node);
    return 0;
}

//ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
//ros2 topic list
//ros2 topic echo /servo_angle
//ros2 topic pub --once /servo_angle  std_msgs/Int32 "data: 100"
//ros2 run otto_robot servo_control_gui