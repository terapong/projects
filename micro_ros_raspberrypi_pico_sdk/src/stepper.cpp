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
// std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 pub_msg;
std_msgs__msg__Int32 sub_msg;

// Stepper motor pins
const uint IN1 = 0;
const uint IN2 = 1;
const uint IN3 = 2;
const uint IN4 = 3;

// Stepper motor sequence
const uint8_t step_sequence[8] = {
    0b0001, //1
    0b0011, //3
    0b0010, //2
    0b0110, //6
    0b0100, //4
    0b1100, //12
    0b1000, //8
    0b1001  //9
};

void stepper_step(uint steps, int direction) {
    static uint step_index = 0;
    
    for(uint i = 0; i < steps; i++) {
        if(direction == 1) {
            step_index = (step_index + 1) % 8;
        } else {
            step_index = (step_index - 1) % 8;
        }
        
        gpio_put(IN1, step_sequence[step_index] & 0b0001);
        gpio_put(IN2, step_sequence[step_index] & 0b0010);
        gpio_put(IN3, step_sequence[step_index] & 0b0100);
        gpio_put(IN4, step_sequence[step_index] & 0b1000);
        
        sleep_ms(2); // Adjust for speed
    }
}

void subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    
    int steps = msg->data;
    int direction = steps > 0 ? 1 : -1;
    steps = abs(steps);
    
    stepper_step(steps, direction);
    
    // Publish feedback
    pub_msg.data = steps * direction;
    rcl_publish(&publisher, &pub_msg, NULL);
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

    // Initialize GPIO
    gpio_init(IN1);
    gpio_init(IN2);
    gpio_init(IN3);
    gpio_init(IN4);
    gpio_set_dir(IN1, GPIO_OUT);
    gpio_set_dir(IN2, GPIO_OUT);
    gpio_set_dir(IN3, GPIO_OUT);
    gpio_set_dir(IN4, GPIO_OUT);

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
        "stepper_feedback");

    // Create subscriber
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "stepper_control");

    // rclc_executor_init(&executor, &support.context, 1, &allocator);
    // rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA);
    
    pub_msg.data = 0;

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    // Cleanup (unreachable in this example)
    rcl_publisher_fini(&publisher, &node);
    rcl_subscription_fini(&subscriber, &node);
    rcl_node_fini(&node);
    return 0;
}

//ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
//colcon build --symlink-install
//ros2 topic list
// # In a new terminal, send commands to the stepper
// ros2 topic pub /stepper_control std_msgs/msg/Int32 "{data: 512}"  # 512 steps forward
// ros2 topic pub /stepper_control std_msgs/msg/Int32 "{data: -256}" # 256 steps backward

// # View feedback
// ros2 topic echo /stepper_feedback
//ros2 run button_tester buzzer_control_gui