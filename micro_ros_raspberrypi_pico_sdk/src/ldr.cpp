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

#include "hardware/adc.h"
#include <std_msgs/msg/int32.h>
std_msgs__msg__Int32 msg;

#define LDR_PIN 0

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        // Read ADC value (0-4095 for 12-bit ADC)
        uint16_t raw = adc_read();
        msg.data = (int32_t)raw;
        
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
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
    adc_gpio_init(26);  // Use GP26 (ADC0)
    adc_select_input(0);

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
        "ldr_value");

    //Create timer (publishes every 500ms)
    const unsigned int timer_timeout = 100;  // 100ms
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback);

    // Create executor Optional: send a startup message
    msg.data = -1;
    rcl_publish(&publisher, &msg, NULL);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    
    msg.data = 0;

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    // Cleanup
    rcl_publisher_fini(&publisher, &node);
    rcl_node_fini(&node);

    return 0;
}

//ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
//colcon build --symlink-install
//ros2 topic list
//ros2 topic echo /ldr_value
//ros2 run button_tester ros2 run button_tester ldr_control_gui