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

#include <std_msgs/msg/int32.h>
std_msgs__msg__Int32 msg;

// #define LED_PIN_A 2
// #define LED_PIN_B 3
// #define LED_PIN_C 6
// #define LED_PIN_D 5
// #define LED_PIN_E 4
// #define LED_PIN_F 1
// #define LED_PIN_G 0
// #define LED_PIN_DP 7

// int HIGH = 1;
// int LOW = 0;

// void segment7(int _a, int _b, int _c, int _d, int _e, int _f, int _g, int _dp) {
//   gpio_put(LED_PIN_A, _a);
//   gpio_put(LED_PIN_B, _b);
//   gpio_put(LED_PIN_C, _c);
//   gpio_put(LED_PIN_D, _d);
//   gpio_put(LED_PIN_E, _e);
//   gpio_put(LED_PIN_F, _f);
//   gpio_put(LED_PIN_G, _g);
//   gpio_put(LED_PIN_DP, _dp);
// }

// void showDigit(int value) {
//   if (value == 0) {
//     segment7(HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, LOW, LOW);
//   } else if (value == 1) {
//     segment7(LOW, HIGH, HIGH, LOW, LOW, LOW, LOW, LOW);
//   } else if (value == 2) {
//     segment7(HIGH, HIGH, LOW, HIGH, HIGH, LOW, HIGH, LOW);
//   } else if (value == 3) {
//     segment7(HIGH, HIGH, HIGH, HIGH, LOW, LOW, HIGH, LOW);
//   } else if (value == 4) {
//     segment7(LOW, HIGH, HIGH, LOW, LOW, HIGH, HIGH, LOW);
//   } else if (value == 5) {
//     segment7(HIGH, LOW, HIGH, HIGH, LOW, HIGH, HIGH, LOW);
//   } else if (value == 6) {
//     segment7(HIGH, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, LOW);
//   } else if (value == 7) {
//     segment7(HIGH, HIGH, HIGH, LOW, LOW, LOW, LOW, LOW);
//   } else if (value == 8) {
//     segment7(HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, LOW);
//   } else if (value == 9) {
//     segment7(HIGH, HIGH, HIGH, HIGH, LOW, HIGH, HIGH, LOW);
//   } else {
//     segment7(LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH);
//   }
// }

// Define GPIO pins for the display
const uint segment_pins[] = {1, 5, 9, 7, 6, 2, 10, 8}; // A, B, C, D, E, F, G, DP
const uint digit_pins[] = {0, 3, 4, 11}; // Digit 1-4

// const uint segment_pins[] = {4, 5, 6, 7, 8, 9, 10, 11}; // A, B, C, D, E, F, G, DP
// const uint digit_pins[] = {0, 1, 2, 3}; // Digit 1-4

// Segment patterns for numbers 0-9
const uint8_t digit_patterns[10] = {
  0b00111111, // 0 77₈  = 63₁₀  = 3F₁₆
  0b00000110, // 1 6₈   = 6₁₀   = 6₁₆
  0b01011011, // 2 133₈ = 91₁₀  = 5B₁₆
  0b01001111, // 3 117₈ = 79₁₀  = 4F₁₆
  0b01100110, // 4 146₈ = 102₁₀ = 66₁₆
  0b01101101, // 5 155₈ = 109₁₀ = 6D₁₆
  0b01111101, // 6 175₈ = 125₁₀ = 7D₁₆
  0b00000111, // 7 7₈   = 7₁₀   = 7₁₆
  0b01111111, // 8 177₈ = 127₁₀ = 7F₁₆
  0b01101111  // 9 157₈ = 111₁₀ = 6F₁₆
    
};

// Variables for display multiplexing
int display_value = 0;
int current_digit = 0;
int digits[4] = {0};

void update_display() {
    // Turn off all digits
    for (int i = 0; i < 4; i++) {
        gpio_put(digit_pins[i], 0);
    }

    // Set segments for current digit
    uint8_t pattern = digit_patterns[digits[current_digit]]; //D1 -> D4
    for (int i = 0; i < 8; i++) {
      gpio_put(segment_pins[i], (pattern >> i) & 0x01); 
      // HIGH LOW >> cout << cint 0x01 = 0000 0001 = 1 เป็นเลข ทัาน 16 0xFF = 1111 1111 = 377
      //(pattern >> i) & 0x01) มันคือ Used in expressions to perform a bitwise AND on integers.
    }
    
    // Turn on current digit
    gpio_put(digit_pins[current_digit], 1);
    
    // Move to next digit
    current_digit = (current_digit + 1) % 4;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    (void)timer;
    update_display();
}

void subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    display_value = msg->data;
    
    // Split the value into individual digits
    int value = display_value;
    for (int i = 3; i >= 0; i--) {
        digits[i] = value % 10;
        value /= 10;
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
    
    // Initialize GPIO pins
    for (int i = 0; i < 8; i++) {
        gpio_init(segment_pins[i]);
        gpio_set_dir(segment_pins[i], GPIO_OUT);
    }
    for (int i = 0; i < 4; i++) {
        gpio_init(digit_pins[i]);
        gpio_set_dir(digit_pins[i], GPIO_OUT);
    }

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
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "display_value");
    
    // Create timer for display multiplexing
    rcl_timer_t timer;
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(5), // Update every 5ms
        timer_callback);
    
    // Create executor
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);
    
    // Main loop
    while (true) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
    
    return 0;
}