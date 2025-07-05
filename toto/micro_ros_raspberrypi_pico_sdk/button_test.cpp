#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define BUTTON_PIN 15

int main() {
    stdio_init_all();
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);
    
    while (true) {
        if(!gpio_get(BUTTON_PIN)) {
            printf("Button pressed\n");
        }

        sleep_ms(250);
    }
}
