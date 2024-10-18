//Get readings from ultrasonic sensor

#include "ultrasonic.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"

static uint trigger_pin;
static uint echo_pin;

void hcsr04_init(uint trigger_pin_param, uint echo_pin_param) {
    trigger_pin = trigger_pin_param;
    echo_pin = echo_pin_param;
    
    gpio_init(trigger_pin);
    gpio_set_dir(trigger_pin, GPIO_OUT);
    gpio_put(trigger_pin, 0);

    gpio_init(echo_pin);
    gpio_set_dir(echo_pin, GPIO_IN);
}

uint32_t hcsr04_send_pulse_and_wait(uint echo_pin) {
    gpio_put(trigger_pin, 0); // Stabilize the sensor
    sleep_us(5);
    gpio_put(trigger_pin, 1);
    sleep_us(10); // Send a 10us pulse
    gpio_put(trigger_pin, 0);

    uint32_t pulse_time = 0;
    while (gpio_get(echo_pin) == 0) {
        if (pulse_time++ >= ECHO_TIMEOUT_US) {
            return ECHO_TIMEOUT_US;
        }
        sleep_us(1);
    }

    pulse_time = 0;
    while (gpio_get(echo_pin) == 1) {
        if (pulse_time++ >= ECHO_TIMEOUT_US) {
            return ECHO_TIMEOUT_US;
        }
        sleep_us(1);
    }

    return pulse_time;
}

float hcsr04_distance_mm(void) {
    uint32_t pulse_time = hcsr04_send_pulse_and_wait(echo_pin);

    if (pulse_time >= ECHO_TIMEOUT_US) {
        return -1; // Out of range
    }

    // Calculate distance in mm
    return (pulse_time * 100.0f) / 582.0f;
}

float hcsr04_distance_cm(void) {
    uint32_t pulse_time = hcsr04_send_pulse_and_wait(echo_pin);

    printf("Pulse Time: %d us\n", pulse_time);

    if (pulse_time >= ECHO_TIMEOUT_US) {
        return -1; // Out of range
    }

    // Calcular distância em cm
    return (pulse_time / 2.0f) / 29.1f;
}


