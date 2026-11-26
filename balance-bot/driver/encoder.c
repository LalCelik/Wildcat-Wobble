#include "encoder.h"
#include <stdio.h>
#include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "microbit_v2.h"

// #define ENC_LEFT_A  EDGE_P0
// #define ENC_LEFT_B  EDGE_P1
// #define ENC_RIGHT_A EDGE_P2
// #define ENC_RIGHT_B EDGE_P8

#define ENC_LEFT_A  EDGE_P1
#define ENC_LEFT_B  EDGE_P2
#define ENC_RIGHT_A EDGE_P3
#define ENC_RIGHT_B EDGE_P9

// direction the wheels are turning
// had to flip one: the wheels face opposite ways, so their counts are opposite
#define ENC_LEFT_DIR  -1  // flipped so +u gives +x (matches the sim)
#define ENC_RIGHT_DIR 1

// #define ENC_LEFT_DIR  -1
// #define ENC_RIGHT_DIR 1

// 6 counts per motor turn * 29.86 gear ratio (Pololu #5158, 30:1 HP)
// (old motor: 6 * 51.45 = 308.7)
// gear box turns 29.86 for 1 wheel turn (from datasheet)
// 12 / 2 
#define COUNTS_PER_REV 179.2f

// hwo far the wheels have tuned in steps
static volatile int32_t left_count = 0;
static volatile int32_t right_count = 0;

static void encoder_handler(nrfx_gpiote_pin_t triggered_pin, nrf_gpiote_polarity_t action) {
    // which gpio pin just changed? 
    if (triggered_pin == ENC_LEFT_A) { // left wheel turned
        if (nrf_gpio_pin_read(ENC_LEFT_A) == nrf_gpio_pin_read(ENC_LEFT_B)) {
            // one step backwards
            left_count--;
        } else {
            // one step forwards
            left_count++;
        }
    } else if (triggered_pin == ENC_RIGHT_A) { // right wheel turned
        if (nrf_gpio_pin_read(ENC_RIGHT_A) == nrf_gpio_pin_read(ENC_RIGHT_B)) {
            right_count--;
        } else {
            right_count++;
        }
    }
}

void encoder_init(void) {
    if (!nrfx_gpiote_is_init()) {
        nrfx_gpiote_init();
    }

    nrf_gpio_cfg_input(ENC_LEFT_B, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(ENC_RIGHT_B, NRF_GPIO_PIN_PULLUP);

    // calls the encoder handler on rising and falling edges of A
    nrfx_gpiote_in_config_t config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    config.pull = NRF_GPIO_PIN_PULLUP;

    nrfx_gpiote_in_init(ENC_LEFT_A, &config, encoder_handler);
    nrfx_gpiote_in_init(ENC_RIGHT_A, &config, encoder_handler);
    nrfx_gpiote_in_event_enable(ENC_LEFT_A, true);
    nrfx_gpiote_in_event_enable(ENC_RIGHT_A, true);
}

float encoder_position(void) {
    // the count / counts per revolution
    // how much the wheel has turned 
    float left = ENC_LEFT_DIR * left_count / COUNTS_PER_REV;
    float right = ENC_RIGHT_DIR * right_count / COUNTS_PER_REV;
    return (left + right) / 2.0f;
}

// DEBUG: raw pin levels and tick counts for each wheel
void encoder_print_raw(void) {
    printf("L: A=%lu B=%lu cnt=%ld | R: A=%lu B=%lu cnt=%ld\n",
           (unsigned long)nrf_gpio_pin_read(ENC_LEFT_A), (unsigned long)nrf_gpio_pin_read(ENC_LEFT_B), (long)left_count,
           (unsigned long)nrf_gpio_pin_read(ENC_RIGHT_A), (unsigned long)nrf_gpio_pin_read(ENC_RIGHT_B), (long)right_count);
}
