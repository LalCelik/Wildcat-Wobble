// Motor driver PWM and direction to spin the wheels

#include "motor.h"

#define AIN1 EDGE_P15
#define AIN2 EDGE_P16

#define BIN1 EDGE_P13
#define BIN2 EDGE_P14

#define nSLEEP EDGE_P8

    void motor_init(void) {
        // initialize motor pins

        // SLEEP
        // set to 0 to turn off motor driver
        // set to 1 to turn on motor driver 
        nrf_gpio_cfg_output(nSLEEP);

        nrf_gpio_cfg_output(AIN1);
        nrf_gpio_cfg_output(AIN2);
        nrf_gpio_cfg_output(BIN1);
        nrf_gpio_cfg_output(BIN2);

        static const nrfx_pwm_t PWM_INST = NRFX_PWM_INSTANCE(0);

        // Holds duty cycle values to trigger PWM toggle
        // creat eplac ei nmemory for PWM value
        static nrf_pwm_values_individual_t sequence_data[1] = {0};

        nrf_pwm_sequence_t pwm_sequence = {
            //
            .values.p_individual = sequence_data, // pointer to mem addr of sequent data
            .length = 4,
            .repeats = 0, // PWM periods to repeat each value
            .end_delay = 0, // PWM periods to wait after sequence ends
        };

        // Sequence structure for configuring
        nrfx_pwm_config_t pwm_config = {
        .output_pins = {AIN1, AIN2, BIN1, BIN2},
        .base_clock = NRF_PWM_CLK_500kHz, //500kHz
        .count_mode = NRF_PWM_MODE_UP,
        // period is 500 × 2 µs = 1 ms
        // so PWM run at 1 kHz 
        .top_value = 500, // where the counter resets 
        .load_mode = NRF_PWM_LOAD_INDIVIDUAL, //gives the 4 pins their own pwm value
        .step_mode = NRF_PWM_STEP_AUTO, //gives the 4 pins their own duty value
        };
    }

    static void slow_decay(uint16_t comp) {
        // forwards or backwards motion:
        if (u > 0) { 
            // forwards
            //store counter value in pwm sequence
            sequence_data[0].channel_0 = 1 // AIN1
            sequence_data[0].channel_1 = (uint16_t)comp | (1<<15);  //AIN2
            sequence_data[0].channel_2 = 1 //BIN1
            sequence_data[0].channel_3 = (uint16_t)comp | (1<<15);  //BIN2
        } elif(u == 0) {
            // still (coasting)
            sequence_data[0].channel_0 = (uint16_t)comp | (1<<15);
            sequence_data[0].channel_1 = 0 | (1<<15);
            sequence_data[0].channel_2 = (uint16_t)comp | (1<<15);
            sequence_data[0].channel_3 = 0 | (1<<15);    
        } else {
            // backwards
            sequence_data[0].channel_0 = (uint16_t)comp | (1<<15);
            sequence_data[0].channel_1 = 1 | (1<<15);
            sequence_data[0].channel_2 = (uint16_t)comp | (1<<15);
            sequence_data[0].channel_3 = 1 | (1<<15);    
        }
    }

    static void pwm_init(void) {
    // Initialize the PWM
    // This time EDGE_P0 should be your output pin
    // Set the clock to 500 kHz, count mode to Up, and load mode to Common
    // This time the COUNTERTOP value DOES matter. It should be configured for a 20 ms period.
    nrfx_pwm_init(&PWM_INST, &pwm_config, NULL);
    }

    // Takes value to set the motor to 
    static void set_motor(float u, float u_max) {

        nrfx_pwm_stop(&PWM_INST, true);
        nrf_gpio_pin_write(nSLEEP, 1);

        // find duty cycle
        // what percent of the duty cycle the u represents
        // what percentage of duty cycle should this value be high
        ratio = (float) abs(u) / u_max
        duty_cycle = min(ratio, 1) // can't be less than 1

        // convert duty cycle into computer comp value
        // high 60% of the time -> 60% of PWM counter ticks
        float comp = pwm_config.top_value * duty_cycle;

        // forwards or backwards motion:
        slow_decay(comp);

    //PWM reads the comp values and gives HIGH or LOW pulse
    // GPIO outputs pulses to the driver

    // Start playback of the samples and loop indefinitely
    nrfx_pwm_simple_playback(&PWM_INST, &pwm_sequence, 1, NRFX_PWM_FLAG_LOOP);
}