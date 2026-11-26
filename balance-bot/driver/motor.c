// Motor driver PWM and direction to spin the wheels

#include "motor.h"
#include "nrf_delay.h"

#define AIN1 EDGE_P15
#define AIN2 EDGE_P16

#define BIN1 EDGE_P13
#define BIN2 EDGE_P14

        static const nrfx_pwm_t PWM_INST = NRFX_PWM_INSTANCE(0);

        // Holds duty cycle values to trigger PWM toggle
        // create place nmemory for PWM value
        static nrf_pwm_values_individual_t sequence_data[1] = {0};

        static nrf_pwm_sequence_t pwm_sequence = {
            //
            .values.p_individual = sequence_data, // pointer to mem addr of sequent data
            .length = 4,
            .repeats = 0, // PWM periods to repeat each value
            .end_delay = 0, // PWM periods to wait after sequence ends
        };

        // Sequence structure for configuring
        static nrfx_pwm_config_t pwm_config = {
        .output_pins = {AIN1, AIN2, BIN1, BIN2},
        .base_clock = NRF_PWM_CLK_500kHz, //500kHz
        .count_mode = NRF_PWM_MODE_UP,
        // period is 500 × 2 µs = 1 ms
        // so PWM run at 1 kHz
        .top_value = 500, // where the counter resets
        .load_mode = NRF_PWM_LOAD_INDIVIDUAL, //gives the 4 pins their own pwm value
        .step_mode = NRF_PWM_STEP_AUTO, //gives the 4 pins their own duty value
        };

    static void pwm_init(void);

    void motor_init(void) {
        // initialize motor pins

        nrf_gpio_cfg_output(AIN1);
        nrf_gpio_cfg_output(AIN2);
        nrf_gpio_cfg_output(BIN1);
        nrf_gpio_cfg_output(BIN2);
        pwm_init();
    }

    static void slow_decay(float u, uint16_t comp) {
        // forwards or backwards motion:
        // printf("u VALUUUEEE: u=%.3f \n\n", u);

        if (u > 0.0f) {
            // printf("im going frwd");

            // forwards
            //store counter value in pwm sequence
            sequence_data[0].channel_0 = (uint16_t)comp;
            sequence_data[0].channel_1 = 0;
            sequence_data[0].channel_2 = 0; //BIN1
            sequence_data[0].channel_3 = (uint16_t)comp;  //BIN2
        } else if (u == 0) {
            // still (coasting)
            sequence_data[0].channel_0 = 0 | (1<<15);
            sequence_data[0].channel_1 = 0 | (1<<15);
            sequence_data[0].channel_2 = 0 | (1<<15);
            sequence_data[0].channel_3 = 0 | (1<<15);
        } else {
            // backwards
            // printf("im going back");
            sequence_data[0].channel_0 = 0; // AIN1
            sequence_data[0].channel_1 = (uint16_t)comp;  //AIN2
            sequence_data[0].channel_2 = (uint16_t)comp;
            sequence_data[0].channel_3 = 0;
        }
    }

    static void high_decay(float u, uint16_t comp) {
        // forwards or backwards motion:
        if (u > 0) {
            // forwards
            //store counter value in pwm sequence
            sequence_data[0].channel_0 = (uint16_t)comp | (1<<15); // AIN1
            sequence_data[0].channel_1 = 0 | (1<<15);  //AIN2
            sequence_data[0].channel_2 = (uint16_t)comp | (1<<15); //BIN1
            sequence_data[0].channel_3 = 0 | (1<<15);  //BIN2
        } else if (u == 0) {
            // still (coasting)
            sequence_data[0].channel_0 = 0 | (1<<15);
            sequence_data[0].channel_1 = 0 | (1<<15);
            sequence_data[0].channel_2 = 0 | (1<<15);
            sequence_data[0].channel_3 = 0 | (1<<15);
        } else {
            // backwards
            sequence_data[0].channel_0 = 0 | (1<<15);
            sequence_data[0].channel_1 = (uint16_t)comp | (1<<15);
            sequence_data[0].channel_2 = 0 | (1<<15);
            sequence_data[0].channel_3 = (uint16_t)comp | (1<<15);
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
    void set_motor(float u, float u_max) {

        nrfx_pwm_stop(&PWM_INST, true);

        // find duty cycle
        // what percent of the duty cycle the u represents
        // what percentage of duty cycle should this value be high
        float ratio = fabsf(u) / u_max;

        //1.0 is full motor voltage
        //start with less just in case
        float duty_cycle = fminf(ratio, 1.0f); // can't be less than 1

        // printf("duty cycle: %.3f \n\n", duty_cycle);

        // convert duty cycle into computer comp value
        // high 60% of the time -> 60% of PWM counter ticks
        float comp = pwm_config.top_value * duty_cycle;

        // printf("comp: %.3f \n\n", comp);


        // forwards or backwards motion:
        slow_decay(u, comp);

    //PWM reads the comp values and gives HIGH or LOW pulse
    // GPIO outputs pulses to the driver

    // Start playback of the samples and loop indefinitely
    nrfx_pwm_simple_playback(&PWM_INST, &pwm_sequence, 1, NRFX_PWM_FLAG_LOOP);
}