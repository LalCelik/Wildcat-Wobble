// Motor driver PWM and direction to spin the wheels
#pragma once

#include <math.h>
#include "nrf_gpio.h"
#include "nrfx_pwm.h"
#include "microbit_v2.h"

void motor_init(void);
void set_motor(float u, float u_max);
void spin_angle(float angle_deg, float speed);
