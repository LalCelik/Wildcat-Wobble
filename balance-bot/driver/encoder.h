#pragma once

#include <stdint.h>

void encoder_init(void);
float encoder_position(void);
void encoder_reset(void);
void encoder_print_raw(void); // DEBUG
