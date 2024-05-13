#ifndef TIMING_H
#define TIMING_H

#include "stdint.h"
#include "core_cm4.h"

void init_cycle_counter(void);
uint32_t read_cycle_counter(void);
uint32_t update_last_cycle_count(void);
uint32_t get_elapsed_cycles(void);
volatile uint32_t last_cycle_count;

void init_gpio_timer();
void gpio_timer_toggle();



#endif
