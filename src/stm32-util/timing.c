#include "timing.h"

void init_cycle_counter() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT
    DWT->CYCCNT = 0; // Reset cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable cycle counter
    last_cycle_count = 0; // Initialize the global variable
}

uint32_t read_cycle_counter() {
  return DWT->CYCCNT;
}

void update_last_cycle_count() {
    g_lastCycleCount = DWT->CYCCNT; // Update the global variable with the current cycle count
}

uint32_t get_elapsed_cycles() {
    uint32_t current_cycle_count = DWT->CYCCNT;
    uint32_t elapsed_cycles;
    
    if (current_cycle_count >= last_cycle_count) {
        elapsed_cycles = current_cycle_count - last_cycle_count;
    } else {
        // Handle rollover
        elapsed_cycles = (UINT32_MAX - last_cycle_count) + current_cycle_count + 1;
    }

    last_cycle_count = current_cycle_count; // Update last cycle count for next measurement
    return elapsed_cycles;
}
