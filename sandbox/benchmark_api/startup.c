#include <stdint.h>

// Forward declaration of the default fault handlers
void Reset_Handler(void);
void Default_Handler(void);

// External declaration for the application's entry point (main)
extern int main(void);

// Reserve stack space
static uint32_t stack[256];

// Vector table
__attribute__((section(".isr_vector")))
const uint32_t vector_table[] = {
    (uint32_t)stack + sizeof(stack), // Initial stack pointer
    (uint32_t)Reset_Handler,         // Reset handler
    // Other exception handlers can be added here
    (uint32_t)Default_Handler,       // NMI handler
    (uint32_t)Default_Handler,       // Hard fault handler
    // ...
};

void Reset_Handler(void) {
    // Call the application's entry point
    main();

    // Infinite loop to prevent exit
    while (1) {}
}

void Default_Handler(void) {
    // Infinite loop
    while (1) {}
}
