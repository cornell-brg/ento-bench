#include <ento-mcu/timing.h>

volatile uint32_t lsu_overflow_count = 0;
volatile uint32_t cpi_overflow_count = 0;
volatile uint32_t sleep_overflow_count = 0;
volatile uint32_t fold_overflow_count = 0;
volatile uint32_t exc_overflow_count = 0;


void latency_pin_enable()
{
  gpio_init(latency_gpio_port,
            latency_gpio_pin,
            GPIO_Mode::Output,
            GPIO_OutputType::PushPull,
            GPIO_Speed::VeryHigh,
            GPIO_Pull::PullDown,
            GPIO_AltFunc::AF0);
}

void latency_pin_disable()
{
  gpio_disable(latency_gpio_port,
               latency_gpio_pin);
}

void trigger_pin_enable()
{
  gpio_init(trigger_gpio_port,
            trigger_gpio_pin,
            GPIO_Mode::Output,
            GPIO_OutputType::PushPull,
            GPIO_Speed::VeryHigh,
            GPIO_Pull::PullDown,
            GPIO_AltFunc::AF0);
}

void trigger_pin_disable()
{
  gpio_disable(trigger_gpio_port,
               trigger_gpio_pin);
}

void __attribute__ ((noinline)) software_delay_cycles(uint32_t cycles)
{
  __asm__ volatile ("");
  while (cycles--)
  {
    __asm__ volatile("add r0, r0, r0");
  }
  __asm__ volatile ("");
}
