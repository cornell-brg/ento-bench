#ifndef GPIO_UTIL_H
#define GPIO_UTIL_H

#include <ento-mcu/conf.h>

enum GPIO_Mode {
  Input,
  Output,
  Alternate,
  Analog
};

enum GPIO_Speed {
  Low,
  Medium,
  High,
  VeryHigh
};

enum GPIO_OutputType
{
  PushPull,
  OpenDrain
};

enum GPIO_Pull
{
  NoPull,
  PullUp,
  PullDown
};

enum GPIO_AltFunc
{
  AF0,
  AF1,
  AF2,
  AF3,
  AF4,
  AF5,
  AF6,
  AF7,
  AF8,
  AF9,
  AF10,
  AF11,
  AF12,
  AF13,
  AF14,
  AF15,
};



// Initialize a GPIO pin
void gpio_init(GPIO_TypeDef*   port, 
               uint32_t        pin,
               GPIO_Mode       mode,
               GPIO_OutputType output,
               GPIO_Speed      speed,
               GPIO_Pull       pull,
               GPIO_AltFunc    af);

void gpio_disable(GPIO_TypeDef* port,
                  uint32_t      pin);

// Set, clear, toggle, and read pin functions
void gpio_set_pin(GPIO_TypeDef* port, uint32_t pin);
void gpio_clear_pin(GPIO_TypeDef* port, uint32_t pin);
void gpio_toggle_pin(GPIO_TypeDef* port, uint32_t pin);
bool gpio_read_pin(GPIO_TypeDef* port, uint32_t pin);

// Check, enable, and discable RCC clock for GPIO Port
bool gpio_is_clock_enabled(GPIO_TypeDef* port);
void gpio_enable_clock(GPIO_TypeDef* port);
void gpio_disable_clock(GPIO_TypeDef* port);

#endif // GPIO_UTIL_H
