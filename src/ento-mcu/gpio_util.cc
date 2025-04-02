#include <ento-mcu/gpio_util.h>

// Helper function to map GPIO_Mode enum to LL_GPIO_MODEj
static uint32_t convert_mode(GPIO_Mode mode) {
  switch (mode) {
      case GPIO_Mode::Input: return LL_GPIO_MODE_INPUT;
      case GPIO_Mode::Output: return LL_GPIO_MODE_OUTPUT;
      case GPIO_Mode::Alternate: return LL_GPIO_MODE_ALTERNATE;
      case GPIO_Mode::Analog: return LL_GPIO_MODE_ANALOG;
      default: return LL_GPIO_MODE_INPUT;
  }
}

static uint32_t convert_speed(GPIO_Speed speed)
{
  switch (speed)
  {
    case GPIO_Speed::Low:         return LL_GPIO_SPEED_FREQ_LOW;
    case GPIO_Speed::Medium:      return LL_GPIO_SPEED_FREQ_MEDIUM;
    case GPIO_Speed::High:        return LL_GPIO_SPEED_FREQ_HIGH;
    case GPIO_Speed::VeryHigh:   return LL_GPIO_SPEED_FREQ_VERY_HIGH;
    default:                      return LL_GPIO_SPEED_FREQ_LOW;
  }
}

static uint32_t convert_output_type(GPIO_OutputType out_type)
{
  switch (out_type)
  {
    case GPIO_OutputType::PushPull: return LL_GPIO_OUTPUT_PUSHPULL;
    case GPIO_OutputType::OpenDrain: return LL_GPIO_OUTPUT_OPENDRAIN;
    default:                         return LL_GPIO_OUTPUT_PUSHPULL;
  }
}

static uint32_t convert_pull(GPIO_Pull pull)
{
  switch (pull)
  {
    case GPIO_Pull::NoPull:      return LL_GPIO_PULL_NO;
    case GPIO_Pull::PullUp:      return LL_GPIO_PULL_UP;
    case GPIO_Pull::PullDown:    return LL_GPIO_PULL_DOWN;
    default:                      return LL_GPIO_PULL_NO;
  }
}

static uint32_t convert_af(GPIO_AltFunc af)
{
  switch (af)
  {
    case GPIO_AltFunc::AF0:  return LL_GPIO_AF_0;
    case GPIO_AltFunc::AF1:  return LL_GPIO_AF_1;
    case GPIO_AltFunc::AF2:  return LL_GPIO_AF_2;
    case GPIO_AltFunc::AF3:  return LL_GPIO_AF_3;
    case GPIO_AltFunc::AF4:  return LL_GPIO_AF_4;
    case GPIO_AltFunc::AF5:  return LL_GPIO_AF_5;
    case GPIO_AltFunc::AF6:  return LL_GPIO_AF_6;
    case GPIO_AltFunc::AF7:  return LL_GPIO_AF_7;
    case GPIO_AltFunc::AF8:  return LL_GPIO_AF_8;
    case GPIO_AltFunc::AF9:  return LL_GPIO_AF_9;
    case GPIO_AltFunc::AF10: return LL_GPIO_AF_10;
#if !defined(STM32G0)
    case GPIO_AltFunc::AF11: return LL_GPIO_AF_11;
    case GPIO_AltFunc::AF12: return LL_GPIO_AF_12;
    case GPIO_AltFunc::AF13: return LL_GPIO_AF_13;
    case GPIO_AltFunc::AF14: return LL_GPIO_AF_14;
    case GPIO_AltFunc::AF15: return LL_GPIO_AF_15;
#endif
    default:                 return LL_GPIO_AF_0; // Default to AF0 if invalid
  }
}

void gpio_init(GPIO_TypeDef* port,
               uint32_t pin,
               GPIO_Mode mode,
               GPIO_OutputType out_type,
               GPIO_Speed speed,
               GPIO_Pull pull,
               GPIO_AltFunc af)
{
  if (!gpio_is_clock_enabled(port)) gpio_enable_clock(port);

  LL_GPIO_InitTypeDef gpio_init = {0};
  gpio_init.Pin = pin;

  gpio_init.Mode = convert_mode(mode);
  gpio_init.Speed = convert_speed(speed);
  gpio_init.OutputType = convert_output_type(out_type);
  gpio_init.Pull = convert_pull(pull);
  gpio_init.Alternate = convert_af(af);

  LL_GPIO_Init(port, &gpio_init);
}

void gpio_disable(GPIO_TypeDef* port, uint32_t pin)
{
  LL_GPIO_InitTypeDef gpio_init = {0};

  gpio_init.Pin = pin;
  gpio_init.Mode       = LL_GPIO_MODE_ANALOG;
  gpio_init.Speed      = LL_GPIO_SPEED_FREQ_LOW;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull       = LL_GPIO_PULL_NO;
  gpio_init.Alternate  = LL_GPIO_AF_0;

  LL_GPIO_Init(port, &gpio_init);
}

void gpio_set_pin(GPIO_TypeDef* port, uint32_t pin)
{
  LL_GPIO_SetOutputPin(port, pin);
}

void gpio_clear_pin(GPIO_TypeDef* port, uint32_t pin)
{
  LL_GPIO_ResetOutputPin(port, pin);
}

void gpio_toggle_pin(GPIO_TypeDef* port, uint32_t pin) {
    LL_GPIO_TogglePin(port, pin);
}

bool gpio_read_pin(GPIO_TypeDef* port, uint32_t pin)
{
    return LL_GPIO_IsInputPinSet(port, pin);
}

bool gpio_is_clock_enabled(GPIO_TypeDef *gpio_port)
{
#if defined(STM32G0)
  if      (gpio_port == GPIOA) return LL_IOP_GRP1_IsEnabledClock(LL_IOP_GRP1_PERIPH_GPIOA);
  else if (gpio_port == GPIOB) return LL_IOP_GRP1_IsEnabledClock(LL_IOP_GRP1_PERIPH_GPIOB);
  else if (gpio_port == GPIOC) return LL_IOP_GRP1_IsEnabledClock(LL_IOP_GRP1_PERIPH_GPIOC);
  else if (gpio_port == GPIOD) return LL_IOP_GRP1_IsEnabledClock(LL_IOP_GRP1_PERIPH_GPIOD);
  else if (gpio_port == GPIOE) return LL_IOP_GRP1_IsEnabledClock(LL_IOP_GRP1_PERIPH_GPIOE);
  else if (gpio_port == GPIOF) return LL_IOP_GRP1_IsEnabledClock(LL_IOP_GRP1_PERIPH_GPIOF);
  else                         return false;
#elif defined(STM32G4)
  if      (gpio_port == GPIOA) return LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  else if (gpio_port == GPIOB) return LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  else if (gpio_port == GPIOC) return LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  else if (gpio_port == GPIOD) return LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOD);
  else if (gpio_port == GPIOE) return LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOE);
  else if (gpio_port == GPIOF) return LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOF);
  else if (gpio_port == GPIOG) return LL_AHB2_GRP1_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOG);
  else                         return false;
#elif defined(STM32H7)
  if      (gpio_port == GPIOA) return LL_AHB4_GRP1_IsEnabledClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  else if (gpio_port == GPIOB) return LL_AHB4_GRP1_IsEnabledClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  else if (gpio_port == GPIOC) return LL_AHB4_GRP1_IsEnabledClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  else if (gpio_port == GPIOD) return LL_AHB4_GRP1_IsEnabledClock(LL_AHB4_GRP1_PERIPH_GPIOD);
  else if (gpio_port == GPIOE) return LL_AHB4_GRP1_IsEnabledClock(LL_AHB4_GRP1_PERIPH_GPIOE);
  else if (gpio_port == GPIOF) return LL_AHB4_GRP1_IsEnabledClock(LL_AHB4_GRP1_PERIPH_GPIOF);
  else if (gpio_port == GPIOG) return LL_AHB4_GRP1_IsEnabledClock(LL_AHB4_GRP1_PERIPH_GPIOG);
  else if (gpio_port == GPIOH) return LL_AHB4_GRP1_IsEnabledClock(LL_AHB4_GRP1_PERIPH_GPIOH);
  else if (gpio_port == GPIOI) return LL_AHB4_GRP1_IsEnabledClock(LL_AHB4_GRP1_PERIPH_GPIOI);
  else                         return false;
#elif defined(STM32F7)
  if      (gpio_port == GPIOA) return LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  else if (gpio_port == GPIOB) return LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  else if (gpio_port == GPIOC) return LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  else if (gpio_port == GPIOD) return LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_GPIOD);
  else if (gpio_port == GPIOE) return LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_GPIOE);
  else if (gpio_port == GPIOF) return LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  else if (gpio_port == GPIOG) return LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_GPIOG);
  else if (gpio_port == GPIOH) return LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  else if (gpio_port == GPIOI) return LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_GPIOI);
  else                         return false;
#elif defined(STM32U5)
  if      (gpio_port == GPIOA) return LL_AHB2_GRP2_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  else if (gpio_port == GPIOB) return LL_AHB2_GRP2_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  else if (gpio_port == GPIOC) return LL_AHB2_GRP2_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  else if (gpio_port == GPIOD) return LL_AHB2_GRP2_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOD);
  else if (gpio_port == GPIOE) return LL_AHB2_GRP2_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOE);
  else if (gpio_port == GPIOF) return LL_AHB2_GRP2_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOF);
  else if (gpio_port == GPIOG) return LL_AHB2_GRP2_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOG);
  else if (gpio_port == GPIOH) return LL_AHB2_GRP2_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOH);
  else if (gpio_port == GPIOI) return LL_AHB2_GRP2_IsEnabledClock(LL_AHB2_GRP1_PERIPH_GPIOI);
  else                         return false;
#else
  static_assert(true, "Build is for unrecognized STM Family.");
  return false;
#endif
}

void gpio_enable_clock(GPIO_TypeDef *gpio_port)
{
#if defined(STM32G0)
  if      (gpio_port == GPIOA)  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  else if (gpio_port == GPIOB)  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  else if (gpio_port == GPIOC)  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  else if (gpio_port == GPIOD)  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOD);
  else if (gpio_port == GPIOE)  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOE);
  else if (gpio_port == GPIOF)  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);
  else                          return;
#elif defined(STM32G4)
  if      (gpio_port == GPIOA)  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  else if (gpio_port == GPIOB)  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  else if (gpio_port == GPIOC)  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  else if (gpio_port == GPIOD)  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
  else if (gpio_port == GPIOE)  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
  else if (gpio_port == GPIOF)  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
  else if (gpio_port == GPIOG)  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOG);
  else                          return;
#elif defined(STM32H7)
  if      (gpio_port == GPIOA)  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  else if (gpio_port == GPIOB)  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  else if (gpio_port == GPIOC)  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  else if (gpio_port == GPIOD)  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
  else if (gpio_port == GPIOE)  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);
  else if (gpio_port == GPIOF)  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOF);
  else if (gpio_port == GPIOG)  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOG);
  else if (gpio_port == GPIOH)  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOH);
  else if (gpio_port == GPIOI)  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOI);
  else                          return;
#elif defined(STM32F7)
  if      (gpio_port == GPIOA)  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  else if (gpio_port == GPIOB)  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  else if (gpio_port == GPIOC)  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  else if (gpio_port == GPIOD)  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
  else if (gpio_port == GPIOE)  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
  else if (gpio_port == GPIOF)  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  else if (gpio_port == GPIOG)  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOG);
  else if (gpio_port == GPIOH)  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  else if (gpio_port == GPIOI)  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOI);
  else                          return;
#elif defined(STM32U5)
  if      (gpio_port == GPIOA)  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  else if (gpio_port == GPIOB)  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  else if (gpio_port == GPIOC)  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  else if (gpio_port == GPIOD)  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
  else if (gpio_port == GPIOE)  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
  else if (gpio_port == GPIOF)  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
  else if (gpio_port == GPIOG)  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOG);
  else if (gpio_port == GPIOH)  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOH);
  else if (gpio_port == GPIOI)  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOI);
  else                          return;
  LL_PWR_EnableVddIO2();
#else
  static_assert(true, "Build is for unrecognized STM Family.");
  return false;
#endif
}

void gpio_disable_clock(GPIO_TypeDef *gpio_port)
{
#if defined(STM32G0)
  if      (gpio_port == GPIOA)  LL_IOP_GRP1_DisableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  else if (gpio_port == GPIOB)  LL_IOP_GRP1_DisableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  else if (gpio_port == GPIOC)  LL_IOP_GRP1_DisableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  else if (gpio_port == GPIOD)  LL_IOP_GRP1_DisableClock(LL_IOP_GRP1_PERIPH_GPIOD);
  else if (gpio_port == GPIOE)  LL_IOP_GRP1_DisableClock(LL_IOP_GRP1_PERIPH_GPIOE);
  else if (gpio_port == GPIOF)  LL_IOP_GRP1_DisableClock(LL_IOP_GRP1_PERIPH_GPIOF);
  else                          return;
#elif defined(STM32G4)
  if      (gpio_port == GPIOA)  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  else if (gpio_port == GPIOB)  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  else if (gpio_port == GPIOC)  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  else if (gpio_port == GPIOD)  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
  else if (gpio_port == GPIOE)  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
  else if (gpio_port == GPIOF)  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
  else if (gpio_port == GPIOG)  LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOG);
  else                          return;
#elif defined(STM32H7)
  if      (gpio_port == GPIOA)  LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  else if (gpio_port == GPIOB)  LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  else if (gpio_port == GPIOC)  LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  else if (gpio_port == GPIOD)  LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
  else if (gpio_port == GPIOE)  LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_GPIOE);
  else if (gpio_port == GPIOF)  LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_GPIOF);
  else if (gpio_port == GPIOG)  LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_GPIOG);
  else if (gpio_port == GPIOH)  LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_GPIOH);
  else if (gpio_port == GPIOI)  LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_GPIOI);
  else                          return;
#elif defined(STM32F7)
  if      (gpio_port == GPIOA)  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  else if (gpio_port == GPIOB)  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  else if (gpio_port == GPIOC)  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  else if (gpio_port == GPIOD)  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
  else if (gpio_port == GPIOE)  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
  else if (gpio_port == GPIOF)  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  else if (gpio_port == GPIOG)  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOG);
  else if (gpio_port == GPIOH)  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  else if (gpio_port == GPIOI)  LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOI);
  else                          return;
#elif defined(STM32U5)
  if      (gpio_port == GPIOA)  LL_AHB2_GRP2_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  else if (gpio_port == GPIOB)  LL_AHB2_GRP2_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  else if (gpio_port == GPIOC)  LL_AHB2_GRP2_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  else if (gpio_port == GPIOD)  LL_AHB2_GRP2_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
  else if (gpio_port == GPIOE)  LL_AHB2_GRP2_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
  else if (gpio_port == GPIOF)  LL_AHB2_GRP2_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
  else if (gpio_port == GPIOG)  LL_AHB2_GRP2_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOG);
  else if (gpio_port == GPIOH)  LL_AHB2_GRP2_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOH);
  else if (gpio_port == GPIOI)  LL_AHB2_GRP2_DisableClock(LL_AHB2_GRP1_PERIPH_GPIOI);
  else                          return;
#else
  static_assert(true, "Build is for unrecognized STM Family.");
  return false;
#endif
}

