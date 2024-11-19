#include <ento-mcu/pwr_util.h>

uint32_t get_vcore_range(void) {
  // Return the current voltage scaling range (VOS)
#if defined(STM32H7)
  return LL_PWR_GetRegulVoltageScaling() >> 9;
#endif
  return 0;
}
