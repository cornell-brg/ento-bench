#include "pwr_util.h"

uint32_t get_vcore_range(void) {
  // Return the current voltage scaling range (VOS)
  return LL_PWR_GetRegulVoltageScaling() >> 9;
}
