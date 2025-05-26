#ifndef ENTO_CONTROL_LQR_H
#define ENTO_CONTROL_LQR_H

#ifndef ENTOBENCH_LQR_TRAITS
  #include <ento-control/lqr_traits_robofly.h>
  using EntoBenchDefaultLQRTraits = RoboFlyLQRTraits;
#else
  #include ENTOBENCH_LQR_TRAITS
  using EntoBenchDefaultLQRTraits = EntoBenchCustomLQRTraits;
#endif

#include "lqr_base.h"

namespace EntoControl
{
  using LQR = LQRStep< EntoBenchDefaultLQRTraits >;
}

#endif // ENTO_CONTROL_LQR_H
