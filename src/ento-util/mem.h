#ifndef ENTO_MEM_H
#define ENTO_MEM_H

#if defined(STM32_BUILD) && defined(STM32H7)
#define ENTOMEM __attribute__((section(".dtcm_data")))
#else
#define ENTOMEM
#endif


#endif
