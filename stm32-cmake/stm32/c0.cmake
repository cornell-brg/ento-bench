set(STM32_C0_TYPES 
    C011xx
    C031xx
    C092xx
)
set(STM32_C0_TYPE_MATCH 
    "C011.[46]"
    "C031.[46]"
    "C092.."
)
set(STM32_C0_RAM_SIZES 
     6K
    12K
    32K
)
set(STM32_C0_CCRAM_SIZES 
     0K
     0K
     0K
)

stm32_util_create_family_targets(C0)

target_compile_options(STM32::C0 INTERFACE 
    -mcpu=cortex-m0plus
)
target_link_options(STM32::C0 INTERFACE 
    -mcpu=cortex-m0plus
)
