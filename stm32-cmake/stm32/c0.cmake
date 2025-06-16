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

# Custom function to use vendor linker scripts for C0 devices
function(stm32c0_use_vendor_linker_script FAMILY DEVICE CORE)
    # Handle empty CORE parameter for C0 devices (they are single-core)
    if(NOT CORE OR CORE STREQUAL "")
        set(CORE "M0PLUS")
    endif()
    
    string(TOUPPER ${CORE} CORE_U)
    string(TOLOWER ${CORE} CORE_L)
    string(TOUPPER ${DEVICE} DEVICE_U)
    string(TOLOWER ${DEVICE} DEVICE_L)
    
    # Define the vendor linker script path
    set(VENDOR_LD_SCRIPT "${STM32_CMAKE_DIR}/linker-scripts/c0/${DEVICE}${CORE_U}.ld")
    
    # Check if vendor linker script exists
    if(EXISTS "${VENDOR_LD_SCRIPT}")
        message(STATUS "Using vendor linker script for ${DEVICE}${CORE_U}: ${VENDOR_LD_SCRIPT}")
        
        # Copy vendor linker script to build directory
        add_custom_command(
            OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${DEVICE}.ld"
            COMMAND ${CMAKE_COMMAND} -E copy "${VENDOR_LD_SCRIPT}" "${CMAKE_CURRENT_BINARY_DIR}/${DEVICE}.ld"
            DEPENDS "${VENDOR_LD_SCRIPT}"
            COMMENT "Copying vendor linker script for ${DEVICE}${CORE_U}"
        )
        
        # Add custom target to ensure the copy happens
        add_custom_target(${DEVICE}${CORE_U}_LINKER_SCRIPT DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/${DEVICE}.ld")
        
        # Configure the linker script for the target
        target_link_options(CMSIS::STM32::${DEVICE} INTERFACE 
            -T${CMAKE_CURRENT_BINARY_DIR}/${DEVICE}.ld
        )
        
        # Add dependency to ensure the linker script is copied before linking
        add_dependencies(CMSIS::STM32::${DEVICE} ${DEVICE}${CORE_U}_LINKER_SCRIPT)
        
        message(STATUS "Vendor linker script configured for CMSIS::STM32::${DEVICE}")
        
        # Set flag to indicate vendor script was configured
        set(VENDOR_SCRIPT_CONFIGURED TRUE PARENT_SCOPE)
    else()
        message(WARNING "Vendor linker script not found: ${VENDOR_LD_SCRIPT}")
        set(VENDOR_SCRIPT_CONFIGURED FALSE PARENT_SCOPE)
    endif()
endfunction()
