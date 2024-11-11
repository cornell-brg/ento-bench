# For information about why and how of this file: https://cmake.org/cmake/help/latest/command/find_package.html




set(CMSIS_RTOS RTOS RTOS_V2)

message(STATUS "CMSIS_FIND_COMPONENTS: ${CMSIS_FIND_COMPONENTS}")
if(NOT CMSIS_FIND_COMPONENTS)
    message(STATUS "Not found CMSIS_FIND_COMPONENTS. Adding ${STM32_SUPPORTED_FAMILIES_LONG_NAME}")
    set(CMSIS_FIND_COMPONENTS ${STM32_SUPPORTED_FAMILIES_LONG_NAME})
endif()

if(STM32H7 IN_LIST CMSIS_FIND_COMPONENTS)
    list(REMOVE_ITEM CMSIS_FIND_COMPONENTS STM32H7)
    list(APPEND CMSIS_FIND_COMPONENTS STM32H7_M7 STM32H7_M4)
    message(STATUS "Added STM32H7_M7 and _M4 to CMSIS_FIND_COMPONENTS: ${CMSIS_FIND_COMPONENTS}")
endif()

if(STM32WB IN_LIST CMSIS_FIND_COMPONENTS)
    list(REMOVE_ITEM CMSIS_FIND_COMPONENTS STM32WB)
    list(APPEND CMSIS_FIND_COMPONENTS STM32WB_M4)
endif()

if(STM32WL IN_LIST CMSIS_FIND_COMPONENTS)
    list(REMOVE_ITEM CMSIS_FIND_COMPONENTS STM32WL)
    list(APPEND CMSIS_FIND_COMPONENTS STM32WL_M4 STM32WL_M0PLUS)
endif()

if(STM32MP1 IN_LIST CMSIS_FIND_COMPONENTS)
    list(REMOVE_ITEM CMSIS_FIND_COMPONENTS STM32MP1)
    list(APPEND CMSIS_FIND_COMPONENTS STM32MP1_M4)
endif()

list(REMOVE_DUPLICATES CMSIS_FIND_COMPONENTS)

# This section fills the RTOS or family components list
message(STATUS "CMSIS_FIND_COMPONENTS: ${CMSIS_FIND_COMPONENTS}")
foreach(COMP ${CMSIS_FIND_COMPONENTS})
    string(TOLOWER ${COMP} COMP_L)
    string(TOUPPER ${COMP} COMP)

    # Component is RTOS component
    if(${COMP} IN_LIST CMSIS_RTOS)
        list(APPEND CMSIS_FIND_COMPONENTS_RTOS ${COMP})
        message(STATUS "Added ${COMP} to CMSIS_FIND_COMPONENTS_RTOS")
        continue()
    endif()

    # Component is not RTOS component, so check whether it is a family component
    message(STATUS "Regex on ${COMP} from ${CMSIS_FIND_COMPONENTS}")
    string(REGEX MATCH "^STM32([CFGHLMUW]P?[0-9BL])([0-9A-Z][0-9M][A-Z][0-9A-Z])?_?(M0PLUS|M4|M7)?.*$" COMP ${COMP})
    if(CMAKE_MATCH_1)
        list(APPEND CMSIS_FIND_COMPONENTS_FAMILIES ${COMP})
        message(STATUS "Added ${COMP} to CMSIS_FIND_COMPONENTS_FAMILIES")
    endif()
endforeach()

if(NOT CMSIS_FIND_COMPONENTS_FAMILIES)
    set(CMSIS_FIND_COMPONENTS_FAMILIES ${STM32_SUPPORTED_FAMILIES_LONG_NAME})
endif()

if(NOT CMSIS_FIND_COMPONENTS_RTOS)
    set(CMSIS_FIND_COMPONENTS_RTOS ${CMSIS_RTOS})
endif()

message(STATUS "Search for CMSIS families: ${CMSIS_FIND_COMPONENTS_FAMILIES}")
message(STATUS "Search for CMSIS RTOS: ${CMSIS_FIND_COMPONENTS_RTOS}")

include(stm32/devices)

function(cmsis_generate_parametrizable_linker_script FAMILY DEVICE CORE)
    # Define keyword arguments with defaults
    set(options "")
    set(one_value_args
        FLASH_ORIGIN RAM_ORIGIN CCRAM_ORIGIN RAM_SHARE_ORIGIN
        FLASH_SIZE RAM_SIZE CCRAM_SIZE RAM_SHARE_SIZE STACK_SIZE HEAP_SIZE)
    set(multi_value_args "")

    cmake_parse_arguments(PARAMS "" "${one_value_args}" "" ${ARGN})

    # Apply default values if keywords are not provided
    set(FLASH_ORIGIN ${PARAMS_FLASH_ORIGIN})
    set(RAM_ORIGIN ${PARAMS_RAM_ORIGIN})
    set(CCRAM_ORIGIN ${PARAMS_CCRAM_ORIGIN})
    set(RAM_SHARE_ORIGIN ${PARAMS_RAM_SHARE_ORIGIN})
    set(FLASH_SIZE ${PARAMS_FLASH_SIZE})
    set(RAM_SIZE ${PARAMS_RAM_SIZE})
    set(CCRAM_SIZE ${PARAMS_CCRAM_SIZE})
    set(RAM_SHARE_SIZE ${PARAMS_RAM_SHARE_SIZE})
    set(STACK_SIZE ${PARAMS_STACK_SIZE})
    set(HEAP_SIZE ${PARAMS_HEAP_SIZE})

    # Fallbacks for any parameters not passed
    if(NOT FLASH_ORIGIN)
        set(FLASH_ORIGIN "0x08000000")
    endif()
    if(NOT RAM_ORIGIN)
        set(RAM_ORIGIN "0x20000000")
    endif()
    if(NOT CCRAM_ORIGIN)
        set(CCRAM_ORIGIN "0x10000000")
    endif()
    if(NOT RAM_SHARE_ORIGIN)
        set(RAM_SHARE_ORIGIN "0x24000000")
    endif()
    if(NOT FLASH_SIZE)
        set(FLASH_SIZE "128K")
    endif()
    if(NOT RAM_SIZE)
        set(RAM_SIZE "64K")
    endif()
    if(NOT CCRAM_SIZE)
        set(CCRAM_SIZE "64K")
    endif()
    if(NOT RAM_SHARE_SIZE)
        set(RAM_SHARE_SIZE "32K")
    endif()
    if(NOT STACK_SIZE)
        set(STACK_SIZE "0x400")
    endif()
    if(NOT HEAP_SIZE)
        set(HEAP_SIZE "0x200")
    endif()

    if(CORE)
      message(STATUS "CORE ${CORE}")
      set(CORE_C "::${CORE}")
      set(CORE_U "_${CORE}")
    endif()

    set(OUTPUT_LD_FILE "${CMAKE_CURRENT_BINARY_DIR}/${DEVICE}${CORE_U}.ld")

    if(${FAMILY} STREQUAL MP1)
        string(TOLOWER ${FAMILY} FAMILY_L)
        find_file(CMSIS_${FAMILY}${CORE_U}_LD_SCRIPT
            NAMES stm32mp15xx_m4.ld
            PATHS "${CMSIS_${FAMILY}${CORE_U}_PATH}/Source/Templates/gcc/linker"
            NO_DEFAULT_PATH
        )
        add_custom_command(OUTPUT "${OUTPUT_LD_FILE}"
            COMMAND ${CMAKE_COMMAND}
                -E copy ${CMSIS_${FAMILY}${CORE_U}_LD_SCRIPT} ${OUTPUT_LD_FILE})
    else()
        # Only call stm32_get_memory_info if variables are not already defined
        if(NOT DEFINED FLASH_ORIGIN)
            stm32_get_memory_info(FAMILY ${FAMILY} DEVICE ${DEVICE} CORE ${CORE} FLASH SIZE FLASH_SIZE ORIGIN FLASH_ORIGIN)
        endif()
        if(NOT DEFINED RAM_ORIGIN)
            stm32_get_memory_info(FAMILY ${FAMILY} DEVICE ${DEVICE} CORE ${CORE} RAM SIZE RAM_SIZE ORIGIN RAM_ORIGIN)
        endif()
        if(NOT DEFINED CCRAM_ORIGIN)
            stm32_get_memory_info(FAMILY ${FAMILY} DEVICE ${DEVICE} CORE ${CORE} CCRAM SIZE CCRAM_SIZE ORIGIN CCRAM_ORIGIN)
        endif()
        if(NOT DEFINED RAM_SHARE_ORIGIN)
            stm32_get_memory_info(FAMILY ${FAMILY} DEVICE ${DEVICE} CORE ${CORE} RAM_SHARE SIZE RAM_SHARE_SIZE ORIGIN RAM_SHARE_ORIGIN)
        endif()
        if(NOT DEFINED STACK_SIZE)
          stm32_get_memory_info(
              STACK
              FAMILY ${STM32_FAMILY}
              DEVICE ${STM32_DEVICE}
              SIZE STACK_SIZE
              ORIGIN STACK_ORIGIN
          )
        endif()
        if(NOT DEFINED HEAP_SIZE)
          stm32_get_memory_info(
              STACK
              FAMILY ${STM32_FAMILY}
              DEVICE ${STM32_DEVICE}
              SIZE HEAP_SIZE
              ORIGIN HEAP_ORIGIN
          )
        endif()
        message(STATUS "Using STACK_SIZE: ${STACK_SIZE}")
        add_custom_command(OUTPUT "${OUTPUT_LD_FILE}"
            COMMAND ${CMAKE_COMMAND} 
                -DFLASH_ORIGIN="${FLASH_ORIGIN}" 
                -DRAM_ORIGIN="${RAM_ORIGIN}" 
                -DCCRAM_ORIGIN="${CCRAM_ORIGIN}" 
                -DRAM_SHARE_ORIGIN="${RAM_SHARE_ORIGIN}" 
                -DFLASH_SIZE="${FLASH_SIZE}" 
                -DRAM_SIZE="${RAM_SIZE}" 
                -DCCRAM_SIZE="${CCRAM_SIZE}"
                -DRAM_SHARE_SIZE="${RAM_SHARE_SIZE}" 
                -DSTACK_SIZE="${STACK_SIZE}" 
                -DHEAP_SIZE="${HEAP_SIZE}" 
                -DLINKER_SCRIPT="${OUTPUT_LD_FILE}"
                -P "${STM32_CMAKE_DIR}/stm32/linker_ld.cmake"
        )
    endif()

    add_custom_target(CMSIS_LD_${DEVICE}${CORE_U} DEPENDS "${OUTPUT_LD_FILE}")
    add_dependencies(CMSIS::STM32::${DEVICE}${CORE_C} CMSIS_LD_${DEVICE}${CORE_U})
    message(STATUS "Linker script dependency: CMSIS::STM32::${DEVICE}${CORE_C}")
    stm32_add_linker_script(CMSIS::STM32::${DEVICE}${CORE_C} INTERFACE "${OUTPUT_LD_FILE}")
endfunction()

function(cmsis_generate_default_linker_script FAMILY DEVICE CORE)
    if(CORE)
        set(CORE_C "::${CORE}")
        set(CORE_U "_${CORE}")
    endif()
    
    set(OUTPUT_LD_FILE "${CMAKE_CURRENT_BINARY_DIR}/${DEVICE}${CORE_U}.ld")
    
    if(${FAMILY} STREQUAL MP1)
        string(TOLOWER ${FAMILY} FAMILY_L)
        find_file(CMSIS_${FAMILY}${CORE_U}_LD_SCRIPT
            NAMES stm32mp15xx_m4.ld
            PATHS "${CMSIS_${FAMILY}${CORE_U}_PATH}/Source/Templates/gcc/linker"
            NO_DEFAULT_PATH
        )
        add_custom_command(OUTPUT "${OUTPUT_LD_FILE}"
            COMMAND ${CMAKE_COMMAND}
                -E copy ${CMSIS_${FAMILY}${CORE_U}_LD_SCRIPT} ${OUTPUT_LD_FILE})
    else()    
        stm32_get_memory_info(FAMILY ${FAMILY} DEVICE ${DEVICE} CORE ${CORE} FLASH SIZE FLASH_SIZE ORIGIN FLASH_ORIGIN)
        stm32_get_memory_info(FAMILY ${FAMILY} DEVICE ${DEVICE} CORE ${CORE} RAM SIZE RAM_SIZE ORIGIN RAM_ORIGIN)
        stm32_get_memory_info(FAMILY ${FAMILY} DEVICE ${DEVICE} CORE ${CORE} CCRAM SIZE CCRAM_SIZE ORIGIN CCRAM_ORIGIN)
        stm32_get_memory_info(FAMILY ${FAMILY} DEVICE ${DEVICE} CORE ${CORE} RAM_SHARE SIZE RAM_SHARE_SIZE ORIGIN RAM_SHARE_ORIGIN)
        stm32_get_memory_info(FAMILY ${FAMILY} DEVICE ${DEVICE} CORE ${CORE} HEAP SIZE HEAP_SIZE)
        stm32_get_memory_info(FAMILY ${FAMILY} DEVICE ${DEVICE} CORE ${CORE} STACK SIZE STACK_SIZE)

        message(STATUS "STACK_SIZE: ${STACK_SIZE}")
        add_custom_command(OUTPUT "${OUTPUT_LD_FILE}"
            COMMAND ${CMAKE_COMMAND} 
                -DFLASH_ORIGIN="${FLASH_ORIGIN}" 
                -DRAM_ORIGIN="${RAM_ORIGIN}" 
                -DCCRAM_ORIGIN="${CCRAM_ORIGIN}" 
                -DRAM_SHARE_ORIGIN="${RAM_SHARE_ORIGIN}" 
                -DFLASH_SIZE="${FLASH_SIZE}" 
                -DRAM_SIZE="${RAM_SIZE}" 
                -DCCRAM_SIZE="${CCRAM_SIZE}"
                -DRAM_SHARE_SIZE="${RAM_SHARE_SIZE}" 
                -DSTACK_SIZE="${STACK_SIZE}" 
                -DHEAP_SIZE="${HEAP_SIZE}" 
                -DLINKER_SCRIPT="${OUTPUT_LD_FILE}"
                -P "${STM32_CMAKE_DIR}/stm32/linker_ld.cmake"
        )
    endif()
    add_custom_target(CMSIS_LD_${DEVICE}${CORE_U} DEPENDS "${OUTPUT_LD_FILE}")
    add_dependencies(CMSIS::STM32::${DEVICE}${CORE_C} CMSIS_LD_${DEVICE}${CORE_U})
    message(STATUS "Linker script dependency: CMSIS::STM32::${DEVICE}${CORE_C}")
    stm32_add_linker_script(CMSIS::STM32::${DEVICE}${CORE_C} INTERFACE "${OUTPUT_LD_FILE}")
endfunction() 

foreach(COMP ${CMSIS_FIND_COMPONENTS_FAMILIES})
    string(TOLOWER ${COMP} COMP_L)
    string(TOUPPER ${COMP} COMP)
    message(STATUS "CMSIS find components families: ${CMSIS_FIND_COMPONENTS_FAMILIES}")
    message(STATUS "For each comp loop (COMP): ${COMP}")
    
    string(REGEX MATCH "^STM32([CFGHLMUW]P?[0-9BL])([0-9A-Z][0-9M][A-Z][0-9A-Z])?_?(M0PLUS|M4|M7)?.*$" COMP ${COMP})
    # CMAKE_MATCH_<n> contains n'th subexpression
    # CMAKE_MATCH_0 contains full match

    
    if(CMAKE_MATCH_2)
        set(FAMILY ${CMAKE_MATCH_1})
        set(STM_DEVICES "${CMAKE_MATCH_1}${CMAKE_MATCH_2}")
        message(TRACE "FindCMSIS: full device name match for COMP ${COMP}, STM_DEVICES is ${STM_DEVICES}")
    else()
        set(FAMILY ${CMAKE_MATCH_1})
        stm32_get_devices_by_family(STM_DEVICES FAMILY ${FAMILY})
        message(TRACE "FindCMSIS: family only match for COMP ${COMP}, STM_DEVICES is ${STM_DEVICES}")
    endif()
    
    if(CMAKE_MATCH_3)
        set(CORE ${CMAKE_MATCH_3})
        set(CORE_C "::${CORE}")
        set(CORE_U "_${CORE}")
        set(CORE_Ucm "_c${CORE}")
        string(TOLOWER ${CORE_Ucm} CORE_Ucm)
        message(TRACE "FindCMSIS: core match in component name for COMP ${COMP}. CORE is ${CORE}")
    else()
        unset(CORE)
        unset(CORE_C)
        unset(CORE_U)
        unset(CORE_Ucm)
    endif()
    
    string(TOLOWER ${FAMILY} FAMILY_L)
    
    if((NOT STM32_CMSIS_${FAMILY}_PATH) AND (NOT STM32_CUBE_${FAMILY}_PATH) AND (DEFINED ENV{STM32_CUBE_${FAMILY}_PATH}))
        # try to set path from environment variable. Note it could be ...-NOT-FOUND and it's fine
        set(STM32_CUBE_${FAMILY}_PATH $ENV{STM32_CUBE_${FAMILY}_PATH} CACHE PATH "Path to STM32Cube${FAMILY}")
        message(STATUS "ENV STM32_CUBE_${FAMILY}_PATH specified, using STM32_CUBE_${FAMILY}_PATH: ${STM32_CUBE_${FAMILY}_PATH}")
    endif()

    if((NOT STM32_CMSIS_${FAMILY}_PATH) AND (NOT STM32_CUBE_${FAMILY}_PATH))
        set(STM32_CUBE_${FAMILY}_PATH /opt/STM32Cube${FAMILY} CACHE PATH "Path to STM32Cube${FAMILY}")
        message(STATUS "Neither STM32_CUBE_${FAMILY}_PATH nor STM32_CMSIS_${FAMILY}_PATH specified using default STM32_CUBE_${FAMILY}_PATH: ${STM32_CUBE_${FAMILY}_PATH}")
    endif()
     
    # search for Include/cmsis_gcc.h
    find_path(CMSIS_${FAMILY}${CORE_U}_CORE_PATH
        NAMES Include/cmsis_gcc.h
        PATHS "${STM32_CMSIS_PATH}" "${STM32_CUBE_${FAMILY}_PATH}/Drivers/CMSIS"
        NO_DEFAULT_PATH
    )
    if (NOT CMSIS_${FAMILY}${CORE_U}_CORE_PATH)
        message(VERBOSE "FindCMSIS: cmsis_gcc.h for ${FAMILY}${CORE_U} has not been found")
        continue()
    endif()
	
    message(STATUS "CMSIS FAMILY PATH: ${STM32_CMSIS_${FAMILY}_PATH}. CUBE FAMILY PATH: ${STM32_CUBE_${FAMILY}_PATH}")
    # search for Include/stm32[XX]xx.h
    find_path(CMSIS_${FAMILY}${CORE_U}_PATH
        NAMES Include/stm32${FAMILY_L}xx.h
        PATHS "${STM32_CMSIS_${FAMILY}_PATH}" "${STM32_CUBE_${FAMILY}_PATH}/Drivers/CMSIS/Device/ST/STM32${FAMILY}xx"
        NO_DEFAULT_PATH
    )
    if (NOT CMSIS_${FAMILY}${CORE_U}_PATH)
        message(VERBOSE "FindCMSIS: stm32${FAMILY_L}xx.h for ${FAMILY}${CORE_U} has not been found")
        continue()
    endif()
    list(APPEND CMSIS_INCLUDE_DIRS "${CMSIS_${FAMILY}${CORE_U}_CORE_PATH}/Include" "${CMSIS_${FAMILY}${CORE_U}_PATH}/Include")

    if(NOT CMSIS_${FAMILY}${CORE_U}_VERSION)
        find_file(CMSIS_${FAMILY}${CORE_U}_PDSC
            NAMES ARM.CMSIS.pdsc
            PATHS "${CMSIS_${FAMILY}${CORE_U}_CORE_PATH}"
            NO_DEFAULT_PATH
        )
        if (NOT CMSIS_${FAMILY}${CORE_U}_PDSC)
            set(CMSIS_${FAMILY}${CORE_U}_VERSION "0.0.0")
        else()
            file(STRINGS "${CMSIS_${FAMILY}${CORE_U}_PDSC}" VERSION_STRINGS REGEX "<release version=\"([0-9]*\\.[0-9]*\\.[0-9]*)\" date=\"[0-9]+\\-[0-9]+\\-[0-9]+\">")
            list(GET VERSION_STRINGS 0 STR)
            string(REGEX MATCH "<release version=\"([0-9]*)\\.([0-9]*)\\.([0-9]*)\" date=\"[0-9]+\\-[0-9]+\\-[0-9]+\">" MATCHED ${STR})
            set(CMSIS_${FAMILY}${CORE_U}_VERSION "${CMAKE_MATCH_1}.${CMAKE_MATCH_2}.${CMAKE_MATCH_3}" CACHE INTERNAL "CMSIS STM32${FAMILY}${CORE_U} version")
        endif()
    endif()
    
    set(CMSIS_${COMP}_VERSION ${CMSIS_${FAMILY}${CORE_U}_VERSION})
    set(CMSIS_VERSION ${CMSIS_${COMP}_VERSION})

    if(NOT (TARGET CMSIS::STM32::${FAMILY}${CORE_C}))
        message(TRACE "FindCMSIS: creating library CMSIS::STM32::${FAMILY}${CORE_C}")
        message(STATUS "FindCMSIS: creating library CMSIS::STM32::${FAMILY}${CORE_C}")
        add_library(CMSIS::STM32::${FAMILY}${CORE_C} INTERFACE IMPORTED)
        #STM32::${FAMILY}${CORE_C} contains compile options and is define in <family>.cmake
        target_link_libraries(CMSIS::STM32::${FAMILY}${CORE_C} INTERFACE STM32::${FAMILY}${CORE_C})
        target_include_directories(CMSIS::STM32::${FAMILY}${CORE_C} INTERFACE "${CMSIS_${FAMILY}${CORE_U}_CORE_PATH}/Include")
        target_include_directories(CMSIS::STM32::${FAMILY}${CORE_C} INTERFACE "${CMSIS_${FAMILY}${CORE_U}_PATH}/Include")
    endif()

    # search for system_stm32[XX]xx.c
    find_file(CMSIS_${FAMILY}${CORE_U}_SYSTEM
        NAMES system_stm32${FAMILY_L}xx.c
        PATHS "${CMSIS_${FAMILY}${CORE_U}_PATH}/Source/Templates"
        NO_DEFAULT_PATH
    )
    list(APPEND CMSIS_SOURCES "${CMSIS_${FAMILY}${CORE_U}_SYSTEM}")
    
    if(NOT CMSIS_${FAMILY}${CORE_U}_SYSTEM)
        message(VERBOSE "FindCMSIS: system_stm32${FAMILY_L}xx.c for ${FAMILY}${CORE_U} has not been found")
        continue()
    endif()
    
    set(STM_DEVICES_FOUND TRUE)
    foreach(DEVICE ${STM_DEVICES})
        message(TRACE "FindCMSIS: Iterating DEVICE ${DEVICE}")
        
        stm32_get_cores(DEV_CORES FAMILY ${FAMILY} DEVICE ${DEVICE})
        if(CORE AND (NOT ${CORE} IN_LIST DEV_CORES))
            message(TRACE "FindCMSIS: skip device because CORE ${CORE} provided doesn't correspond to FAMILY ${FAMILY} DEVICE ${DEVICE}")
            continue()
        endif()
                
        stm32_get_chip_type(${FAMILY} ${DEVICE} TYPE)
        string(TOLOWER ${DEVICE} DEVICE_L)
        string(TOLOWER ${TYPE} TYPE_L)
        
        get_property(languages GLOBAL PROPERTY ENABLED_LANGUAGES)
        if(NOT "ASM" IN_LIST languages)
            message(STATUS "Languages: ${languages}")
            message(STATUS "FindCMSIS: Not generating target for startup file and linker script because ASM language is not enabled")
            continue()
        endif()
        
        find_file(CMSIS_${FAMILY}${CORE_U}_${TYPE}_STARTUP
            NAMES startup_stm32${TYPE_L}.s 
                  startup_stm32${TYPE_L}${CORE_Ucm}.s
            PATHS "${CMSIS_${FAMILY}${CORE_U}_PATH}/Source/Templates/gcc"
            NO_DEFAULT_PATH
        )
        list(APPEND CMSIS_SOURCES "${CMSIS_${FAMILY}${CORE_U}_${TYPE}_STARTUP}")
        if(NOT CMSIS_${FAMILY}${CORE_U}_${TYPE}_STARTUP)
            set(STM_DEVICES_FOUND FALSE)
            message(VERBOSE "FindCMSIS: did not find file: startup_stm32${TYPE_L}.s or startup_stm32${TYPE_L}${CORE_Ucm}.s")
            break()
        endif()
        
        if(NOT (TARGET CMSIS::STM32::${TYPE}${CORE_C}))
            message(TRACE "FindCMSIS: creating library CMSIS::STM32::${TYPE}${CORE_C}")
            message(STATUS "FindCMSIS: creating library CMSIS::STM32::${TYPE}${CORE_C}")
            add_library(CMSIS::STM32::${TYPE}${CORE_C} INTERFACE IMPORTED)
            target_link_libraries(CMSIS::STM32::${TYPE}${CORE_C} INTERFACE CMSIS::STM32::${FAMILY}${CORE_C} STM32::${TYPE}${CORE_C})
            target_sources(CMSIS::STM32::${TYPE}${CORE_C} INTERFACE "${CMSIS_${FAMILY}${CORE_U}_${TYPE}_STARTUP}")
            target_sources(CMSIS::STM32::${TYPE}${CORE_C} INTERFACE "${CMSIS_${FAMILY}${CORE_U}_SYSTEM}")
        endif()
        
        add_library(CMSIS::STM32::${DEVICE}${CORE_C} INTERFACE IMPORTED)
        target_link_libraries(CMSIS::STM32::${DEVICE}${CORE_C} INTERFACE CMSIS::STM32::${TYPE}${CORE_C})
        message(STATUS "Generating default linker script for ${FAMILY}, ${DEVICE}, ${CORE}")
        cmsis_generate_default_linker_script(${FAMILY} ${DEVICE} "${CORE}")
    endforeach()

    if(STM_DEVICES_FOUND)
       set(CMSIS_${COMP}_FOUND TRUE)
       message(DEBUG "CMSIS_${COMP}_FOUND TRUE")
    else()
       set(CMSIS_${COMP}_FOUND FALSE)
       message(DEBUG "CMSIS_${COMP}_FOUND FALSE")
    endif()

    foreach(RTOS_COMP ${CMSIS_FIND_COMPONENTS_RTOS})
        if (${RTOS_COMP} STREQUAL "RTOS_V2")
            set(RTOS_COMP_VERSION "2")
        else()
            unset(RTOS_COMP_VERSION)
        endif()

        find_path(CMSIS_${FAMILY}${CORE_U}_${RTOS_COMP}_PATH
            NAMES "cmsis_os${RTOS_COMP_VERSION}.h"
            PATHS "${STM32_CUBE_${FAMILY}_PATH}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_${RTOS_COMP}"
            NO_DEFAULT_PATH
        )
        if (NOT CMSIS_${FAMILY}${CORE_U}_${RTOS_COMP}_PATH)
            continue()
        endif()

        find_file(CMSIS_${FAMILY}${CORE_U}_${RTOS_COMP}_SOURCE
            NAMES "cmsis_os${RTOS_COMP_VERSION}.c"
            PATHS "${STM32_CUBE_${FAMILY}_PATH}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_${RTOS_COMP}"
            NO_DEFAULT_PATH
        )
        if (NOT CMSIS_${FAMILY}${CORE_U}_${RTOS_COMP}_SOURCE)
            continue()
        endif()

        if(NOT (TARGET CMSIS::STM32::${FAMILY}${CORE_C}::${RTOS_COMP}))
            add_library(CMSIS::STM32::${FAMILY}${CORE_C}::${RTOS_COMP} INTERFACE IMPORTED)
      xxdd_library(CMSIS::STM32::${DEVICE}${CORE_C} INTERFACE IMPORTED)
            target_link_libraries(CMSIS::STM32::${FAMILY}${CORE_C}::${RTOS_COMP} INTERFACE CMSIS::STM32::${FAMILY}${CORE_C})
            target_include_directories(CMSIS::STM32::${FAMILY}${CORE_C}::${RTOS_COMP} INTERFACE "${CMSIS_${FAMILY}${CORE_U}_${RTOS_COMP}_PATH}")
            target_sources(CMSIS::STM32::${FAMILY}${CORE_C}::${RTOS_COMP} INTERFACE "${CMSIS_${FAMILY}${CORE_U}_${RTOS_COMP}_SOURCE}")
        endif()

        list(APPEND CMSIS_SOURCES "${CMSIS_${FAMILY}${CORE_U}_${RTOS_COMP}_SOURCE}")
        list(APPEND CMSIS_INCLUDE_DIRS "${CMSIS_${FAMILY}${CORE_U}_${RTOS_COMP}_PATH}")
        set(CMSIS_${RTOS_COMP}_FOUND TRUE)
    endforeach()

    list(REMOVE_DUPLICATES CMSIS_INCLUDE_DIRS)
    list(REMOVE_DUPLICATES CMSIS_SOURCES)
endforeach()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CMSIS
    REQUIRED_VARS CMSIS_INCLUDE_DIRS CMSIS_SOURCES
    FOUND_VAR CMSIS_FOUND
    VERSION_VAR CMSIS_VERSION
    HANDLE_COMPONENTS
)
