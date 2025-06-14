# STM32 Vendor Linker Scripts

This directory contains vendor-provided linker scripts for STM32 devices that override the automatically generated ones.

## Directory Structure

```
linker-scripts/
├── h7/                    # STM32H7 family linker scripts
│   └── H7A3ZI_M7.ld      # H7A3ZI Cortex-M7 core
└── README.md              # This file
```

## How It Works

1. **Automatic Detection**: When building for STM32H7 devices, the CMake system first checks if a vendor linker script exists in the appropriate family directory.

2. **Naming Convention**: Vendor linker scripts should be named `{DEVICE}_{CORE}.ld` where:
   - `{DEVICE}` is the device name without the STM32 prefix (e.g., `H7A3ZI`)
   - `{CORE}` is the core type (e.g., `M7`, `M4`)

3. **Fallback Behavior**: If no vendor script is found, the system falls back to generating the default linker script automatically.

## Usage

To add a vendor linker script for a new device:

1. **Create the script**: Place your vendor linker script in the appropriate family directory with the correct naming convention.
   
   Example: For STM32H743ZI with M7 core, create `stm32-cmake/linker-scripts/h7/H743ZI_M7.ld`

2. **Build**: The system will automatically detect and use your vendor script during the build process.

3. **Verification**: Look for these messages in the CMake output:
   ```
   -- Using vendor linker script for H7A3ZI_M7: /path/to/vendor/script.ld
   -- Vendor linker script configured successfully for H7A3ZI_M7
   ```

## Build Process

During the build, the vendor linker script is:
1. **Detected** during CMake configuration
2. **Copied** to the build directory during the build process
3. **Used** by the linker via the `-T` flag

## Example Output

When the system successfully uses a vendor script, you'll see:
```
[  0%] Copying vendor linker script for H7A3ZI_M7
-- Using vendor linker script for H7A3ZI_M7: /path/to/stm32-cmake/linker-scripts/h7/H7A3ZI_M7.ld
```

## Supported Families

Currently supported STM32 families:
- **H7**: All H7 devices with M7 and M4 cores

To add support for other families, modify the `FindCMSIS.cmake` file to include the vendor script logic for the desired family.

## Memory Sections

The vendor scripts include special sections for different memory domains:

- `.axisram`: Large data buffers in AXI SRAM
- `.d2ram`: DMA-coherent buffers in D2 domain
- `.d3ram`: Backup domain data in D3 SRAM

Use these sections in your code with attributes:
```c
// Large buffer in AXI SRAM
__attribute__((section(".axisram"))) uint8_t large_buffer[64*1024];

// DMA buffer in D2 domain  
__attribute__((section(".d2ram"))) uint8_t dma_buffer[1024];
```

## Benefits

- **Vendor-Optimized**: Uses ST's official memory layouts and optimizations
- **Device-Specific**: Tailored to exact memory configuration of each device
- **Maintainable**: Easy to update when ST releases new versions
- **Fallback Safe**: Automatically falls back to generated scripts if vendor script is missing 