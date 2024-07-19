# Compiler and linker
CXX = arm-none-eabi-g++
LD = arm-none-eabi-ld

# Paths
CMSIS_PATH = CMSIS_6/CMSIS
CMSIS_DSP_PATH = $(CMSIS_PATH)/DSP
CMSIS_CORE_PATH = $(CMSIS_PATH)/Core
CMSIS_INCLUDE = $(CMSIS_PATH)/Include
CMSIS_CORE_INCLUDE = $(CMSIS_CORE_PATH)/Include
CMSIS_DSP_INCLUDE = $(CMSIS_DSP_PATH)/Include
EIGEN_INCLUDE = ../../external/eigen

# Compiler flags
MCPU=cortex-m4
MFPU=fpv4-sp-d16
MFLOAT_ABI=hard
CPUFLAGS=-mcpu=$(MCPU) -mthumb -mfpu=$(MFPU) -mfloat-abi=$(MFLOAT_ABI)
CXXFLAGS = -std=c++20 -Wall -O3 $(CPUFLAGS) -I$(CMSIS_INCLUDE) -I$(CMSIS_DSP_INCLUDE) -I$(CMSIS_CORE_INCLUDE) -I$(EIGEN_INCLUDE)

# Linker flags
LDFLAGS = -L$(CMSIS_DSP_PATH) -lm -larm_cortexM4lf_math --specs=nosys.specs --specs=nano.specs $(CPUFLAGS)

# Source files
SRCS = main.cc # FixedPoint.hh EigenFixedPoint.hh

# Object files
OBJS = $(SRCS:.cc=.o)

# Executable
TARGET = main.elf

# Default rule
all: $(TARGET)

# Link
$(TARGET): $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS) -o $(TARGET)

# Compile
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean
clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean

