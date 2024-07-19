GEM5_PATH ?= $(shell echo $$GEM5_INSTALL_PATH)

TARGET_ISA=arm

# Compiler toolchain
CXX = arm-none-linux-gnueabihf-g++
CC = arm-none-linux-gnueabihf-gcc

# Compiler and linker flags
CXXFLAGS += -I$(GEM5_PATH)/include
LDFLAGS += -L$(GEM5_PATH)/util/m5/build/$(TARGET_ISA)/out -lm5

# Source files
SRCS = test_armlinux_program.cc

# Output binary
TARGET = test_armlinux_program

# Build rules
all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRCS) $(LDFLAGS)

clean:
	rm -f $(TARGET)
