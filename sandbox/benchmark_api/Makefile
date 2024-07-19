PATH_TO_EIGEN=../../external/eigen

# Define the compilers and flags
ARM_CC = arm-none-eabi-g++
NATIVE_CC = clang++
NATIVE_CXXFLAGS = -O0 -std=c++20 -I$(PATH_TO_EIGEN) -I.

ARM_CFLAGS = -O3 -march=armv7e-m -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -nostartfiles
ARM_CXXFLAGS = $(ARM_CFLAGS) -fno-rtti -fno-exceptions -std=c++20 -I$(PATH_TO_EIGEN) -I.
ARM_LDFLAGS = -Wl,-Map=output.map,--cref --specs=nano.specs --specs=nosys.specs

DEFINES=

# Define the source files and targets
TEST_PROGRAMS = test_assembly test_pnp test_program test_eigen_program harness3 test_program2 test_program3
NATIVE_TEST_PROGRAMS = test_pnp test_eigen_program
C_SRCS = syscalls.c startup.c
C_OBJS = $(C_SRCS:.c=.o)

# Default rule to build all targets
all: $(addsuffix .elf, $(TEST_PROGRAMS)) $(addsuffix _native, $(NATIVE_TEST_PROGRAMS))

# Rule to link the object files into the final ARM executables
%.elf: %.o startup.o syscalls.o
	$(ARM_CC) $(ARM_CXXFLAGS) $^ -o $@ $(ARM_LDFLAGS)
	arm-none-eabi-objdump -d $@ > $@.dump
	c++filt < $@.dump > $@.demangled.dump

# Rule to link the object files into the final native executables
%_native: %_native.o
	$(NATIVE_CC) $(NATIVE_CXXFLAGS) $< -o $@
	llvm-objdump -d $@ > $@.dump
	c++filt < $@.dump > $@.demangled.dump

# Rule to compile the C++ source files into object files for ARM
%.o: %.cc
	$(ARM_CC) $(ARM_CXXFLAGS) -c $< -o $@

# Rule to compile the C++ source files into object files for native
%_native.o: %.cc
	$(NATIVE_CC) $(NATIVE_CXXFLAGS) -DNATIVE -c $< -o $@

# Rule to compile the C source file into object files for ARM
syscalls.o: syscalls.c
	arm-none-eabi-gcc $(ARM_CFLAGS) -c $< -o $@

startup.o: startup.c
	arm-none-eabi-gcc $(ARM_CFLAGS) -c $< -o $@

# Clean up generated files
clean:
	rm -f $(addsuffix .o, $(TEST_PROGRAMS)) $(addsuffix _native.o, $(NATIVE_TEST_PROGRAMS)) $(addsuffix .elf, $(TEST_PROGRAMS)) $(addsuffix _native, $(NATIVE_TEST_PROGRAMS)) *.dump *.demangled.dump output.map syscalls.o startup.o
