ARCH ?= arm
CROSS_COMPILE ?= arm-none-linux-gnueabihf-

AS      = $(CROSS_COMPILE)as
LD      = $(CROSS_COMPILE)ld
CC      = $(CROSS_COMPILE)gcc
CXX     = $(CROSS_COMPILE)g++
CPP     = $(CROSS_COMPILE)cpp
AR      = $(CROSS_COMPILE)ar
NM      = $(CROSS_COMPILE)nm
STRIP   = $(CROSS_COMPILE)strip
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump

TARGET = main.elf
OBJS   = main.o

CFLAGS_BASE= -Wall -g -static

# Pass PLATFORM flag to specify between building for Cortex-A8 and Cortex-M4
# gem5 does not support CM4 so this is mostly to compare objdumps of the compiled
# binaries
ifeq ($(PLATFORM),CORTEX_A8)
CORE        = cortex-a8
FPU         = vfpv3
FLOAT_ABI   = hard
MARCH       = armv8-a
DISASSEMBLY = main-ca8-disassembly.txt
#CFLAGS=$(CFLAGS_BASE) $(CFLAGS_CA8)
all: program
else ifeq ($(PLATFORM),CORTEX_M4)
CORE       = cortex-m4
FPU        = fpv4-sp-d16
FLOAT_ABI  = hard
MARCH      = armv7e-m
DISASSEMBLY = main-cm4-disassembly.txt
#CFLAGS=$(CFLAGS_BASE) $(CFLAGS_CM4)
all: program
else
all:
	$(error PLATFORM not set to a valid target (CORTEX_A8 or CORTEX_M4))
endif

CFLAGS       = $(CFLAGS_BASE) -mcpu=$(CORE) -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI) -mthumb -I . -fno-exceptions
CXXFLAGS     = $(CFLAGS_BASE) -mcpu=$(CORE) -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI) -mthumb -I . -fno-exceptions
LNK_OPT      = -static -mcpu=$(CORE) -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI) -mthumb

program: main.elf disassemble

main.o: main.c
	$(CC) $(CFLAGS) -c main.c -o main.o

$(TARGET): $(OBJS)
	$(CC) $(OBJS) $(LNK_OPT) -o $@

disassemble: $(TARGET)
	$(OBJDUMP) -D $(TARGET) > $(DISASSEMBLY)

clean: 
	rm -f *.o *.elf 

