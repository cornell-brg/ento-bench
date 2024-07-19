ARCH ?= arm
CROSS_COMPILE ?= arm-none-eabi-

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
#OBJS   = boot.o main.o
OBJS   = boot.o syscalls.o main.o


CFLAGS_BASE= -Wall -g -static

ifeq ($(PLATFORM),CORTEX_A8)
CORE        = cortex-a8
FPU         = vfpv3
FLOAT_ABI   = softfp
MARCH       = armv8-a
DISASSEMBLY = main-ca8-disassembly.txt
#LFLAGS      = ""
all: program

else ifeq ($(PLATFORM),CORTEX_M4)
CORE         = cortex-m4
FPU          = fpv4-sp-d16
FLOAT_ABI    = hard
MARCH        = armv7e-m
DISASSEMBLY  = main-cm4-disassembly.txt
#LFLAGS       = -L$(ARM_NONE_EABI_LIB)/thumb/v7e-m+fp/hard -lc_nano
LNK_OPT      = -static -mcpu=$(CORE) -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI) --specs=nosys.specs
all: program

else
all:
	$(error PLATFORM not set to a valid target (CORTEX_A8 or CORTEX_M4))
endif

BOOTFLAGS    = -EL
SYSCALLFLAGS = -mcpu=$(CORE) -mfloat-abi=$(FLOAT_ABI) -mfpu=$(FPU) -mthumb -fomit-frame-pointer
LNK_OPT      = -static -mcpu=$(CORE) -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI) --specs=nosys.specs 
#LNK_OPT      = -static -mcpu=$(CORE) -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI) 
LNK_SCRIPT   = boot.ld
LNK_FILE_OPT = -nostartfiles -Xlinker -T$(LNK_SCRIPT)

ifdef NANO_SPECS
CFLAGS_BASE += -DNANO_SPECS
LFLAGS       = -L$(ARM_NONE_EABI_LIB)/thumb/v7e-m+fp/hard -lc_nano -u _printf_float
LNK_OPT      = -static -mcpu=$(CORE) -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI) --specs=nosys.specs --specs=nano.specs -u _uprintf_float

OBJS         = boot.o syscalls.o main.o
endif

CFLAGS       = $(CFLAGS_BASE) -mcpu=$(CORE) -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI) -mthumb -g -I . -fno-exceptions
CXXFLAGS     = $(CFLAGS_BASE) -mcpu=$(CORE) -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI) -mthumb -g -I . -fno-exceptions

program: main.elf disassemble

main.o: main.c
	$(CC) $(CXXFLAGS) -c main.cpp -o main.o

main.elf: $(OBJS) $(LNK_SCRIPT)
	$(CC) $(LFLAGS) $(LNK_FILE_OPT) -o $@ $(OBJS) $(LNK_OPT) -lm

boot.o: Makefile
	$(CPP) -mcpu=$(CORE) -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI) boot.s | $(AS) $(BOOTFLAGS) -o boot.o

syscalls.o: syscalls.c
	$(CC) $(SYSCALLFLAGS) -c syscalls.c -o syscalls.o

disassemble: $(TARGET)
	arm-none-eabi-objdump -D $(TARGET) > $(DISASSEMBLY)

clean: 
	rm -f *.o *.elf 

