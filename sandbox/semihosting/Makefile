CC=arm-none-eabi-gcc
CFLAGS=-mcpu=cortex-m4 -mthumb -g -O0
LDFLAGS=--specs=rdimon.specs -lc -lrdimon
TARGET=fopen_example

all: $(TARGET)

$(TARGET): fopen_example.o
	$(CC) $(LDFLAGS) $^ -o $@.elf

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o $(TARGET)

debug:
	$(DEBUGGER) $(TARGET)

