CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
CFLAGS=-mcpu=cortex-m3 -mthumb -nostdlib -Wall

all: blink.bin

blink.elf: startup.s main.c
	$(CC) $(CFLAGS) -T stm32f103.ld startup.s main.c -o blink.elf

blink.bin: blink.elf
	$(OBJCOPY) -O binary blink.elf blink.bin

clean:
	rm -f *.elf *.bin
