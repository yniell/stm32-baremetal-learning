PROJECT = blinky

DEVICE  = stm32f103c8t6
OPENCM3_DIR = /c/Users/Rayniel/blue_pill_files/libopencm3

CFILES = main.c
OBJS   = $(CFILES:.c=.o)

CFLAGS   += -O2 -Wall
CPPFLAGS += -MD
LDFLAGS  += -static -nostartfiles
LDLIBS   += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

.PHONY: all clean

all: $(PROJECT).elf $(PROJECT).hex

clean:
	rm -f *.o *.d *.elf *.hex *.bin *.map generated.*.ld

include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk