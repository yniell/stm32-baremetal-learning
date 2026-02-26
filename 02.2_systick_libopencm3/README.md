# SysTick using libopencm3
For this section, I used the libopencm3 for peripheral access (GPIO, RCC, and SysTick). The same way we did in the previous folder, I removed the blocking delays, and replaced it with SysTick interrupt for accurate timing. The goal is to toggle LED (PC13) every 500 ms using an interrupt-driven approach. This demonstrates basic interrupt-driven programming on Cortex-M3.

I cloned the `libopencm3` repository and built it locally using `make` to generate static libraries for STM32.

`git clone https://github.com/libopencm3/libopencm3.git`

And then I navigated inside the folder `cd` and typed `make` to build the library. In the `Makefile`, I pointed it to where the folder is:

`OPENCM3_DIR = /c/Users/Rayniel/blue_pill_files/libopencm3`

## Clock
We always start with configuring the system clock. This configures the system to run from the external 8 MHz crystal (HSE) and multiplies it using the PLL to 72 MHz, which is required for accurate SysTick timing.

`rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);`

## GPIO 
Our goal again is to blink PC13, the on-board LED:

```
rcc_periph_clock_enable(RCC_GPIOC);

gpio_set_mode(GPIOC,
              GPIO_MODE_OUTPUT_2_MHZ,
              GPIO_CNF_OUTPUT_PUSHPULL,
              GPIO13);
```

Just a reminder again, the LED on the Blue Pill is active LOW. That means `Pin LOW` = `LED ON`, and `Pin HIGH` = `LED OFF`.

## SysTick Timer Setup
Since the core clock is 72 MHz, 72 000 ticks = 1 ms. We configure SysTick to generate 1 ms interrupts:

```
systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
systick_set_reload(72000 - 1); // 1 ms @ 72 MHz
systick_interrupt_enable();
systick_counter_enable();
```

Then we enable global interrupts in `int main(void)`:

`cm_enable_interrupts();`

## SysTick Interrupt Handler
The LED toggles every 500 ms using the SysTick counter. The main loop does nothing and all timing is handled by interrupt, meaning it leaves the main loop free for future tasks.
```
void sys_tick_handler(void)
{
    static uint32_t counter = 0;

    counter++;

    if (counter == 500) // 500 ms
    {
        gpio_toggle(GPIOC, GPIO13);
        counter = 0;
    }
}
```

## Full C Code:
```
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>

static void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOC);

    gpio_set_mode(GPIOC,
                  GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL,
                  GPIO13);
}

static void systick_setup(void)
{
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(72000 - 1);   // 1ms
    systick_interrupt_enable();
    systick_counter_enable();
}

void sys_tick_handler(void)
{
    static uint32_t counter = 0;

    counter++;

    if (counter == 500)
    {
        gpio_toggle(GPIOC, GPIO13);
        counter = 0;
    }
}

int main(void)
{
    clock_setup();
    gpio_setup();
    systick_setup();

    cm_enable_interrupts();

    while (1);
}
```

## Flash
We flash it by:

`st-flash write blinky.bin 0x8000000` 

## Conclusion
The output is the same as the previous blinky example, but the implementation is significantly different. In the earlier version, a busy-wait delay was used, which kept the CPU fully occupied while waiting.

With SysTick, I learned that it is a core timer inside the ARM Cortex-M CPU that runs independently of the main loop. It generates periodic interrupts based on the configured time interval and provides a precise time base for the system.

Because the timing is handled by interrupts, the CPU is free to execute other tasks in the main loop between those interrupts, making the design more efficient and scalable for future applications.

## Makefile
Previously, I mostly copy-pasted the Makefile and just focused on the how it is important as a whole. In this section, the Makefile is again not written by me, but I'd like to have a piece by piece simple explanation for further understanding. Although I plan to not delve too deep for now. Just a light explanation should suffice.
```
PROJECT = blinky
DEVICE  = stm32f103c8t6
OPENCM3_DIR = /c/Users/Rayniel/blue_pill_files/libopencm3
```
**These lines means:**
| Variable    | Meaning                     |
| ----------- | --------------------------- |
| PROJECT     | Output file name            |
| DEVICE      | Target MCU                  |
| OPENCM3_DIR | Where libopencm3 is located |

The output files of these are:
- blinky.elf
- blinky.hex
- blinky.bin

### Source Files
CFILES = main.c
OBJS   = $(CFILES:.c=.o)

This means `main.c -> main.o`. If let's say we add more files later: `CFILES = main.c gpio.c systick.c`, these will automatically be compiled.

### Compiler Flags
```
CFLAGS   += -O2 -Wall
CPPFLAGS += -MD
```

`-O2` is Optimization level 2. It makes the firmware smaller and faster.

`-Wall` show all useful warnings.

`-MD` generates .d dependency files. It allows incremental builds (only rebuilds what changed).

### Linker Flags
`LDFLAGS  += -static -nostartfiles`

`-static` ensures all required code is linked into the final ELF so the firmware is fully self-contained for bare-metal execution.

`-nostartfiles` means we do not use Linux/Windows startup code. Instead, the startup comes from `libopencm3`.

### Standard Libraries
`LDLIBS += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group`

These provide:
| Library  | Why it is needed                   |
| -------- | ---------------------------------- |
| libc     | basic C functions (`memcpy`, etc.) |
| libgcc   | compiler helper functions          |
| libnosys | dummy system calls for bare-metal  |

### Build and Clean Targets
`all: $(PROJECT).elf $(PROJECT).hex`

Running `make` builds everything.

```
clean:
	rm -f *.o *.d *.elf *.hex *.bin *.map generated.*.ld
```

If we run `make clean`, it removes all generated files so we can rebuild from scratch.

### Most Important Lines
```
include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk
```

These automatically select the correct CPU flags (-mcpu=cortex-m3, -mthumb), add the correct include paths, generate the correct linker script, and select the correct startup file. This is why the Makefile stays small. Without these, the Makefile would be much longer and harder to maintain.

### Build Rules
```
include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk
```
These provide the actual commands for: `compiling .c -> .o`, `linking .o → .elf`, and `generating .bin and .hex` so we don’t have to write them manually.

### Output Files
| File   | Purpose                 |
| ------ | ----------------------- |
| `.elf` | Debugging (used by GDB) |
| `.bin` | Raw binary for flashing |
| `.hex` | Flash format            |
| `.map` | Memory usage            |
| `.o`   | Object files            |
| `.d`   | Dependency files        |

