# Note
This section documents my attempt to blink the Blue Pill LED on PC13 using bare-metal SysTick interrupts. **The code does not work — the LED does not turn on.**

I am keeping this section as a learning reference, because it highlights important concepts about SysTick, interrupts, vector tables, and register-level programming.

I cannot make it work correctly, so I decided that I will try to use **libopencm3** in a separate folder, an open-source, low-level firmware library for ARM Cortex-M microcontrollers. If anyone knows why it doesn't work or what I did wrong, I would really appreciate it.

# SysTick
SysTick is a 24-bit hardware timer inside the ARM Cortex-M core. We can see the ARM documentation here:

 https://developer.arm.com/documentation/dui0552/a/cortex-m3-peripherals/system-timer--systick

Its address is fixed by ARM architecture, and in our case, the Blue Pill is Cortex-M3. It contains 4 important registers:

| ARM manual name | Short name (CMSIS / tutorials) | Meaning               |
| --------------- | ------------------------------ | --------------------- |
| `SYST_CSR`      | `CTRL`                         | Control & status      |
| `SYST_RVR`      | `LOAD`                         | Reload value          |
| `SYST_CVR`      | `VAL`                          | Current counter value |
| `SYST_CALIB`    | `CALIB`                        | Calibration value     |

*Here are the names for each registers so we won't get confused. Most online examples and tutorials use the "short name" for readability*

We can learn from the datasheet that the base address of SysTick is `0xE000E010`. Our goal here is to:
1. Generate a `1 ms interrupt`.
2. Increment a global tick counter
3. Blink LED using time comparison (we remove the busy wait loop used in blinky).

## Step 1 - Clock
When the chip powers up, the `HSI` turns on automatically. The default `HSI Frequency` of the Blue Pill as indicated in the Datasheet is `8 Mhz`. This becames the system clock. So right after a reset:
```
HSI = 8 MHz     // HSI generates an 8 Mhz signal
SYSCLK = 8 MHz  // SYSCLK is configured to select HSI
AHB = 8 MHz     // AHB prescaler = 1, so 8/1 = 8 (we divide). For example, we changed the prescaler to "2", the CPU freq will be "4".
CPU = 8 MHz     // CPU runs from AHB
```
## Step 2 - Reload Value
We want a `1 ms period`, and the clock is `8,000,000 Hz`. We compute the counts per ms:

`8000000 / 1000 = 8000`

And the SysTick counts down to zero so `RVR = 8000 - 1`.

## Step 3 - Registers
From the ARM manual:
```
0xE000E010 → SYST_CSR  (CTRL)
0xE000E014 → SYST_RVR  (LOAD)
0xE000E018 → SYST_CVR  (VAL)
```
we define these registers in C:
```
#define CTRL (*(volatile uint32_t*)0xE000E010)
#define LOAD (*(volatile uint32_t*)0xE000E014)
#define VAL (*(volatile uint32_t*)0xE000E018)
// always remember the volatile
```

*I learned that using `volatile uint32_t` is better than `volatile unsigned int` since it guarantees we are configuring a 32-bit hardware registers*

## Step 4 - Global millisecond counter
We will use global because it is modified inside an interrupt, and will be used in main code.

```
volatile uint32_t ms_ticks = 0;
volatile uint8_t led_state = 0; // 0 = off, 1 = on
```

## Step 5 - SysTick interrupt handler
This must match the vector table entry. 
```
void SysTick_Handler(void)
{
    ms_ticks++;
}
```
This runs automatically every 1 ms. What happens in the hardware is that when the timer reaches 0, the CPU jumps to this function. The counter increments, and the CPU returns automatically. 

## Step 6 - Add to vector table
In our `startup.s`, we add:

`.word SysTick_Handler`

For Cortex-M3, SysTick is exception number 15, so it must be placed in the 16th entry. The `startup.s` should now be:
```
.syntax unified
.cpu cortex-m3
.thumb

.global _estack
.global Reset_Handler

/* 20 KB SRAM → stack top */
_estack = 0x20005000

.section .isr_vector,"a",%progbits
.word _estack
.word Reset_Handler
.word 0                /* NMI */
.word 0                /* HardFault */
.word 0
.word 0
.word 0
.word 0
.word 0
.word 0
.word 0
.word 0                /* SVC */
.word 0
.word 0
.word 0                /* PendSV */
.word SysTick_Handler  /* SysTick */

.section .text
Reset_Handler:
    bl main
    b .
```

## Step 7 - Configure SysTick
```
void systick_init(void)
{
    LOAD = 8000 - 1; // 1ms tick
    VAL = 0;          // Clear current value
    CTRL = (1U << 2) | (1U << 1) | (1U << 0); // Enable, tick interrupt, processor clock
}
```
| Bit | Name      | Meaning             |
| --- | --------- | ------------------- |
| 0   | ENABLE    | Turn SysTick on     |
| 1   | TICKINT   | Enable interrupt    |
| 2   | CLKSOURCE | Select clock source |

*In the ARM documentation, we can see that in `CLKSOURCE`, `0 = external clock` and `1 = processor clock`. The `processor clock` means the core clock (HCLK), which is the clock that drives the Cortex-M3 CPU itself.*

## Step 8 - Replace delay with non-blocking timing
The old code uses a blocking timing delay inside `main()`, while the new code separates hardware control into dedicated functions and uses a hardware timer (SysTick) for more accurate, non-blocking timing. 

```
uint32_t last = 0;

while (1)
{
    if ((ms_ticks - last) >= 500) // toggle every 500 ms
    {
        last = ms_ticks;

        if (led_state)
        {
            GPIOC_BSRR = (1 << 13);        // Turn off LED (PC13 high)
            led_state = 0;
        }
        else
        {
            GPIOC_BSRR = (1 << (13 + 16)); // Turn on LED (PC13 low)
            led_state = 1;
        }
    }
}
```

## Full C Code:
```
#include <stdint.h>

#define RCC_APB2ENR (*(volatile uint32_t*)0x40021018)
#define GPIOC_CRH (*(volatile uint32_t*)0x40011004)
#define GPIOC_BSRR (*(volatile uint32_t*)0x40011010)
#define CTRL (*(volatile uint32_t*)0xE000E010)
#define LOAD (*(volatile uint32_t*)0xE000E014)
#define VAL (*(volatile uint32_t*)0xE000E018)

volatile uint32_t ms_ticks = 0;
volatile uint8_t led_state = 0; // 0=off, 1=on

void SysTick_Handler(void)
{
    ms_ticks++;
}

void systick_init(void)
{
    LOAD = 8000 - 1; // 1ms tick @ 8 Mhz HSI
    VAL = 0;
    CTRL = (1U << 2) | (1U << 1) | (1U << 0);
}

int main(void)
{
    // Enable GPIOC clock
    RCC_APB2ENR |= (1 << 4);

    // Configure PC13 as output push-pull, 2 MHz
    GPIOC_CRH &= ~(0xF << 20);
    GPIOC_CRH |=  (0x2 << 20);

    // Turn LED off initially (high = off)
    GPIOC_BSRR = (1 << 13);

    systick_init();

    uint32_t last = 0;

    while (1)
    {
        if ((ms_ticks - last) >= 500) // 500ms toggle
        {
            last = ms_ticks;

            if (led_state)
            {
                // Turn off LED set PC13 high
                GPIOC_BSRR = (1 << 13);
                led_state = 0;
            }
            else
            {
                // Turn on LED reset PC13 low
                GPIOC_BSRR = (1 << (13 + 16));
                led_state = 1;
            }
        }
    }
}
```
## Conclusion
When running this code on the Blue Pill (STM32F103C8T6), the LED on PC13 does not turn on. **Essentially, this attempt did not produce a visible LED blink.**

*However*, There is a noteable behavior that I found in running this code. 
- If a previous program is already running (from my experience, the "blinky" where the LED blinks every 500ms) and we flash this program (systick), it blinks really fast.
- Pressing reset button clears the behavior, then the LED won't be blinking again.
- Using `st-flash erase` then run this code again, the LED still won't blink.
- I tried a working program of blinky with systick using `libopencm3`, then run this code, and the fast blinking is at it again. I am thinking that for the 'weird' fast blinking behavior only activates if there is an existing program then I run this program.

From attempting this LED Blinky with SysTick Interrupt, even if I did not managed to make the PC13 blink, I did understood how hardware timers, registers, and interrupts interact. Configuring the reload values, enabling interrupts, and the matching the vector table. Even a simple 1 ms interrupt depends on correct clock and startup setup. 
