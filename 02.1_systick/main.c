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