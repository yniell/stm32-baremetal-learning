#include <stdint.h>

#define RCC_APB2ENR   (*(volatile uint32_t*)0x40021018)
#define GPIOC_CRH     (*(volatile uint32_t*)0x40011004)
#define GPIOC_BSRR    (*(volatile uint32_t*)0x40011010)

void delay(volatile uint32_t t)
{
    while(t--);
}

int main(void)
{
    // Enable GPIOC clock
    RCC_APB2ENR |= (1 << 4);

    // Configure PC13 as output push-pull 2MHz
    GPIOC_CRH &= ~(0xF << 20);
    GPIOC_CRH |=  (0x2 << 20);

    while (1)
    {
        // LED ON (Blue Pill is inverted)
        GPIOC_BSRR = (1 << (13 + 16));
        delay(500000);

        // LED OFF
        GPIOC_BSRR = (1 << 13);
        delay(500000);
    }
}
