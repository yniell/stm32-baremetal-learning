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