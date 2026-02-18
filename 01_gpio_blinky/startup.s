.syntax unified
.cpu cortex-m3
.thumb

.global _estack
.global Reset_Handler

/* Stack top (20KB SRAM for STM32F103C8) */
_estack = 0x20005000

.section .isr_vector,"a",%progbits
.word _estack
.word Reset_Handler

.section .text
Reset_Handler:
    bl main
    b .
