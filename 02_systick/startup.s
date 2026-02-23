.syntax unified
.cpu cortex-m3
.thumb

.global _estack
.global Reset_Handler
.global SysTick_Handler

_estack = 0x20005000

/* Default handler */
.section .text.Default_Handler,"ax",%progbits
Default_Handler:
    b .

.section .isr_vector,"a",%progbits
.word _estack
.word Reset_Handler
.word Default_Handler /* NMI */
.word Default_Handler /* HardFault */
.word Default_Handler
.word Default_Handler
.word Default_Handler
.word 0
.word 0
.word 0
.word 0
.word Default_Handler /* SVC */
.word 0
.word 0
.word Default_Handler /* PendSV */
.word SysTick_Handler /* SysTick */

.section .text
Reset_Handler:
    /* Copy .data from FLASH to RAM */
    ldr r0, =_sdata
    ldr r1, =_edata
    ldr r2, =_sidata
1:
    cmp r0, r1
    ittt lt
    ldrlt r3, [r2], #4
    strlt r3, [r0], #4
    blt 1b

    /* Zero .bss */
    ldr r0, =_sbss
    ldr r1, =_ebss
    movs r2, #0
2:
    cmp r0, r1
    itt lt
    strlt r2, [r0], #4
    blt 2b

    /* Call main */
    bl main

    b .