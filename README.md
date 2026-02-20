# stm32-baremetal-learning
Register-level firmware development on the STM32F103C8T6 (Blue Pill).

## Overview

This repository documents my journey in learning bare-metal programming on the STM32F103C8T6.

My goal is to deeply understand how microcontroller peripherals work internally by programming them directly through memory-mapped registers — without using HAL or high-level abstraction libraries.

## Objectives
1. Understand the ARM Cortex-M3 core
2. Learn STM32 memory map
3. Configure peripherals using registers
4. Implement timing using SysTick
5. Build from GPIO → interrupts → timers → communication peripherals

## Hardware
- MCU: STM32F103C8T6 (Blue Pill)
- Core: ARM Cortex-M3

## References
- RM0008 Reference Manual
- ARM Cortex-M3 Technical Reference Manual
- STM32F103C8T6 Datasheet

*Note: I'm a beginner and this repository is part of my self-learning journey. I would really appreciate it if you could point out any mistakes you may find, and provide me suggestions and improvements.*
