NEC_DECODE
==========

NEC IR Protocol decoding library for ARM Cortex M series

Description
===========

It uses External Interrupt(EXTI) to capture the pulses and uses timer(TIM2 in the example) to get the duration of the high and low periods in microseconds and then it decodes those pulses into NEC_FRAME.

Usage
=====
There are several basic steps to get it working...
0: Add library files to your project
0.5: Add #include "ir_decode.h" to your project
1: Modify defines in ir_decode.h to suit your project
2: Specify EXTI handler in your project(prefferably in your main.c as it is in the example)
3: Specify TIMx handler in your project(again prefferably in your main.c)
3: Specify NEC_ReceiveInterrupt. It is the function that is called when a frame was successfully decoded.
4: Use (NEC_FRAME).Address and (NEC_FRAME).Command in your project
5: Build it
6: Run it
7: Enjoy it :D

Example
=======

Currently I have only STM32F4DISCOVERY Board so that is the only micro I tested... But as long as you have correct defines it should work ok... and if you have any problems just email me : petoknm@gmail.com

In main.c implemented:
void NEC_ReceiveInterrupt(NEC_FRAME f)
void EXTI0_IRQHandler()
void TIM2_IRQHandler()

You don't need to change anything except for the receive interrupt to do whatever you want... but the other two functions are build to be easily implemented in any project and don't look messy... they just call the library routines when needed

Bugs
====

Currently I have not noticed any... but I will be happy if you point them out to me

Author
======

Peter M.
petoknm@gmail.com
