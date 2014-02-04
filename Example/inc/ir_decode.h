#ifndef __IR_DECODE_H
#define __IR_DECODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_conf.h"
#include <stdio.h>
#include "nec_decode.h"

#define IR_GPIO_PORT            GPIOA                    /*!< Port which IR output is connected */
#define IR_GPIO_PORT_CLK        RCC_AHB1Periph_GPIOA      /*!< IR pin GPIO Clock Port */
#define IR_GPIO_PIN             GPIO_Pin_0               /*!< Pin which IR is connected */
#define IR_GPIO_SOURCE          GPIO_PinSource0
#define IR_GPIO_PORT_SOURCE		GPIO_PortSourceGPIOA
#define IR_EXTI_LINE			EXTI_Line0
#define IR_NVIC_IRQn			EXTI0_IRQn
#define IR_EXTI_PORT_SOURCE		EXTI_PortSourceGPIOA
#define IR_EXTI_PIN_SOURCE		EXTI_PinSource0

#ifdef __cplusplus
}
#endif

#endif
