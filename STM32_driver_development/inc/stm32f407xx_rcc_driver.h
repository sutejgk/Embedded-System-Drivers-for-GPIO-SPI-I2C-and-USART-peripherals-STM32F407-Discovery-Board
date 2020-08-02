/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: 30-Jul-2019
 *      Author: Sutej Kulkarni
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32f407xx.h"

//Returns APB1 CLK value
uint32_t RCC_GetPCLK1Value(void);

//Returns APB2 CLK value
uint32_t RCC_GetPCLK2Value(void);

#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
