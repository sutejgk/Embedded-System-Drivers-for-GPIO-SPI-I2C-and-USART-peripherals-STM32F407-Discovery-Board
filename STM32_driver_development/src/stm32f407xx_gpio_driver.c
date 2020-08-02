/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 06-Jun-2019
 *      Author: Sutej Kulkarni
 */

#include "stm32f407xx_gpio_driver.h"

/*
 * Peripheral clock setup(used to enable or disable the peripheral clock for a given
 GPIO base address
 */

/***************************************************************
 * @fn                     - GPIO_PeriClockControl
 *
 * @brief                  - This function enables or disables clock for the given GPIO port
 *
 * @param[in]                  - base address of the GPIO peripheral
 * @param[in]                  - ENABLE or DISABLE macros
 * @param                  -
 *
 * @return                 - None
 *
 * @note                   - None
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDis)
{
	if (EnorDis == ENABLE)
	{
		if (pGPIOx == GPIOA)
		        {
			        GPIOA_PCLK_EN();
		        }
		else if (pGPIOx == GPIOB)
				{
					GPIOB_PCLK_EN();
				}
		else if (pGPIOx == GPIOC)
				{
					GPIOC_PCLK_EN();
				}
		else if (pGPIOx == GPIOD)
				{
					GPIOD_PCLK_EN();
				}
		else if (pGPIOx == GPIOE)
				{
					GPIOF_PCLK_EN();
				}
		else if (pGPIOx == GPIOG)
				{
					GPIOH_PCLK_EN();
				}
		else if (pGPIOx == GPIOI)
				{
					GPIOI_PCLK_EN();
				}
	}
	else
	{
		if (pGPIOx == GPIOA)
				{
					GPIOA_PCLK_DI();
				}
		else if (pGPIOx == GPIOB)
						{
							GPIOB_PCLK_DI();
						}
		else if (pGPIOx == GPIOC)
						{
							GPIOC_PCLK_DI();
						}
		else if (pGPIOx == GPIOD)
						{
							GPIOD_PCLK_DI();
						}
		else if (pGPIOx == GPIOE)
						{
							GPIOF_PCLK_DI();
						}
		else if (pGPIOx == GPIOG)
						{
							GPIOH_PCLK_DI();
						}
		else if (pGPIOx == GPIOI)
						{
							GPIOI_PCLK_DI();
						}
	}

}

/*
 * Init and Deinit
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)                       //API to initialize the given port and pin
{

//1. configure the mode of the GPIO pin(non interrupt mode)(peripheral side)

	uint32_t temp;

	//Peripheral clock enable
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_AN)
	{
		temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~((0X3) << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));  //clearing
		pGPIOHandle->pGPIOx->MODER |= temp;           //setting the required bit fields
	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//2. Clear corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//2. Clear corresponding RTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. configure both FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. Configure the GPIO port selection in SYSCFG_EXTICR register

		uint8_t temp1,temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
        uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
        SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = (portcode << (4*temp2));

		//3. Enable the EXTI interrupt delivery using IMR(interrupt mask register)
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}


	temp = 0;

	//2. configure the speed of the GPIO pin


    temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed) << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~((0X3) << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));     //clearing
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;                       //setting the required bit fields

    temp = 0;

//3.configure the pull up pull down settings

    temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl) << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~((0X3) << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));       //clearing
    pGPIOHandle->pGPIOx->PUPDR |= temp;                          //setting the required bit fields

    temp = 0;
//4. configure the O/P type

    temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType) << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~((0X1) << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));        //clearing
    pGPIOHandle->pGPIOx->OTYPER |= temp;                            //setting the required bit fields

    temp = 0;

//5. configure the alternate functionality

    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUN)
    {
    uint8_t temp1,temp2;	//Configure the alternate functionality registers
    temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
    temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
	pGPIOHandle->pGPIOx->AFR[temp1] &= ~((0XF) << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));           //clearing
    pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));        //setting the required bit fields
    }
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)                        //used to de-initialize the registers, used to reset the registers of a particluar GPIO port means sending the registers back to their reset state
{
	if (pGPIOx == GPIOA)
			        {
				        GPIOA_REG_RESET();
			        }
			else if (pGPIOx == GPIOB)
					{
						GPIOB_REG_RESET();
					}
			else if (pGPIOx == GPIOC)
					{
						GPIOC_REG_RESET();
					}
			else if (pGPIOx == GPIOD)
					{
						GPIOD_REG_RESET();
					}
			else if (pGPIOx == GPIOE)
					{
						GPIOF_REG_RESET();
					}
			else if (pGPIOx == GPIOG)
					{
						GPIOH_REG_RESET();
					}
			else if (pGPIOx == GPIOI)
					{
						GPIOI_REG_RESET();
					}
}
/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t GPIO_PinNumber)
{
   uint8_t value;
   value = (uint8_t)pGPIOx->IDR >> GPIO_PinNumber & (0x00000001);
   return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;

return 0;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t GPIO_PinNumber,uint8_t Value)
{
   if(Value == GPIO_PIN_SET)
   {
	   //write 1 to the output data register at the bit field corresponding to the pin number
	pGPIOx->ODR |= (1<<GPIO_PinNumber);
   }
   else
   {
	   //write 0(similar to above)
		pGPIOx->ODR &= ~(1<<GPIO_PinNumber);
   }

}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
pGPIOx->ODR = Value;

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t GPIO_PinNumber)
{
//pGPIOx->ODR = pGPIOx->ODR ^ (1<<GPIO_PinNumber); This is the long form(So that you understand Structures and pointers)
pGPIOx->ODR ^= (1<<GPIO_PinNumber);            //Short hand notation
}

/*
 * IRQ configuration and Handling(All the configurations in this API is Processor specific)
 * Here you are configuring NVIC registers of the ARM Cortex MX processor
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)                   // Used to configure the IRQ number of the GPIO pin,like enabling it, setting up the priority etc,
{
if(EnorDi == ENABLE)
{
    if(IRQNumber<=31)
{
//program ISER0 register
    	*NVIC_ISER0 |= (1<<IRQNumber);
}
    else if(IRQNumber > 31 && IRQNumber < 63)
{
//program ISER1 register(32 to 63)
    	*NVIC_ISER1 |= (1<<IRQNumber%32);
}
    else if(IRQNumber>=64 && IRQNumber < 96)
{
//program ISER2 register(64 to 96)
    	*NVIC_ISER2 |= (1<<IRQNumber%64);
}
else
	if(IRQNumber<=31)
	{
	//program ICER0 register
    	*NVIC_ICER0 |= (1<<IRQNumber);
	}
	else if(IRQNumber > 31 && IRQNumber < 63)
	{
	//program ICER1 register(32 to 63)
    	*NVIC_ICER1 |= (1<<IRQNumber%32);
	}
	else if(IRQNumber>=64 && IRQNumber < 96)
	{
	//program ICER2 register(64 to 96)
    	*NVIC_ICER2 |= (1<<IRQNumber%64);

	}
}

}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

void GPIO_IRQHandling(uint8_t GPIO_PinNumber)                 //Whenever the interrupt triggers, the user application can call this IRQHandling API/function in order to process that interrupt
{
//1. Clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR & (1<<GPIO_PinNumber))
	{
		//clear
		EXTI->PR |= (1<<GPIO_PinNumber);
	}

}







