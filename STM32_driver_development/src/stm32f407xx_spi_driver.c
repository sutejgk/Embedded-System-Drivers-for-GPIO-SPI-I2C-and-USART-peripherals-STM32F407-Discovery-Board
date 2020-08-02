/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 24-Jun-2019
 *      Author: Sutej Kulkarni
 */

#include "stm32f407xx_spi_driver.h"

//Static because they are private helper functions, so if application tries to call this, compiler will issue an error
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*
 * Peripheral clock setup(used to enable or disable the peripheral clock for a given
 GPIO base address
 */

/***************************************************************
 * @fn                     - SPI_PeriClockControl
 *
 * @brief                  - This function enables or disables clock for the given SPI port
 *
 * @param[in]                  - base address of the SPI peripheral
 * @param[in]                  - ENABLE or DISABLE macros
 * @param                  -
 *
 * @return                 - None
 *
 * @note                   - None
 *
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDis)
{
	if (EnorDis == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
				{
					SPI2_PCLK_EN();
				}
		else if (pSPIx == SPI3)
				{
					SPI2_PCLK_EN();
				}
	}
	else
	{
		if (pSPIx == SPI1)
				{
					SPI1_PCLK_DI();
				}
				else if (pSPIx == SPI2)
						{
							SPI2_PCLK_DI();
						}
				else if (pSPIx == SPI3)
						{
							SPI3_PCLK_DI();
						}
	}
}

/*
 * Init and Deinit
 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//first lets configure the SPI_CR1_register
	uint32_t tempreg = 0;

	//Peripheral clock enable

	SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);

	//1. Lets configure the device mode

	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

    //2. Configure the bus config

    if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
    	//BIDI mode should be cleared
    	tempreg &= ~(1 << SPI_CR1_BIDIMODE);
    }
    else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
    	//BIDI mode should be enabled
    	tempreg |= (1 << SPI_CR1_BIDIMODE);
    }
    else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
    {
    	//BIDI mode should be cleared and RXONLY bit must be set
    	tempreg &= ~(1 << SPI_CR1_BIDIMODE);
    	tempreg |= (1 << SPI_CR1_RXONLY);
    }

    //3. Configure the SCLK speed(baud rate control bits)
    tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR ;

    //4. Configure the data frame format

    tempreg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

    //5. Configure the CPHA

    tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA ;

    //6. Configure the CPOL

    tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL ;

    //7. Configure the SSM

    tempreg |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM ;

    pSPIHandle->pSPIx->CR1 = tempreg;                //Here you can use assignment operator as you are freshly initializing the CR1 register


}


void SPI_DeInit(SPI_RegDef_t *pSPIx);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Data send and receive
 */

//This is a blocking call or polling based code or polling type API function(another way to way to implement this is interrupt based, which is non blocking call)
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)    //Len - no of bytes to be transmitted,*pTxBuffer is a pointer to the data, *pSPIX-base address of SPI
{

	while(Len>0)
	{
		//1. Wit for TX buffer to be empty
		//one way to wait is to use while loop :   while(!(pSPIx->SR & 1<<1));
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. Check the DFF bit in CR1
		if((pSPIx->CR1) & (1 << SPI_CR1_DFF))
		{
			//16-bit DFF
			//1. Load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//8-bit DFF
			pSPIx->DR = *pTxBuffer;
	        Len--;
	        pTxBuffer++;
		}


	}

}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len>0)
		{
			//1. Wait for RXNE to be set
			//one way to wait is to use while loop :   while(!(pSPIx->SR & 1<<1));
			while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

			//2. Check the DFF bit in CR1
			if((pSPIx->CR1) & (1 << SPI_CR1_DFF))
			{
				//16-bit DFF
				//1. Load the data from DR to Rxbuffer address
				*((uint16_t*)pRxBuffer) = pSPIx->DR ;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}
			else
			{
				//8-bit DFF
				*(pRxBuffer) = pSPIx->DR ;
		        Len--;
		        pRxBuffer++;
			}


		}
}


/*
 * IRQ Configuration and IRQ Handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)   // Used to configure the IRQ number of the SPI pin,like enabling it, setting up the priority etc
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	//1. first lets find out the ipr register
		uint8_t iprx = IRQNumber / 4;
		uint8_t iprx_section  = IRQNumber %4 ;

		uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

		*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}

/*
 * Other Peripheral Control APIs
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIEnable(SPI_RegDef_t *pSPIx,uint8_t EnorDis)

{
	if(EnorDis == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDis)
{
	if (EnorDis == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
	    pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
	//1. Save the Tx buffer address and len information in some global variables
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = Len;

	//2. Mark the SPI state as busy in transmission so that no other code can
	//   take over same SPI peripheral until transmission is over

	pSPIHandle->TxState = SPI_BUSY_IN_TX;

	//3. Enable TXEIE control bit in CR2 register to get an interrupt whenever TXE flag bit is set in SR

   pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	//4. Data transmission will be handled by the ISR
	}

	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

		if(state != SPI_BUSY_IN_RX)
		{
		//1. Save the Rx buffer address and len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in reception so that no other code can
		//   take over same SPI peripheral until reception is over

		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable RXEIE control bit in CR2 register to get an interrupt whenever RXE flag bit is set in SR

	   pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
		//4. Data transmission will be handled by the ISR
		}

		return state;

}


void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1,temp2;

	//First lets check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
       //handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//Second lets check for RXNE
		temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
		temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	//Third lets check for OVR(overrun error)
	   temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	   temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

		if(temp1 && temp2)
		{
			//handle RXNE
			spi_ovr_err_interrupt_handle(pHandle);
		}

}

//some helper function implementations
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if((pSPIHandle->pSPIx->CR1) & (1 << SPI_CR1_DFF))
			{
				//16-bit DFF
				//1. Load the data into the DR
				pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
				pSPIHandle->TxLen--;
				pSPIHandle->TxLen--;
				(uint16_t*)pSPIHandle->pTxBuffer++;
			}
			else
			{
				//8-bit DFF
				pSPIHandle->pSPIx->DR = pSPIHandle->pTxBuffer;
				pSPIHandle->TxLen--;
		        pSPIHandle->pTxBuffer++;
			}

	if(!pSPIHandle->TxLen)
	{
		//TxLen is 0, so close the SPI transmission and inform the application that TX is over
		//This prevents interrupts from setting up of TXE flag
		CloseTransmission(pSPIHandle);
		SPIApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);


	}

}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//2. Check the DFF bit in CR1
				if((pSPIHandle->pSPIx->CR1) & (1 << SPI_CR1_DFF))
				{
					//16-bit DFF
					//1. Load the data from DR to Rxbuffer address
					*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR ;
					pSPIHandle->RxLen--;
					pSPIHandle->RxLen--;
					(uint16_t*)pSPIHandle->pRxBuffer++;
				}
				else
				{
					//8-bit DFF
					*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR ;
					pSPIHandle->RxLen--;
			        pSPIHandle->pRxBuffer++;
				}


		if(!pSPIHandle->RxLen)
		{
			//RxLen is 0,reception is complete so close the SPI reception and inform the application that RX is over
			//This prevents interrupts from setting up of RXNE flag, lets turn off the RXNEIE interrupt
			CloseReception(pSPIHandle);
			SPIApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);


		}


}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    uint8_t temp;

	//1. Clear the OVR flag
    if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
    {
    temp = pSPIHandle->pSPIx->DR;
    temp = pSPIHandle->pSPIx->SR;
    }

	SPIApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);


}

void CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
    temp = pSPIx->DR;
    temp = pSPIx->SR;
}

__weak void SPIApplicationEventCallback(SPI_Handle_t *pSPIHandle_t,uint8_t AppEv)
{
    //This is a weak implementation. The application may override this function.

}


