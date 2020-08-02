/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 22-Jul-2019
 *      Author: Sutej Kulkarni
 */


#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_rcc_driver.h"

static void I2CGenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2CGenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);


static void I2CGenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2CGenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
SlaveAddr = SlaveAddr <<1 ;
SlaveAddr &= ~(1);            // slave address + r/w bit = 0(w)
pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr <<1 ;
	SlaveAddr |= 1;            // slave address + r/w bit = 1(r)
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
    (void)dummyRead;
}

void I2C_ManageACKing(I2C_RegDef_t *pI2Cx,uint8_t EnorDis)
{
	if (EnorDis == I2C_ACK_ENABLE)
	{
		//Enable the ack
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		//Disable the ack
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}


uint32_t RCC_GetPLLOPClock(void)
{
	return 0;
}

/*
 * Function to get RCCPCLK1 frequency
 */



void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDis)
{
	if (EnorDis == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}

	else
	{
			if (pI2Cx == I2C1)
			{
				I2C1_PCLK_DI();
			}
			else if(pI2Cx == I2C2)
			{
				I2C2_PCLK_DI();
			}
			else if(pI2Cx == I2C3)
			{
				I2C3_PCLK_DI();
			}
		}

}



void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg=0;

	//Enable peripheral clock FOR I2C

	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

    //ack control bit
	tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << 10);
	pI2CHandle->pI2Cx->CR1 = tempreg ;

	//Configure the freq field of CR2

	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//Configure the OAR1 register(program the device own address)

	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1<<14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

    //CCR calculations
	uint16_t ccr_value=0;
	tempreg=0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_Speed_SM)
	{
		//this is standard mode
		ccr_value = (RCC_GetPCLK1Value()/2)/pI2CHandle->I2C_Config.I2C_SCLSpeed ;
		tempreg = (ccr_value & 0XFFF);
	}
	else
	{
		//This is fast mode
		//1. Enable the Fm Mode bit in CCR
        tempreg |= (1<<15);
        tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

        if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
        {
        	ccr_value = (RCC_GetPCLK1Value()/3)/pI2CHandle->I2C_Config.I2C_SCLSpeed;
        }
        else
        {
            ccr_value = (RCC_GetPCLK1Value()/25)/pI2CHandle->I2C_Config.I2C_SCLSpeed;
        }
        tempreg |= ccr_value & 0xFFF;

	}
	pI2CHandle->pI2Cx->CCR = tempreg ;

	//TRISE calculation
     tempreg=0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_Speed_SM)
		{
			//this is standard mode
            tempreg = (RCC_GetPCLK1Value()/1000000U) + 1 ;
		}
		else
		{
			//this is fastmode
			tempreg = ((RCC_GetPCLK1Value()*300)/(1000000000U)) + 1 ;
		}

	pI2CHandle->pI2Cx->TRISE= tempreg & 0x3F;

}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{

}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName)
{
if (pI2Cx->SR1 & FlagName)
{
	return FLAG_SET;
}
	return FLAG_RESET;
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint32_t Len,uint8_t SlaveAddr)
{
//1. Generate the start condition

	I2CGenerateStartCondition(pI2CHandle->pI2Cx);

//2. Confirm that the start generation is completed by checking the SB flag in the SR1 register
	//NOTE: Until SB is cleared, SCL will be stretched(pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

//3. Send the address of the slave with r/w bit set to w(0) (total 8 bits)

	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

//4. Wait until address phase is completed by checking the ADDR flag in the SR1 register

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

//5. Clear the ADDR flag according to its software sequence
	//NOTE: Until ADDR is cleared, SCL will be stretched(pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

//6. Send the data until Len becomes 0

while(Len>0)
{
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE)));
	pI2CHandle->pI2Cx->DR = *pTxbuffer;                               //need to figure this out
	pTxbuffer++;
	Len--;
}

//7. When length becomes 0 wait for TXE=1 AND BTF = 1 before generating the stop condition
//NOTE: TXE =1 AND BTF=1 means that both SR and DR are empty and next transmission should begin
// when BTF=1 SCL will be stretched (pulled to LOW)

while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE)));
while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF)));

//8. Generate STOP condition and master need not to wait for the completion of stop condition
// NOTE: Generating STOP, automatically clears the BTF

I2CGenerateStopCondition(pI2CHandle->pI2Cx);


}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer,uint32_t Len,uint8_t SlaveAddr)
{

	// 1. Generate the start condition

	I2CGenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that the start generation is completed by checking the SB flag in the SR1 register
		//NOTE: Until SB is cleared, SCL will be stretched(pulled to LOW)

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	//3. Send the address of the slave with r/w bit set to w(0) (total 8 bits)

	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	//4. Wait until address phase is completed by checking the ADDR flag in the SR1 register

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//procedure to read only 1 byte from slave

	if(Len==1)
	{
		//Disable ACKing
		I2C_ManageACKing(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//generate stop condition

		I2CGenerateStopCondition(pI2CHandle->pI2Cx);

		//Clear the ADDR flag

		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//Wait until RXNE becomes 1

		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE)));

		//read data in to buffer

		*pRxbuffer = pI2CHandle->pI2Cx->DR;

		return;
	}

	//procedure to read data from slave when Len >1

	if(Len > 1)
	{
		//Clear the ADDR flag

		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//Read the data until Len becomes 0
		for(uint32_t i = Len; i >0 ; i--)
		{

			//Wait until RXNE becomes 1

			while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE)));

			if(i==2) //if last 2 byte are remaining
			{
				        //Disable ACKing
						I2C_ManageACKing(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				        //generate stop condition

						I2CGenerateStopCondition(pI2CHandle->pI2Cx);

			}

            //read the data from data register into buffer

			*pRxbuffer = pI2CHandle->pI2Cx->DR;

			//Increment the buffer address

			pRxbuffer++;

		}

	}

	//Re-enable the ACKing

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
       I2C_ManageACKing(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}
}




void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
	{
		pI2Cx->CR1 |= (1<<I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1<<I2C_CR1_PE);
	}
}


/*uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition

		I2CGenerateStartCondition(pI2CHandle->pI2Cx);


		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);



	}

	return busystate;

}


uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition

				I2CGenerateStartCondition(pI2CHandle->pI2Cx);


				//Implement the code to enable ITBUFEN Control Bit
				pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

				//Implement the code to enable ITEVFEN Control Bit
		        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

				//Implement the code to enable ITERREN Control Bit
		        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}

	return busystate;
}


*/



