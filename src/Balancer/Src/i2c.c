/*
 * i2c.c
 *
 *  Created on: 23 окт. 2017 г.
 *      Author: SYSTEM
 */

//#include "stm32f1xx_hal.h"
#include "conf.h"

I2C_HandleTypeDef hi2c2;

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
}

uint8_t i2cRead(uint8_t Addr, uint8_t Reg, uint8_t len, uint8_t* Data)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t result = 0;

	status = HAL_I2C_Mem_Read(&hi2c2, Addr, Reg, I2C_MEMADD_SIZE_8BIT, Data, len, 0xFF);

	if(status != HAL_OK)
	{
//		Error();
		result = 0;
		Error_Handler();
	}
	else
	{
		vTaskDelay(10);
		result = 1;
	}

	return result;
}


uint8_t i2cWrite(uint16_t Addr, uint8_t Reg, uint8_t Data)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t result = 0;

	status = HAL_I2C_Mem_Write(&hi2c2, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Data, 1, 0xFF);

	if(status != HAL_OK)
	{
		result = 1;
		Error_Handler();
	}
	else
	{
		vTaskDelay(10);
		result = 0;
	}

	return result;
}


