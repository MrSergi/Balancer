//******************************************************************************
//  Секция include: здесь подключается заголовочный файл к модулю
//******************************************************************************

#include "f1_i2c.h"
#include "f1_gpio.h"

//******************************************************************************
//  Секция определения переменных, используемых в модуле
//******************************************************************************

//------------------------------------------------------------------------------
// Глобальные
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// Локальные
//------------------------------------------------------------------------------

static i2c_cfg_t * i2c_p;
static uint8_t     i2c_num = 0;
static I2C_HandleTypeDef hi2c[2];

//******************************************************************************
//  Секция прототипов локальных функций
//******************************************************************************



//******************************************************************************
//  Секция описания функций (сначала глобальных, потом локальных)
//******************************************************************************

/*******************************************************************************
*  Function:       i2cConfig
*-------------------------------------------------------------------------------
*  description:
*  parameters:     void
*  on return:      void
*******************************************************************************/
void i2cConfig(const i2c_cfg_t * cfg, uint8_t num)
{
	i2c_p = cfg;
	i2c_num = num;
}

/*******************************************************************************
*  Function:       i2cInit
*-------------------------------------------------------------------------------
*  description:
*  parameters:     void
*  on return:      void
*******************************************************************************/
int32_t i2cInit(uint8_t i2c_id)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	if (i2c_id >= i2c_num)
		return -1;	// Не допустимый индекс порта

	if (i2c_p == NULL)
		return -2;	// Драйвер не инициализирован. Нет вызова uartConfig().

	hi2c[i2c_id].Instance = i2c_p[i2c_id].regs;
    hi2c[i2c_id].Init.ClockSpeed = i2c_p[i2c_id].ClkSpeed;
    hi2c[i2c_id].Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c[i2c_id].Init.OwnAddress1 = 0;
    hi2c[i2c_id].Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c[i2c_id].Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c[i2c_id].Init.OwnAddress2 = 0;
    hi2c[i2c_id].Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c[i2c_id].Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if(hi2c[i2c_id].State == HAL_I2C_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
	  hi2c[i2c_id].Lock = HAL_UNLOCKED;


	  gpioClockEnable(i2c_p[i2c_id].scl_gpio_port);
	  gpioClockEnable(i2c_p[i2c_id].sda_gpio_port);

	  GPIO_InitStruct.Pin   = i2c_p[i2c_id].scl_gpio_pin;                               /* Configure gpio pin */
	  GPIO_InitStruct.Mode  = GPIO_MODE_AF_OD;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(i2c_p[i2c_id].scl_gpio_port, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin   = i2c_p[i2c_id].sda_gpio_pin;
	  HAL_GPIO_Init(i2c_p[i2c_id].sda_gpio_port, &GPIO_InitStruct);

    /* Init the low level hardware : GPIO, CLOCK, NVIC */
//    HAL_I2C_MspInit(&hi2c[i2c_id]);
  }

  if (HAL_I2C_Init(&hi2c[i2c_id]) != HAL_OK)
  {
    Error_Handler();
  }

  return 0;
}

/*******************************************************************************
*  Function:       i2cClockEnable
*-------------------------------------------------------------------------------
*  description:
*  parameters:     void
*  on return:      void
*******************************************************************************/
void i2cClockEnable(I2C_TypeDef  *I2Cx)
{
	if (I2Cx == I2C1)
	{
		__HAL_RCC_I2C1_CLK_ENABLE();
		vTaskDelay(100);
		__HAL_RCC_I2C1_FORCE_RESET();
		vTaskDelay(100);
		__HAL_RCC_I2C1_RELEASE_RESET();
		vTaskDelay(100);
	}
	else if (I2Cx == I2C2)
		{
			__HAL_RCC_I2C2_CLK_ENABLE();
			vTaskDelay(100);
			__HAL_RCC_I2C2_FORCE_RESET();
			vTaskDelay(100);
			__HAL_RCC_I2C2_RELEASE_RESET();
			vTaskDelay(100);
		}
}
//void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
//{
//
//  GPIO_InitTypeDef GPIO_InitStruct;
//  if(hi2c->Instance==I2C2)
//  {
//    /**I2C2 GPIO Configuration
//    PB10     ------> I2C2_SCL
//    PB11     ------> I2C2_SDA
//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//  }
//}
//
//void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
//{
//
//  if(hi2c->Instance==I2C2)
//  {
//    __HAL_RCC_I2C2_CLK_DISABLE();
//
//    /**I2C2 GPIO Configuration
//    PB10     ------> I2C2_SCL
//    PB11     ------> I2C2_SDA
//    */
//    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);
//  }
//
//}

/*************************************************************
*  Function:       i2cRead
*------------------------------------------------------------
*  description:    Принимает байта данных по шине I2C
*  parameters:     Addr - адрес устройства
*                  Reg - адрес регистра
*                  Data - данные для записи
*  on return:      uint_8 - результат записи
*                  1 - ошибка, 0 - успех
*************************************************************/
uint8_t i2cRead(uint8_t Addr, uint8_t Reg, uint8_t len, uint8_t* Data)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t result = 0;

	status = HAL_I2C_Mem_Read(&hi2c[0], Addr, Reg, I2C_MEMADD_SIZE_8BIT, Data, len, 0xFF);

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

/*************************************************************
*  Function:       i2cWrite
*------------------------------------------------------------
*  description:    Запись байта данных по шине I2C
*  parameters:     Addr - адрес устройства
*                  Reg - адрес регистра
*                  Data - данные для записи
*  on return:      uint_8 - результат записи
*                  1 - ошибка, 0 - успех
*************************************************************/
uint8_t i2cWrite(uint16_t Addr, uint8_t Reg, uint8_t Data)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t result = 0;

	status = HAL_I2C_Mem_Write(&hi2c[0], Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Data, 1, 0xFF);

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

//******************************************************************************
//  ENF OF FILE
//******************************************************************************
