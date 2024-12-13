//******************************************************************************
//  ������ include: ����� ������������ ������������ ���� � ������
//******************************************************************************

#include "tim.h"

//******************************************************************************
//  ������ ����������� ����������, ������������ � ������
//******************************************************************************

//------------------------------------------------------------------------------
// ����������
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// ���������
//------------------------------------------------------------------------------

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

//******************************************************************************
//  ������ ���������� ��������� �������
//******************************************************************************

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle);

//******************************************************************************
//  ������ �������� ������� (������� ����������, ����� ���������)
//******************************************************************************



/*******************************************************************************
*  Function:       MX_TIM1_Init
*-------------------------------------------------------------------------------
*  description:    TIM1 init function
*  parameters:     void
*  on return:      void
*******************************************************************************/
void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 720-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
}

/*******************************************************************************
*  Function:       MX_TIM4_Init
*-------------------------------------------------------------------------------
*  description:    TIM4 init function
*  parameters:     void
*  on return:      void
*******************************************************************************/
void MX_TIM4_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7200 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
     Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
	 Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
     Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
	 Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_3);
}

/*******************************************************************************
*  Function:       HAL_TIM_Base_MspInit
*-------------------------------------------------------------------------------
*  description:    TIM1 init function
*  parameters:     void
*  on return:      void
*******************************************************************************/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();                   /* Peripheral clock enable */
  }
}

/*******************************************************************************
*  Function:       HAL_TIM_PWM_MspInit
*-------------------------------------------------------------------------------
*  description:    TIM1 init function
*  parameters:     void
*  on return:      void
*******************************************************************************/
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{
  if(tim_pwmHandle->Instance==TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();                   /* TIM4 clock enable */
  }
}

/*******************************************************************************
*  Function:       HAL_TIM_MspPostInit
*-------------------------------------------------------------------------------
*  description:    TIM1 init function
*  parameters:     void
*  on return:      void
*******************************************************************************/
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(timHandle->Instance==TIM1)
  {
    /**TIM1 GPIO Configuration
    PB14     ------> TIM1_CH2N
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  else if(timHandle->Instance==TIM4)
  {
    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1
    PB8     ------> TIM4_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

/*******************************************************************************
*  Function:       HAL_TIM_Base_MspDeInit
*-------------------------------------------------------------------------------
*  description:    TIM1 init function
*  parameters:     void
*  on return:      void
*******************************************************************************/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_DISABLE();                 /* Peripheral clock disable */
  }
}

/*******************************************************************************
*  Function:       HAL_TIM_PWM_MspDeInit
*-------------------------------------------------------------------------------
*  description:    TIM1 init function
*  parameters:     void
*  on return:      void
*******************************************************************************/
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{
  if(tim_pwmHandle->Instance==TIM4)
  {
    __HAL_RCC_TIM4_CLK_DISABLE();                 /* Peripheral clock disable */
  }
}


void TimPWMSetPulseValue(uint8_t Channel, uint16_t PulseValue)
{
	switch(Channel) {
		case 1:
			htim4.Instance->CCR1 = PulseValue;
			break;
		case 3:
			htim4.Instance->CCR3 = PulseValue;
			break;
		default:
			break;
	}
}

//******************************************************************************
//  ENF OF FILE
//******************************************************************************
