/*
 * adc.c
 *
 *  Created on: 6 окт. 2017 г.
 *      Author: SYSTEM
 */
#include "stm32f1xx_hal.h"
#include "adc.h"

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

static volatile uint16_t adcValues[ADC_NUM_CHANNELS];

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
//    Error_Handler();
  }

    /**Configure Regular Channel
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
//    Error_Handler();
  }

    /**Configure Regular Channel
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
//    Error_Handler();
  }

    /**Configure Regular Channel
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
//    Error_Handler();
  }

    /**Configure Regular Channel
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
//    Error_Handler();
  }

  while(HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK); // Калибровка АЦП

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adcValues, ADC_NUM_CHANNELS);

}

/*****************************************************************************
 * Function:		adcGetChannel
 * ---------------------------------------------------------------------------
 * description:		Считываем значение канала АЦП
 * parameters:		uint8_t:	номер канала АЦП
 * 					Возможные значения:	ADC_1 == 0
 * 										ADC_2 == 1
 * 										ADC_3 == 2
 * 										ADC_TEMP_SENSOR == 3
 * on return:		uint16_t:	значение канала АЦП
 ****************************************************************************/
uint16_t adcGetChannel(uint8_t channel)
{
    return adcValues[channel];
}
