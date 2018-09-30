#include "uart.h"
#include "string.h"
#include "stm32f1xx_hal.h"
#include "console.h"



void prv_SendChar(uint8_t data);



/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
;

void UART_Init(void)
{

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart1);

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

}



void UART_SendString(const char *str)
{
//	xSemaphoreTake(xTxSemaphore, portMAX_DELAY);
//	int i = 0;
//	while (str[i] != 0)
//	{
//		prv_SendChar(str[i]);
//		i++;
//	}
//	HAL_UART_Transmit_IT(&huart1, str, strlen(str));
	HAL_UART_Transmit(&huart1, str, strlen(str), 500);
//	xSemaphoreGive(xTxSemaphore);
}


/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{

  HAL_UART_IRQHandler(&huart1);
//
  uint8_t x;
//
//  HAL_UART_Receive_IT(&huart1, &x, 1); // принимаем символ
//
//  consoleInput(&x, 1);
    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
    {

        HAL_UART_Receive(&huart1, &x, 1, 500); // получаем символ
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

//        x = (uint8_t)(USART1->RDR);
//    	HAL_UART_Receive_IT(&huart1, &x, 1); // принимаем символ
//        c_collback[9] = x;
//        HAL_UART_Transmit(&huart1, c_collback, sizeof(c_collback)-1, 500);
//          HAL_UART_Transmit(&huart1, &x, 1, 500);
          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
          consoleInput(&x, 1);


    }

}

