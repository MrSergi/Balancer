//******************************************************************************
//  Секция include
//******************************************************************************

#include "sensors.h"

//******************************************************************************
//  Секция определения переменных, используемых в модуле
//******************************************************************************

//------------------------------------------------------------------------------
// Глобальные
//------------------------------------------------------------------------------

acc_sensor_t AccSensor;			// Structure for real time accel sensor data
gyro_sensor_t GyroSensor;		// Structure for real time gyro sensor data
float pAngle = 0.0;				// Значение, пропорциональное углу отклонения
uint8_t period = 0;

//------------------------------------------------------------------------------
// Локальные
//------------------------------------------------------------------------------

//******************************************************************************
//  Секция прототипов локальных функций
//******************************************************************************

static void SensUpdateAcc(void);
static void SensUpdateGyro(void);
static float clamp(float v, float minv, float maxv);

//******************************************************************************
//  Секция описания функций (сначала глобальных, потом локальных)
//******************************************************************************

/*************************************************************
*  Function:       SensGetTempCPU
*------------------------------------------------------------
*  description:    Считывание температуры микроконтроллера
*  parameters:     void
*  on return:      CPU temperature in 0.1 degrees celsius
*************************************************************/
int16_t SensGetTempCPU(void)
{
	float Temp_Voltage = adcGetChannel(ADC_TEMP_SENSOR) * 3.3 / 4095.0;
	const float STM32_TEMP_V25 = 1.43; 			/* V */
	const float STM32_TEMP_AVG_SLOPE = 4.3; 	/* mV/C */
	return ((STM32_TEMP_V25 - Temp_Voltage) * 1000.0 / STM32_TEMP_AVG_SLOPE + 25.0) * 10;
}

/*************************************************************
*  Function:       SensInit
*------------------------------------------------------------
*  description:    Инициализация и конфигурация датчиков
*  parameters:     void
*  on return:      void
*************************************************************/
void SensInit(void)
{
	uint8_t error = 0;

	if (!l3gd20Detect())   // настраиваем гироскоп
	{
		// failureMode(3); // if this fails, we get a beep + blink pattern. we're doomed, no gyro or i2c error.
		error = 1;
	}

	lsm303dlhcConfig();    // настраиваем акселерометр и магнетометр
}

/*************************************************************
*  Function:       SensUpdateAcc
*------------------------------------------------------------
*  description:    Обновляет значения угловой скорости
*                  от гироскопа по 3-м осям
*  parameters:     void
*  on return:      void
*************************************************************/
static void SensUpdateGyro(void)
{
	int16_t gyro[3];

	l3gd20Read(gyro);

	GyroSensor.DataDeg[0] = gyro[0] * 0.07f;    // Gyro FS=2000 dps, Sensitivity = 70 mdps/lsb (L3GD20);
	GyroSensor.DataDeg[1] = gyro[1] * 0.07f;
	GyroSensor.DataDeg[2] = gyro[2] * 0.07f;

	GyroSensor.Angle[0] = GyroSensor.Angle[0] + GyroSensor.DataDeg[0] * 0.02; // расчёт угла отклонения по данным гироскопа

	static uint8_t temp_counter = 0;            // Temperature sensor refresh rate 1 Hz. Obtain through 50 cycles

	if (temp_counter++ == 50)
	{
		int16_t t;

		temp_counter = 0;

		if (l3gd20GetTemp(&t))
			GyroSensor.Temp = -7.87 * t + 332.3; // Convert to 0.1 degrees of Celcius
	}
}


/*************************************************************
*  Function:       clamp
*------------------------------------------------------------
*  description:    Не даёт значению выйти за установленные пределы
*  parameters:     v - проверяемое значение
*                  minv - минимально возможное значение
*                  maxv - максимально возможное значение
*  on return:      float - значение после проверки
*************************************************************/
static float clamp(float v, float minv, float maxv)
{
   if(v > maxv)
      return maxv;
   else if(v < minv)
      return minv;
   return v;
}

/*************************************************************
*  Function:       SensUpdateAcc
*------------------------------------------------------------
*  description:    Обновляет значения ускорений от акселерометра
*                  по 3-м осям
*  parameters:     void
*  on return:      void
*************************************************************/
static void SensUpdateAcc(void)
{
	int16_t accel[3];

	lsm303dlhcReadAcc(accel);

	AccSensor.DataMSS[0] = accel[0] * 0.0392266;  // Scale to the m/s2.
	AccSensor.DataMSS[1] = accel[1] * 0.0392266;  // acc_scale: 0,004 * 9,80665 = 0,0392266 (m/s2/lsb)
	AccSensor.DataMSS[2] = accel[2] * 0.0392266;

//	accel[1] = accel[1] * 0.004

	AccSensor.Angle[0] = atan2f(accel[1] * 0.004, accel[2] * 0.004) * 180.0 / 3.1415; // вычисляем угол от акселерометра
//	angle_ax = 90 - TO_DEG*acos(ay);
//	AccSensor.Angle[0] = 90.0 - TO_DEG * acos(accel[1] * 0.004); // вычисляем угол наклона по акселерометру

    static uint8_t TempCounter = 0;               // Temperature sensor refresh rate 1 Hz. Obtain through 50 cycles

    if (TempCounter++ == 50)
    {
    	int16_t t;

    	TempCounter = 0;

    	if (GetTemperature(&t))
    	{
    		AccSensor.Temp = 1.149 * t + 170;	  // Convert to 0.1 degrees of Celcius
    	}
    }
}

//******************************************************************************
//  Секция описания задач ОС
//******************************************************************************

portTASK_FUNCTION_PROTO(SensorTask, pvParameters)
{
	portTickType xLastWakeTime;
	const float FK = 0.2;
	uint32_t SysTime = 0;
	static uint32_t pTime = 0;
	uint32_t dTime = 1;

	/* Sensors already initialized in main.c */

	xLastWakeTime = xTaskGetTickCount();     // Initialise the xLastWakeTime variable with the current time.

    while (1)
    {
//    	SensUpdateAcc();                     // Read accel sensor data each cycle

//    	SensUpdateGyro();                    // Read gyro sensor data each cycle

//    	pAngle = GyroSensor.Angle[0] * (1.0 - FK) + AccSensor.Angle[0] * FK;
    	//angle_gx = angle_gx*(1-FK) + angle_ax*FK;
		//pAngle=0.98*(pAngle+gyroAngle)+0.02*accAngle-desiredAngle;               // Complementary filter yields best value for current angle

//		if(period)
//			period = 0;
//		else
//			period = 10;
//
//		SysTime = HAL_GetTick();
//    	dTime = SysTime - pTime;
//    	pTime = SysTime;

//    	HAL_Delay(1000); // задержка в 1000 мс
    	vTaskDelay(1000);
//    	CDC_Transmit_FS(&dTime, 4);

    	vTaskDelayUntil(&xLastWakeTime, 1000); // Wait for the next cycle. Task cycle time 20 ms.
//		vTaskDelay(1000);
//		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}

//******************************************************************************
//  ENF OF FILE
//******************************************************************************
