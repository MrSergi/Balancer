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

//------------------------------------------------------------------------------
// Локальные
//------------------------------------------------------------------------------

//******************************************************************************
//  Секция прототипов локальных функций
//******************************************************************************

static void SensUpdateAcc(void);
static void SensUpdateGyro(void);

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
*  description:    Обновляем значения угловой скорости
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


	static uint8_t temp_counter = 0;            // Temperature sensor refresh rate 1 Hz. Obtain through 20 cycles

	if (temp_counter++ == 20)
	{
		int16_t t;

		temp_counter = 0;

		if (l3gd20GetTemp(&t))
			GyroSensor.Temp = -7.87 * t + 332.3; // Convert to 0.1 degrees of Celcius
	}
}

/*************************************************************
*  Function:       SensUpdateAcc
*------------------------------------------------------------
*  description:    Обновляем значения ускорений от акселерометра
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

    static uint8_t TempCounter = 0;               // Temperature sensor refresh rate 1 Hz. Obtain through 20 cycles

    if (TempCounter++ == 20)
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

	/* Sensors already initialized in main.c */

	xLastWakeTime = xTaskGetTickCount();     // Initialise the xLastWakeTime variable with the current time.

    while (1)
    {
    	SensUpdateAcc();                     // Read accel sensor data each cycle

    	SensUpdateGyro();                    // Read gyro sensor data each cycle

		vTaskDelayUntil(&xLastWakeTime, 50); // Wait for the next cycle. Task cycle time 50 ms.
    }
}

//******************************************************************************
//  ENF OF FILE
//******************************************************************************
