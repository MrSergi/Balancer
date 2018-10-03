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
float GyroAngleX = 0.0;			// Значение, пропорциональное углу отклонения от гироскопа
float DriveSpeed = 0.0;         // Variable for motor speed

//------------------------------------------------------------------------------
// Локальные
//------------------------------------------------------------------------------

float kp = 1;   //40                              // 145 Proportional gain kU 400-500
float kd = 0;   //2 was quite good                // Derivative gain
float ki = 0;   //30                              // Integrative gain
float OVERALL_SCALAR = 170;                       // Overall scalar that speed is divided by
//float accBias = 0;                              // Accelerometer Bias
//float gyroBias = 0;                             // Gyro Bias
//float accAngle = 0;                             // Global to hold angle measured by Accelerometer
//float gyroAngle = 0;                            // This variable holds the amount the angle has changed
float iAngle = 0.0;                               // Integral value of angle-error (sum of gyro-angles)NOT EQUAL TO gyroAngle
float dAngle = 0.0;                               // Derivative value for angle, angular velocity, how fast angle is changing
//float pAngle = 0;                               // Proportional value for angle, current angle (best measurement)
float desiredAngle = 0.0;                         // Setpoint. Set unequal zero to drive

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
*  description:    Обновляет значения угловой скорости
*                  от гироскопа по 3-м осям
*  parameters:     void
*  on return:      void
*************************************************************/
static void SensUpdateGyro(void)
{
	int16_t gyro[3];
	uint8_t xSign;

	l3gd20Read(gyro);

	GyroSensor.DataDeg[X] = (float)gyro[X] * 0.07f;    // Gyro FS=2000 dps, Sensitivity = 70 mdps/lsb (L3GD20);
	GyroSensor.DataDeg[Y] = (float)gyro[Y] * 0.07f;
	GyroSensor.DataDeg[Z] = (float)gyro[Z] * 0.07f;

	if((gyro[0] & 0x8000) == 0)
	{
		xSign = 0;
	}
	else
	{
		xSign = 1;
		gyro[X] &= 0x7FFF;
		gyro[X] = 0x8000 - gyro[0];
	}

	if(abs(gyro[X]) < 0x0A)
	{
		gyro[X] = 0;
	}

	if(xSign == 0)
	{
		GyroSensor.Angle[X] = (float)gyro[X] * 0.07f;
	}
	else
	{
		GyroSensor.Angle[X] = (-1) * (float)gyro[X] * 0.07f;
	}

	//GyroSensor.DataDeg[0] = (float)gyro[0] * 0.07f;    // Gyro FS=2000 dps, Sensitivity = 70 mdps/lsb (L3GD20);

	//GyroSensor.Angle[0] += GyroSensor.DataDeg[0] * 0.02f; // расчёт угла отклонения по данным гироскопа

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

	AccSensor.DataMSS[0] = (float)accel[0] * 0.0392266;  // Scale to the m/s2.
	AccSensor.DataMSS[1] = (float)accel[1] * 0.0392266;  // acc_scale: 0,004 * 9,80665 = 0,0392266 (m/s2/lsb)
	AccSensor.DataMSS[2] = (float)accel[2] * 0.0392266;

	AccSensor.Angle[Y] = (-1.0f) * atan2((float)accel[Z], (float)accel[X]) * RAD_TO_DEG + 90.0f; // вычисляем угол от акселерометра

//	if (AccSensor.Angle[0] > 180.0)
//	    AccSensor.Angle[0] -= 360;

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
	const float FK = 0.1;
	                                         /* Sensors already initialized in main.c */

	xLastWakeTime = xTaskGetTickCount();                                      // Initialise the xLastWakeTime variable with the current time.

    while (1)
    {
    	SensUpdateGyro();                                                     // Read gyro sensor data each cycle

    	SensUpdateAcc();                                                      // Read accel sensor data each cycle

    	GyroAngleX += GyroSensor.Angle[X] * 0.015f;

    	pAngle = (pAngle + GyroSensor.Angle[X] * 0.015f) * (1.0 - FK) + AccSensor.Angle[Y] * FK;

    	dAngle = pAngle - dAngle;                                             // Ang. Veloc. less noisy than dAngle = -(imu.gx-gyroBias);

    	iAngle += (pAngle * 0.015f);                                          // integrate the angle (multiply by timestep to get dt!)

        if(abs(pAngle-desiredAngle) >= 1.0 && abs(pAngle-desiredAngle) <= 15.0) // If it is tilted enough, but not too much
        {
           DriveSpeed = (ki*iAngle + kd*dAngle + kp*pAngle) / OVERALL_SCALAR;     // drive to correct
           if(DriveSpeed < -1.0)
        	   DriveSpeed = -1.0;                                                    // Cap if undershoot
           else
        	   if(DriveSpeed > 1.0)
        		   DriveSpeed = 1.0;                                                 // Cap if overshoot
        }
        else
        {
        	   DriveSpeed = 0;                                                     // If we've fallen over or are steady on top
        }

        MotorCtrlDrive(DriveSpeed);                                                // Write speed to the motors

    	vTaskDelayUntil(&xLastWakeTime, 15);                                  // Wait for the next cycle. Task cycle time 15 ms.
    }
}

//******************************************************************************
//  ENF OF FILE
//******************************************************************************
