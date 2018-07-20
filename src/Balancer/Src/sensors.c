//******************************************************************************
//  ������ include
//******************************************************************************

#include "sensors.h"

//******************************************************************************
//  ������ ����������� ����������, ������������ � ������
//******************************************************************************

//------------------------------------------------------------------------------
// ����������
//------------------------------------------------------------------------------

acc_sensor_t AccSensor;			// Structure for real time accel sensor data
gyro_sensor_t GyroSensor;		// Structure for real time gyro sensor data
float pAngle = 0.0;				// ��������, ���������������� ���� ����������
uint8_t period = 0;
float GyroAngleX = 0.0;			// ��������, ���������������� ���� ���������� �� ���������

//------------------------------------------------------------------------------
// ���������
//------------------------------------------------------------------------------

//******************************************************************************
//  ������ ���������� ��������� �������
//******************************************************************************

static void SensUpdateAcc(void);
static void SensUpdateGyro(void);
static float clamp(float v, float minv, float maxv);

//******************************************************************************
//  ������ �������� ������� (������� ����������, ����� ���������)
//******************************************************************************

/*************************************************************
*  Function:       SensGetTempCPU
*------------------------------------------------------------
*  description:    ���������� ����������� ����������������
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
*  description:    ������������� � ������������ ��������
*  parameters:     void
*  on return:      void
*************************************************************/
void SensInit(void)
{
	uint8_t error = 0;

	if (!l3gd20Detect())   // ����������� ��������
	{
		// failureMode(3); // if this fails, we get a beep + blink pattern. we're doomed, no gyro or i2c error.
		error = 1;
	}

	lsm303dlhcConfig();    // ����������� ������������ � �����������
}

/*************************************************************
*  Function:       clamp
*------------------------------------------------------------
*  description:    �� ��� �������� ����� �� ������������� �������
*  parameters:     v - ����������� ��������
*                  minv - ���������� ��������� ��������
*                  maxv - ����������� ��������� ��������
*  on return:      float - �������� ����� ��������
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
*  description:    ��������� �������� ������� ��������
*                  �� ��������� �� 3-� ����
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

	//GyroSensor.Angle[0] += GyroSensor.DataDeg[0] * 0.02f; // ������ ���� ���������� �� ������ ���������

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
*  description:    ��������� �������� ��������� �� �������������
*                  �� 3-� ����
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

	AccSensor.Angle[Y] = (-1.0f) * atan2((float)accel[Z], (float)accel[X]) * RAD_TO_DEG + 90.0f; // ��������� ���� �� �������������

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
//  ������ �������� ����� ��
//******************************************************************************

portTASK_FUNCTION_PROTO(SensorTask, pvParameters)
{
	portTickType xLastWakeTime;
	const float FK = 0.1;
	                                         /* Sensors already initialized in main.c */

	xLastWakeTime = xTaskGetTickCount();     // Initialise the xLastWakeTime variable with the current time.

    while (1)
    {
    	SensUpdateGyro();                    // Read gyro sensor data each cycle

    	SensUpdateAcc();                     // Read accel sensor data each cycle

    	GyroAngleX += GyroSensor.Angle[X] * 0.015f;

    	pAngle = (pAngle + GyroSensor.Angle[X] * 0.015f) * (1.0 - FK) + AccSensor.Angle[Y] * FK;

    	vTaskDelayUntil(&xLastWakeTime, 15); // Wait for the next cycle. Task cycle time 20 ms.
    }
}

//******************************************************************************
//  ENF OF FILE
//******************************************************************************
