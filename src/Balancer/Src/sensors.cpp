#include "sensors.h"
#include "motor.h"

#include <cmath>

acc_sensor_t AccSensor;			// Structure for real time accel sensor data
gyro_sensor_t GyroSensor;		// Structure for real time gyro sensor data

static void UpdateAccel(void);
static void UpdateGyro(void);
static float clamp(float v, float minv, float maxv);


/*************************************************************
*  Function:       SensGetTempCPU
*------------------------------------------------------------
*  description:
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


void SensInit(void)
{
	if(!l3gd20Detect()) {
		// failureMode(3); // if this fails, we get a beep + blink pattern. we're doomed, no gyro or i2c error.
//		error = 1;
		Error_Handler();
	}

	lsm303dlhcConfig();
}


static float clamp(float v, float minv, float maxv)
{
   if(v > maxv)
      return maxv;
   else if(v < minv)
      return minv;
   return v;
}


static void UpdateGyro(void)
{
	int16_t gyro[3];
	uint8_t xSign;

	l3gd20Read(gyro);

	GyroSensor.DataDeg[X] = (float)gyro[X] * 0.07f;    // Gyro FS=2000 dps, Sensitivity = 70 mdps/lsb (L3GD20);
	GyroSensor.DataDeg[Y] = (float)gyro[Y] * 0.07f;
	GyroSensor.DataDeg[Z] = (float)gyro[Z] * 0.07f;

	if((gyro[0] & 0x8000) == 0) {
		xSign = 0;
	} else {
		xSign = 1;
		gyro[X] &= 0x7FFF;
		gyro[X] = 0x8000 - gyro[0];
	}

	if(abs(gyro[X]) < 0x0A) {
		gyro[X] = 0;
	}

	if(xSign == 0) {
		GyroSensor.Angle[X] = (float) gyro[X] * 0.07f;
	} else {
		GyroSensor.Angle[X] = (-1) * (float) gyro[X] * 0.07f;
	}

	//GyroSensor.DataDeg[0] = (float)gyro[0] * 0.07f;    // Gyro FS=2000 dps, Sensitivity = 70 mdps/lsb (L3GD20);

	//GyroSensor.Angle[0] += GyroSensor.DataDeg[0] * 0.02f; // ������ ���� ���������� �� ������ ���������

	static uint8_t temp_counter = 0;            // Temperature sensor refresh rate 1 Hz. Obtain through 50 cycles

	if (temp_counter++ == 50) {
		int16_t t;
		temp_counter = 0;

		if(l3gd20GetTemp(&t)) {
			GyroSensor.Temp = -7.87 * t + 332.3; // Convert to 0.1 degrees of Celcius
		}
	}
}

/**
 * @brief Updates acceleration sensor data and calculates angles.
 *
 * This function reads raw acceleration data from the LSM303DLHC accelerometer,
 * converts it to physical units (m/s²), and calculates the tilt angle along the Y-axis.
 *
 * @details
 * - The scaling coefficient depends on the full-scale range of the accelerometer.
 * - The default scaling coefficient in this implementation corresponds to ±8g full-scale range.
 * - Full-scale ranges and corresponding coefficients:
 *   - ±2g: 0.00980665 m/s²/LSB
 *   - ±4g: 0.0196133 m/s²/LSB
 *   - ±8g: 0.0392266 m/s²/LSB
 *   - ±16g: 0.1176798 m/s²/LSB
 *
 * To adjust for a different range, update the scaling coefficients:
 * - For example, if the full-scale range is set to ±4g, use the following:
 *   AccSensor.DataMSS[i] = (float)accel[i] * 0.0196133;
 *
 * Temperature is updated every 50 cycles (1 Hz refresh rate).
 */
static void UpdateAccel(void)
{
	int16_t accel[3];

	lsm303dlhcReadAcc(accel);

	AccSensor.DataMSS[0] = (float)accel[0] * 0.0392266;  // Scale to the m/s2.
	AccSensor.DataMSS[1] = (float)accel[1] * 0.0392266;  // acc_scale: 0,004 * 9,80665 = 0,0392266 (m/s2/lsb)
	AccSensor.DataMSS[2] = (float)accel[2] * 0.0392266;

	AccSensor.Angle[Y] = (-1.0f) * atan2((float)accel[Z], (float)accel[X]) * RAD_TO_DEG + 90.0f;

//	if (AccSensor.Angle[0] > 180.0)
//	    AccSensor.Angle[0] -= 360;

	static uint8_t temp_counter = 0;               // Temperature sensor refresh rate 1 Hz. Obtain through 50 cycles

	if (temp_counter++ == 50) {
		int16_t t;
		temp_counter = 0;

		if (GetTemperature(&t)){
			AccSensor.Temp = 1.149 * t + 170;	  // Convert to 0.1 degrees of Celcius
		}
	}
}


portTASK_FUNCTION_PROTO(SensorTask, pvParameters)
{
	float p_angle = 0.0;
	float i_angle = 0.0;                               // Integral value of angle-error (sum of gyro-angles) NOT EQUAL TO gyro_angle
	float d_angle = 0.0;                               // Derivative value for angle, angular velocity, how fast angle is changing
	float previous_p_angle = 0.0;

  const float k_p = 1.0;  // Proportional gain
  const float k_i = 0.1;  // Integrative gain
  const float k_d = 0.05; // Derivative gain

	const float delta_time = 0.01f; // Time step in seconds
	float overall_scalar = 170;                       // Overall scalar that speed is divided by
	float desired_angle = 0.0;                         // Setpoint. Set unequal zero to drive

	portTickType x_last_wake_time;
	const float f_k = 0.1;
	float drive_speed = 0.0;         // Variable for motor speed
	/* Sensors already initialized in main.cpp */

	x_last_wake_time = xTaskGetTickCount(); // Initialise the x_last_wake_time variable with the current time.

	Motor motor(GPIOB, GPIO_PIN_9, GPIOB, GPIO_PIN_7, 3, 1);

	while(true) {
		UpdateGyro();                        // Read gyro sensor data each cycle
		UpdateAccel();                        // Read accel sensor data each cycle

		p_angle = (p_angle + GyroSensor.Angle[X] * delta_time) * (1.0 - f_k)
				+ AccSensor.Angle[Y] * f_k;

		d_angle = (p_angle - previous_p_angle) / delta_time;
		previous_p_angle = p_angle;

		i_angle += (p_angle * delta_time); // integrate the angle (multiply by timestep to get dt!)

		float error = p_angle - desired_angle;

		if(fabs(error) >= 1.0 && abs(error) <= 15.0) { // If it is tilted enough, but not too much
			drive_speed = (k_i * i_angle + k_d * d_angle + k_p * p_angle) / overall_scalar; // drive to correct

      // Cap speed to [-1.0, 1.0]
      drive_speed = fmax(fmin(drive_speed, 1.0), -1.0);

		} else {
			drive_speed = 0;               // If we've fallen over or are steady on top
		}

		motor.Drive(drive_speed);

		vTaskDelayUntil(&x_last_wake_time, static_cast<uint32_t>(delta_time * 1000)); // Wait for the next cycle. Task cycle time 15 ms.
	}
}


//******************************************************************************
//  ENF OF FILE
//******************************************************************************
