/*
 * sensors.h
 *
 *  Created on: 4 июл. 2018 г.
 *      Author: Sergey Pervushkin
 */

#ifndef SENSORS_H_   // Блокируем повторное включение этого модуля
#define SENSORS_H_

//******************************************************************************
//  Секция include
//******************************************************************************

#include "conf.h"
#include "lsm303dlhc.h"
#include "l3gd20.h"
#include "usbd_cdc_if.h"

//******************************************************************************
//  Секция определения констант
//******************************************************************************

#define RAD_TO_DEG   57.29577951308232087679815481410517033f
#define S_PI         3.14159265358979323846f


enum         // Оси датчиков
{
    X = 0,
    Y,
    Z
};

//******************************************************************************
//  Секция определения типов
//******************************************************************************

typedef struct acc_sensor_t
{
	uint32_t	SampleCount;			// Accel sensor loop counter. Should be increased with 760 Hz freq (Gyro ODR).
	uint32_t	OverrunCount;			// Accel sensor data not abalible counter. Should be 0.
	int16_t		Temp;					// Accel sensor temperature in 0.1 degrees of celsius
	float		DataMSS[3];				// Accel sensor data scaled to the m/s2
	float		DataMW[3];				// Accel sensor data scaled to MultiWii
	float		Average[3];				// Average value (degrees / second)
	float		Variance[3];			// Variance (degrees / second)
	float		Angle[3];				// Угол отклонения (градус)
} acc_sensor_t;

typedef struct gyro_sensor_t {
	uint32_t	SampleCount;			// Gyro sensor loop counter. Should be increased with 760 Hz freq (Gyro ODR).
	uint32_t	OverrunCount;			// Gyro sensor data not abalible counter. Should be 0.
	int16_t		Temp;					// Gyro sensor temperature in 0.1 degrees of celsius
	float		DataDeg[3];			    // Gyro sensor data. Scaled to degrees / second
	float		DataRad[3];			    // Gyro sensor data. Scaled to radian / second
	float		Average[3];				// Average value (degrees / second)
	float		Variance[3];			// Variance (degrees / second)
	float		Angle[3];				// Угол отклонения (градус)
} gyro_sensor_t;

//******************************************************************************
//  Секция определения глобальных переменных
//******************************************************************************

extern acc_sensor_t AccSensor;
extern gyro_sensor_t GyroSensor;
extern float pAngle;
extern float GyroAngleX;

//******************************************************************************
//  Секция прототипов глобальных функций
//******************************************************************************

void SensInit(void);
int16_t SensGetTempCPU(void);
portTASK_FUNCTION_PROTO(SensorTask, pvParameters);

//******************************************************************************
//  Секция определения макросов
//******************************************************************************

#endif                   // Закрывающий #endif к блокировке повторного включения

//******************************************************************************
//  ENF OF FILE
//******************************************************************************

