//******************************************************************************
//  Секция include
//******************************************************************************

#include "motor.h"

//******************************************************************************
//  Секция определения переменных, используемых в модуле
//******************************************************************************

//------------------------------------------------------------------------------
// Глобальные
//------------------------------------------------------------------------------

int direction = 0;

//------------------------------------------------------------------------------
// Локальные
//------------------------------------------------------------------------------

//******************************************************************************
//  Секция прототипов локальных функций
//******************************************************************************


//******************************************************************************
//  Секция описания функций (сначала глобальных, потом локальных)
//******************************************************************************

/*************************************************************
*  Function:       MotorCtrlDrive
*------------------------------------------------------------
*  description:    Управление скоростью и направлением
*                  вращения моторами
*  parameters:     speed - значение скорости вращения в %,
*                  если значение положительное, то вращение
*                  в одну сторону, если отрицательное, то в другую
*  on return:      void
*************************************************************/
void MotorCtrlDrive(float speed)
{
   direction = 0;              // Variable to hold direction we want to drive

   if (speed > 0.0)
      direction = 1;               // Positive speed indicates forward
   if (speed < 0.0)
	  direction = 2;               // Negative speed indicates backwards
   if(speed == 0.0)
	  direction = 0;               // Zero speed indicates stopping

   switch(direction)               // Depending on what direction was passed
   {
      case 0:                      // Stop case
	     MOTOR_LEFT_SPEED(STOP);   // Set the DigitalOuts to stop the motors
	     MOTOR_RIGHT_SPEED(STOP);
	     MOTOR_RIGHT_DIR(STOP);
	     MOTOR_LEFT_DIR(STOP);
	     break;
	  case 1:                      // Forward case
	     MOTOR_RIGHT_DIR(FORWARD); // Set the DigitalOuts to run the motors forward
	     MOTOR_LEFT_DIR(FORWARD);
	     MOTOR_LEFT_SPEED(0);
	     MOTOR_RIGHT_SPEED(0);
	     break;
	  case 2:                      // Backwards
	     MOTOR_RIGHT_DIR(BACKWARD);// Set the DigitalOuts to run the motors backward
	     MOTOR_LEFT_DIR(BACKWARD);
	     MOTOR_LEFT_SPEED(1);
	     MOTOR_RIGHT_SPEED(1);
	     break;
	  default:                     // Catch-all (Stop)
		 MOTOR_LEFT_SPEED(STOP);   // Set the DigitalOuts to stop the motors
		 MOTOR_RIGHT_SPEED(STOP);
		 MOTOR_RIGHT_DIR(STOP);
		 MOTOR_LEFT_DIR(STOP);
	     break;
   }
}


//******************************************************************************
//  ENF OF FILE
//******************************************************************************
