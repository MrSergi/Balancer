//#include "motor.h"
//
//int direction = 0;
//
//void MotorCtrlDrive(float speed)
//{
//	uint16_t MotorSpeed = 0;
//
//	direction = 0;                  // Variable to hold direction we want to drive
//
//	if(speed > 0.0) {
//		direction = 1;               // Positive speed indicates forward
//	}
//	if(speed < 0.0) {
//		direction = 2;               // Negative speed indicates backwards
//	}
//	if(speed == 0.0) {
//		direction = 0;                     // Zero speed indicates stopping
//	}
//
//   MotorSpeed = abs(speed * 100);
//
//   switch(direction) { 								// Depending on what direction was passed
//      case 0:                         // Stop case
//	     MOTOR_LEFT_SPEED(MotorSpeed);  // Set the DigitalOuts to stop the motors
//	     MOTOR_RIGHT_SPEED(MotorSpeed);
//	     MOTOR_RIGHT_DIR(STOP);
//	     MOTOR_LEFT_DIR(STOP);
//	     break;
//	  case 1:                            // Forward case
//	     MOTOR_RIGHT_DIR(FORWARD);       // Set the DigitalOuts to run the motors forward
//	     MOTOR_LEFT_DIR(FORWARD);
//	     MOTOR_LEFT_SPEED(MotorSpeed);
//	     MOTOR_RIGHT_SPEED(MotorSpeed);
//	     break;
//	  case 2:                            // Backwards
//	     MOTOR_RIGHT_DIR(BACKWARD);      // Set the DigitalOuts to run the motors backward
//	     MOTOR_LEFT_DIR(BACKWARD);
//			MOTOR_LEFT_SPEED(MotorSpeed);
//			MOTOR_RIGHT_SPEED(MotorSpeed);
//	     break;
//	  default:                           // Catch-all (Stop)
//		 MOTOR_LEFT_SPEED(MotorSpeed);   // Set the DigitalOuts to stop the motors
//		 MOTOR_RIGHT_SPEED(MotorSpeed);
//		 MOTOR_RIGHT_DIR(STOP);
//		 MOTOR_LEFT_DIR(STOP);
//		 break;
//   }
//}
//
//
////******************************************************************************
////  ENF OF FILE
////******************************************************************************
