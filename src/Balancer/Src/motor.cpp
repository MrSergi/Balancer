#include "motor.h"

Motor::Motor(GPIO_TypeDef* leftDirPort, uint16_t leftDirPin,
             GPIO_TypeDef* rightDirPort, uint16_t rightDirPin,
             uint8_t leftPwmChannel, uint8_t rightPwmChannel)
    : leftDirPort(leftDirPort),
      leftDirPin(leftDirPin),
      rightDirPort(rightDirPort),
      rightDirPin(rightDirPin),
      leftPwmChannel(leftPwmChannel),
      right_pwm_channel(rightPwmChannel),
      current_direction(Direction::STOP),
      current_speed(0) {}

void Motor::Drive(float speed)
{
	current_direction = DetermineDirection(speed);
	current_speed = static_cast<uint16_t>(std::abs(speed * 100));

	SetDirection(current_direction);
	SetSpeed(current_speed);
}

void Motor::Stop()
{
	SetDirection(Direction::STOP);
	SetSpeed(0);
}

Direction Motor::GetDirection() const
{
	return current_direction;
}

float Motor::GetSpeed() const
{
	return static_cast<float>(current_speed) / 100;
}

Direction Motor::DetermineDirection(float speed) const
{
	if (speed > 0.0f) return Direction::FORWARD;
	if (speed < 0.0f) return Direction::BACKWARD;
	return Direction::STOP;
}

void Motor::SetDirection(Direction direction)
{
	switch(direction) {
		case Direction::FORWARD:
			HAL_GPIO_WritePin(leftDirPort, leftDirPin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(rightDirPort, rightDirPin, GPIO_PIN_SET);
			break;
		case Direction::BACKWARD:
			HAL_GPIO_WritePin(leftDirPort, leftDirPin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(rightDirPort, rightDirPin, GPIO_PIN_RESET);
			break;
		case Direction::STOP:
		default:
			HAL_GPIO_WritePin(leftDirPort, leftDirPin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(rightDirPort, rightDirPin, GPIO_PIN_RESET);
			break;
	}
}

void Motor::SetSpeed(uint16_t speed)
{
	TimPWMSetPulseValue(leftPwmChannel, speed);
	TimPWMSetPulseValue(right_pwm_channel, speed);
}
