#ifndef MOTOR_H_
#define MOTOR_H_

#include "conf.h"
#include "tim.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STOP = 0,
    BACKWARD = 1,
    FORWARD = 2
} Direction;

class Motor {
public:
    Motor(GPIO_TypeDef* leftDirPort, uint16_t leftDirPin,
          GPIO_TypeDef* rightDirPort, uint16_t rightDirPin,
          uint8_t leftPwmChannel, uint8_t rightPwmChannel);

    void Drive(float speed);
    void Stop();
    Direction GetDirection() const;
    float GetSpeed() const;

private:
    GPIO_TypeDef* leftDirPort;
    uint16_t leftDirPin;
    GPIO_TypeDef* rightDirPort;
    uint16_t rightDirPin;
    uint8_t leftPwmChannel;
    uint8_t right_pwm_channel;
    Direction current_direction;
    uint16_t current_speed;

    Direction DetermineDirection(float speed) const;
    void SetDirection(Direction direction);
    void SetSpeed(uint16_t speed);
};

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_H_
