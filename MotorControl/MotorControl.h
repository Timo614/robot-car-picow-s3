#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include "driver/mcpwm.h"

enum MotorStatus {
    STOP = 0,
    MOVE = 1
};

enum MotorDirection {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    LEFT_FORWARD,
    RIGHT_FORWARD,
    LEFT_BACKWARD,
    RIGHT_BACKWARD,
    TURN_LEFT,
    TURN_RIGHT
};

int mapSpeed(int x, int in_min, int in_max, int out_min, int out_max);

class MotorControl {
public:
    MotorControl();
    void motorLeftFront(MotorStatus status, MotorDirection direction, int speed);
    void motorRightFront(MotorStatus status, MotorDirection direction, int speed);
    void motorRightBack(MotorStatus status, MotorDirection direction, int speed);
    void motorLeftBack(MotorStatus status, MotorDirection direction, int speed);
    void stopAllMotors();
    void moveCar(MotorStatus status, MotorDirection direction, int speed);

private:
    int Motor_LF_PWM_Pin;
    int Motor_LF_Dir_Pin;
    int Motor_RF_PWM_Pin;
    int Motor_RF_Dir_Pin;
    int Motor_RB_PWM_Pin;
    int Motor_RB_Dir_Pin;
    int Motor_LB_PWM_Pin;
    int Motor_LB_Dir_Pin;

    void initMotor(int pwmPin, int dirPin, mcpwm_unit_t unit, mcpwm_io_signals_t io_signal, mcpwm_timer_t timer);
    void controlMotor(int pwmPin, int dirPin, MotorStatus status, MotorDirection direction, int speed, mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_generator_t op, bool invertDir = false);
};

#endif // MOTORCONTROL_H
