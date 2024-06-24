#include "MotorControl.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#define MCPWM_UNIT_LF MCPWM_UNIT_0
#define MCPWM_UNIT_RF MCPWM_UNIT_0
#define MCPWM_UNIT_RB MCPWM_UNIT_1
#define MCPWM_UNIT_LB MCPWM_UNIT_1

#define MCPWM_TIMER_LF MCPWM_TIMER_0
#define MCPWM_TIMER_RF MCPWM_TIMER_1
#define MCPWM_TIMER_RB MCPWM_TIMER_0
#define MCPWM_TIMER_LB MCPWM_TIMER_1

#define MCPWM_IO_LF MCPWM0A
#define MCPWM_IO_RF MCPWM1A
#define MCPWM_IO_RB MCPWM0A
#define MCPWM_IO_LB MCPWM1A

#define MCPWM_OPR_LF MCPWM_GEN_A
#define MCPWM_OPR_RF MCPWM_GEN_A
#define MCPWM_OPR_RB MCPWM_GEN_A
#define MCPWM_OPR_LB MCPWM_GEN_A

int mapSpeed(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

MotorControl::MotorControl() {
    Motor_LF_PWM_Pin = 39;
    Motor_LF_Dir_Pin = 40;
    Motor_RF_PWM_Pin = 42;
    Motor_RF_Dir_Pin = 41;
    Motor_RB_PWM_Pin = 1;
    Motor_RB_Dir_Pin = 2;
    Motor_LB_PWM_Pin = 4;
    Motor_LB_Dir_Pin = 3;

    initMotor(Motor_LF_PWM_Pin, Motor_LF_Dir_Pin, MCPWM_UNIT_LF, MCPWM_IO_LF, MCPWM_TIMER_LF);
    initMotor(Motor_RF_PWM_Pin, Motor_RF_Dir_Pin, MCPWM_UNIT_RF, MCPWM_IO_RF, MCPWM_TIMER_RF);
    initMotor(Motor_RB_PWM_Pin, Motor_RB_Dir_Pin, MCPWM_UNIT_RB, MCPWM_IO_RB, MCPWM_TIMER_RB);
    initMotor(Motor_LB_PWM_Pin, Motor_LB_Dir_Pin, MCPWM_UNIT_LB, MCPWM_IO_LB, MCPWM_TIMER_LB);
}

void MotorControl::initMotor(int pwmPin, int dirPin, mcpwm_unit_t unit, mcpwm_io_signals_t io_signal, mcpwm_timer_t timer) {
    pinMode(dirPin, OUTPUT);
    mcpwm_gpio_init(unit, io_signal, pwmPin);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;  // Frequency in Hz
    pwm_config.cmpr_a = 0;        // Duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;        // Duty cycle of PWMxB = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(unit, timer, &pwm_config);
}

void MotorControl::controlMotor(int pwmPin, int dirPin, MotorStatus status, MotorDirection direction, int speed, mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_generator_t op, bool invertDir) {
    if (status == STOP) {
        digitalWrite(dirPin, LOW);
        mcpwm_set_duty(unit, timer, op, 0);
    } else {
        int value = mapSpeed(speed, 0, 100, 0, 100); // MCPWM uses percentage for duty cycle
        if ((direction == FORWARD && !invertDir) || (direction == BACKWARD && invertDir)) {
            digitalWrite(dirPin, LOW);
            mcpwm_set_duty(unit, timer, op, value);
            mcpwm_set_duty_type(unit, timer, op, MCPWM_DUTY_MODE_0);
        } else if ((direction == BACKWARD && !invertDir) || (direction == FORWARD && invertDir)) {
            digitalWrite(dirPin, HIGH);
            mcpwm_set_duty(unit, timer, op, value);
            mcpwm_set_duty_type(unit, timer, op, MCPWM_DUTY_MODE_0);
        }
    }
}

void MotorControl::motorLeftFront(MotorStatus status, MotorDirection direction, int speed) {
    controlMotor(Motor_LF_PWM_Pin, Motor_LF_Dir_Pin, status, direction, speed, MCPWM_UNIT_LF, MCPWM_TIMER_LF, MCPWM_OPR_LF);
}

void MotorControl::motorRightFront(MotorStatus status, MotorDirection direction, int speed) {
    controlMotor(Motor_RF_PWM_Pin, Motor_RF_Dir_Pin, status, direction, speed, MCPWM_UNIT_RF, MCPWM_TIMER_RF, MCPWM_OPR_RF);
}

void MotorControl::motorRightBack(MotorStatus status, MotorDirection direction, int speed) {
    controlMotor(Motor_RB_PWM_Pin, Motor_RB_Dir_Pin, status, direction, speed, MCPWM_UNIT_RB, MCPWM_TIMER_RB, MCPWM_OPR_RB, true); // invertDir = true
}

void MotorControl::motorLeftBack(MotorStatus status, MotorDirection direction, int speed) {
    controlMotor(Motor_LB_PWM_Pin, Motor_LB_Dir_Pin, status, direction, speed, MCPWM_UNIT_LB, MCPWM_TIMER_LB, MCPWM_OPR_LB, true); // invertDir = true
}

void MotorControl::stopAllMotors() {
    motorLeftFront(STOP, FORWARD, 0);
    motorRightFront(STOP, FORWARD, 0);
    motorRightBack(STOP, FORWARD, 0);
    motorLeftBack(STOP, FORWARD, 0);
}

void MotorControl::moveCar(MotorStatus status, MotorDirection direction, int speed) {
    if (status == STOP) {
        stopAllMotors();
    } else {
        switch (direction) {
            case FORWARD:
                motorLeftFront(MOVE, FORWARD, speed);
                motorRightFront(MOVE, FORWARD, speed);
                motorRightBack(MOVE, FORWARD, speed);
                motorLeftBack(MOVE, FORWARD, speed);
                break;
            case BACKWARD:
                motorLeftFront(MOVE, BACKWARD, speed);
                motorRightFront(MOVE, BACKWARD, speed);
                motorRightBack(MOVE, BACKWARD, speed);
                motorLeftBack(MOVE, BACKWARD, speed);
                break;
            case LEFT:
                motorLeftFront(MOVE, BACKWARD, speed);
                motorRightFront(MOVE, FORWARD, speed);
                motorRightBack(MOVE, BACKWARD, speed);
                motorLeftBack(MOVE, FORWARD, speed);
                break;
            case RIGHT:
                motorLeftFront(MOVE, FORWARD, speed);
                motorRightFront(MOVE, BACKWARD, speed);
                motorRightBack(MOVE, FORWARD, speed);
                motorLeftBack(MOVE, BACKWARD, speed);
                break;
            case LEFT_FORWARD:
                motorLeftFront(STOP, FORWARD, 0);
                motorRightFront(MOVE, FORWARD, speed);
                motorRightBack(STOP, FORWARD, 0);
                motorLeftBack(MOVE, FORWARD, speed);
                break;
            case RIGHT_FORWARD:
                motorLeftFront(MOVE, FORWARD, speed);
                motorRightFront(STOP, FORWARD, 0);
                motorRightBack(MOVE, FORWARD, speed);
                motorLeftBack(STOP, FORWARD, 0);
                break;
            case LEFT_BACKWARD:
                motorLeftFront(MOVE, BACKWARD, speed);
                motorRightFront(STOP, FORWARD, 0);
                motorRightBack(MOVE, BACKWARD, speed);
                motorLeftBack(STOP, FORWARD, 0);
                break;
            case RIGHT_BACKWARD:
                motorLeftFront(STOP, FORWARD, 0);
                motorRightFront(MOVE, BACKWARD, speed);
                motorRightBack(STOP, FORWARD, 0);
                motorLeftBack(MOVE, BACKWARD, speed);
                break;
            case TURN_LEFT:
                motorLeftFront(MOVE, BACKWARD, speed);
                motorRightFront(MOVE, FORWARD, speed);
                motorRightBack(MOVE, FORWARD, speed);
                motorLeftBack(MOVE, BACKWARD, speed);
                break;
            case TURN_RIGHT:
                motorLeftFront(MOVE, FORWARD, speed);
                motorRightFront(MOVE, BACKWARD, speed);
                motorRightBack(MOVE, BACKWARD, speed);
                motorLeftBack(MOVE, FORWARD, speed);
                break;
            default:
                Serial.println("Direction error!");
                break;
        }
    }
}

