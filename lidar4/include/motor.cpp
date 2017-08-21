#include <wiringPi.h>
#include "motor.h"

//#define PWM_CLOCK_DIVISOR 2 // 25 kHz
#define PWM_CLOCK_DIVISOR 8
#define PWM_RANGE 384

#define PIN_MOTOR_LEFT_L_EN  23
#define PIN_MOTOR_LEFT_R_EN  24
#define PIN_MOTOR_LEFT_LPWM  18
#define PIN_MOTOR_LEFT_RPWM  12
#define PIN_MOTOR_RIGHT_L_EN  5
#define PIN_MOTOR_RIGHT_R_EN  6
#define PIN_MOTOR_RIGHT_LPWM 19
#define PIN_MOTOR_RIGHT_RPWM 13

void motor_init (void)
{
    // Left motor
    pinMode(PIN_MOTOR_LEFT_LPWM, OUTPUT);
    digitalWrite(PIN_MOTOR_LEFT_LPWM, LOW);
    pinMode(PIN_MOTOR_LEFT_RPWM, OUTPUT);
    digitalWrite(PIN_MOTOR_LEFT_RPWM, LOW);
    pinMode(PIN_MOTOR_LEFT_L_EN, OUTPUT);
    digitalWrite(PIN_MOTOR_LEFT_L_EN, HIGH);
    pinMode(PIN_MOTOR_LEFT_R_EN, OUTPUT);
    digitalWrite(PIN_MOTOR_LEFT_R_EN, HIGH);
    
    // Right motor
    pinMode(PIN_MOTOR_RIGHT_LPWM, OUTPUT);
    digitalWrite(PIN_MOTOR_RIGHT_LPWM, LOW);
    pinMode(PIN_MOTOR_RIGHT_RPWM, OUTPUT);
    digitalWrite(PIN_MOTOR_RIGHT_RPWM, LOW);
    pinMode(PIN_MOTOR_RIGHT_L_EN, OUTPUT);
    digitalWrite(PIN_MOTOR_RIGHT_L_EN, HIGH);
    pinMode(PIN_MOTOR_RIGHT_R_EN, OUTPUT);
    digitalWrite(PIN_MOTOR_RIGHT_R_EN, HIGH);
}

void motor_term (void)
{
    // Left motor
    pinMode(PIN_MOTOR_LEFT_L_EN, OUTPUT);
    digitalWrite(PIN_MOTOR_LEFT_L_EN, LOW);
    pinMode(PIN_MOTOR_LEFT_R_EN, OUTPUT);
    digitalWrite(PIN_MOTOR_LEFT_R_EN, LOW);
    pinMode(PIN_MOTOR_LEFT_LPWM, OUTPUT);
    digitalWrite(PIN_MOTOR_LEFT_LPWM, LOW);
    pinMode(PIN_MOTOR_LEFT_RPWM, OUTPUT);
    digitalWrite(PIN_MOTOR_LEFT_RPWM, LOW);
    
    // Right motor
    pinMode(PIN_MOTOR_RIGHT_L_EN, OUTPUT);
    digitalWrite(PIN_MOTOR_RIGHT_L_EN, LOW);
    pinMode(PIN_MOTOR_RIGHT_R_EN, OUTPUT);
    digitalWrite(PIN_MOTOR_RIGHT_R_EN, LOW);
    pinMode(PIN_MOTOR_RIGHT_LPWM, OUTPUT);
    digitalWrite(PIN_MOTOR_RIGHT_LPWM, LOW);
    pinMode(PIN_MOTOR_RIGHT_RPWM, OUTPUT);
    digitalWrite(PIN_MOTOR_RIGHT_RPWM, LOW);
}

void motor_left_set_normalized_speed (double normalized_speed)
{
    int pulse_width;
    
    if (normalized_speed > 0) {
        pulse_width = (int) (PWM_RANGE * normalized_speed);
        
        pinMode(PIN_MOTOR_LEFT_RPWM, OUTPUT);
        digitalWrite(PIN_MOTOR_LEFT_RPWM, LOW);
        
        pinMode(PIN_MOTOR_LEFT_LPWM, PWM_OUTPUT);
        pwmSetMode(PWM_MODE_MS);
        pwmSetRange(PWM_RANGE);
        pwmWrite(PIN_MOTOR_LEFT_LPWM, pulse_width);
        pwmSetClock(PWM_CLOCK_DIVISOR);
    } else if (normalized_speed < 0) {
        pulse_width = (int) (PWM_RANGE * -normalized_speed);
        
        pinMode(PIN_MOTOR_LEFT_LPWM, OUTPUT);
        digitalWrite(PIN_MOTOR_LEFT_LPWM, LOW);
        
        pinMode(PIN_MOTOR_LEFT_RPWM, PWM_OUTPUT);
        pwmSetMode(PWM_MODE_MS);
        pwmSetRange(PWM_RANGE);
        pwmWrite(PIN_MOTOR_LEFT_RPWM, pulse_width);
        pwmSetClock(PWM_CLOCK_DIVISOR);
    } else {
        pinMode(PIN_MOTOR_LEFT_LPWM, OUTPUT);
        digitalWrite(PIN_MOTOR_LEFT_LPWM, LOW);
        pinMode(PIN_MOTOR_LEFT_RPWM, OUTPUT);
        digitalWrite(PIN_MOTOR_LEFT_RPWM, LOW);
    }
}

void motor_right_set_normalized_speed (double normalized_speed)
{
    int pulse_width;
    
    if (normalized_speed > 0) {
        pulse_width = (int) (PWM_RANGE * normalized_speed);
        
        pinMode(PIN_MOTOR_RIGHT_RPWM, OUTPUT);
        digitalWrite(PIN_MOTOR_RIGHT_RPWM, LOW);
        
        pinMode(PIN_MOTOR_RIGHT_LPWM, PWM_OUTPUT);
        pwmSetMode(PWM_MODE_MS);
        pwmSetRange(PWM_RANGE);
        pwmWrite(PIN_MOTOR_RIGHT_LPWM, pulse_width);
        pwmSetClock(PWM_CLOCK_DIVISOR);
    } else if (normalized_speed < 0) {
        pulse_width = (int) (PWM_RANGE * -normalized_speed);
        
        pinMode(PIN_MOTOR_RIGHT_LPWM, OUTPUT);
        digitalWrite(PIN_MOTOR_RIGHT_LPWM, LOW);
        
        pinMode(PIN_MOTOR_RIGHT_RPWM, PWM_OUTPUT);
        pwmSetMode(PWM_MODE_MS);
        pwmSetRange(PWM_RANGE);
        pwmWrite(PIN_MOTOR_RIGHT_RPWM, pulse_width);
        pwmSetClock(PWM_CLOCK_DIVISOR);
    } else {
        pinMode(PIN_MOTOR_RIGHT_LPWM, OUTPUT);
        digitalWrite(PIN_MOTOR_RIGHT_LPWM, LOW);
        pinMode(PIN_MOTOR_RIGHT_RPWM, OUTPUT);
        digitalWrite(PIN_MOTOR_RIGHT_RPWM, LOW);
    }
}
