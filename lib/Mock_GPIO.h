# pragma once

#define PWM_OUTPUT 1
#define OUTPUT 0
#define HIGH 1
#define LOW 0
#include <iostream>

int mockSetupGpio();

void mockSoftPwmCreate(int pinNumber, int startValue, int max);

void mockPinMode(int pinNumber, int mode);

void mockDigitalWrite(int pinNumber, int voltage);

void mockPwmWrite(int pinNumber, int pwm);

void mockSoftPwmWrite(int pinNumber, int pwm);