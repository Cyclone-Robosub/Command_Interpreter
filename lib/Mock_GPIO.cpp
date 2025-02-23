#include "Mock_GPIO.h"
#include <iostream>

int mockSetupGpio() {
    std::cout << "[Mock] wiringPi GPIO set up!" << std::endl;
    return 0;
}

void mockSoftPwmCreate(int pinNumber, int startValue, int max) {
    std::cout << "Create software pwm pin number " << pinNumber << " in range [0," << max << "] and starting at "
              << startValue << std::endl;
}

void mockPinMode(int pinNumber, int mode) {
    std::cout << "[Mock] pinMode: pin " << pinNumber << " set to mode " << mode << std::endl;
}

void mockDigitalWrite(int pinNumber, int voltage) {
    std::cout << "[Mock] digitalWrite: pin " << pinNumber << ", voltage " << voltage << std::endl;
}

void mockPwmWrite(int pinNumber, int pwm) {
    std::cout << "[Mock] pwmWrite: pin " << pinNumber << ", PWM " << pwm << std::endl;
}

void mockSoftPwmWrite(int pinNumber, int pwm) {
    std::cout << "[Mock] softPwmWrite: pin" << pinNumber << ", PWM " << pwm << std::endl;
}
