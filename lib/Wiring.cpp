// William Barber

#include "Wiring.h"

// When compiling for non-RPI devices which cannot run wiringPi library,
// use -MOCK_RPI flag to enable mock functions
#ifdef MOCK_RPI

#include "Mock_GPIO.h"

const int MAX_HARDWARE_PWM_VALUE = 1023;
const int MAX_SOFTWARE_PWM_VALUE = 100;

int scalePwm(int pwm, int maxPwmValue) {
    double pwmFrequency = pwm;
    // 1100 - 1464 = negative
    // 1536 - 1900 = positive
    const int minPosPwmFrequency = 1535; // slightly less than 1536 so that 1536 is not zero
    const int maxPosPwmFrequency = 1900;
    const int minNegPwmFrequency = 1100;
    const int maxNegPwmFrequency = 1465; // slightly greater than 1464 so that 1464 is not zero
    int pwmMagnitude;
    double multiplier;

    if (pwmFrequency >= 1465 && pwmFrequency <= 1535) {
        return 0;
    }
    if (pwmFrequency <= 1464) {
        multiplier = ((double)(maxPwmValue) / (double)(maxNegPwmFrequency - minNegPwmFrequency));
        pwmMagnitude = maxPwmValue - (int)((double)(pwmFrequency - minNegPwmFrequency) * multiplier);
    }
    else {
        multiplier = ((double)(maxPwmValue) / (double)(maxPosPwmFrequency - minPosPwmFrequency));
        pwmMagnitude = (int)((double)(pwmFrequency - minPosPwmFrequency) * multiplier);
    }
    return pwmMagnitude;
}


bool WiringControl::initializeGPIO() {
    if (mockSetupGpio() == 0) {
        return true;
    }
    else {
        return false;
    }
}

void WiringControl::setPinType(int pinNumber, PinType pinType) {
    switch (pinType) {
        case Digital:
            mockPinMode(pinNumber, OUTPUT);
            digitalPinStatuses[pinNumber] = Low;
            break;
        case HardwarePWM:
            pwmPinStatuses[pinNumber] = PwmPinStatus {scalePwm(1500, MAX_HARDWARE_PWM_VALUE), 0};
            mockPinMode(pinNumber, PWM_OUTPUT);
            mockSoftPwmCreate(pinNumber, 0, 100);
            break;
        case SoftwarePWM:
            pwmPinStatuses[pinNumber] = PwmPinStatus {scalePwm(1500, MAX_SOFTWARE_PWM_VALUE), 0};
            mockPinMode(pinNumber, OUTPUT);
            break;
        default:
            std::cerr << "Impossible pin type " << pinType << "! Exiting." << std::endl;
            exit(42);
    }
    pinTypes[pinNumber] = pinType;
}

void WiringControl::digitalWrite(int pinNumber, DigitalPinStatus digitalPinStatus) {
    switch (digitalPinStatus) {
        case Low:
            mockDigitalWrite(pinNumber, LOW);
            break;
        case High:
            mockDigitalWrite(pinNumber, HIGH);
            break;
        default:
            std::cerr << "Impossible digital pin status " << digitalPinStatus << "! Exiting." << std::endl;
            exit(42);
    }
    digitalPinStatuses[pinNumber] = digitalPinStatus;
}

DigitalPinStatus WiringControl::digitalRead(int pinNumber) {
    return digitalPinStatuses[pinNumber];
}

void WiringControl::pwmWrite(int pinNumber, int pwmFrequency) {
    switch (pinTypes[pinNumber]) {
        case HardwarePWM:
            mockPwmWrite(pinNumber, scalePwm(pwmFrequency, MAX_HARDWARE_PWM_VALUE));
            pwmPinStatuses[pinNumber].frequency = scalePwm(pwmFrequency, MAX_HARDWARE_PWM_VALUE);
            break;
        case SoftwarePWM:
            mockSoftPwmWrite(pinNumber, scalePwm(pwmFrequency, MAX_SOFTWARE_PWM_VALUE));
            pwmPinStatuses[pinNumber].frequency = scalePwm(pwmFrequency, MAX_SOFTWARE_PWM_VALUE);
            break;
        case Digital:
            std::cerr << "Invalid pin type \"Digital\". Digital pin type cannot be used for PWM. Exiting." << std::endl;
            exit(42);
            break;
        default:
            std::cerr << "Impossible pin type " << pinTypes[pinNumber] << "! Exiting." << std::endl;
            exit(42);
    }
}

void WiringControl::pwmWriteMaximum(int pinNumber) {
    pwmWrite(pinNumber, 1900);
}

void WiringControl::pwmWriteOff(int pinNumber) {
    pwmWrite(pinNumber, 1500);
}

PwmPinStatus WiringControl::pwmRead(int pinNumber) {
    return pwmPinStatuses[pinNumber];
}

#else

#ifdef WIRINGPI // If we're compiling to use the WiringPi library

namespace wiringPi {
#include <wiringPi.h>
	#include <softPwm.h>
}
#include <iostream>

const int MAX_HARDWARE_PWM_VALUE = 1023;
const int MAX_SOFTWARE_PWM_VALUE = 100;

int scalePwm(int pwm, int maxPwmValue) {
    double pwmFrequency = pwm;
    // 1100 - 1464 = negative
    // 1536 - 1900 = positive
    const int minPosPwmFrequency = 1535; // slightly less than 1536 so that 1536 is not zero
    const int maxPosPwmFrequency = 1900;
    const int minNegPwmFrequency = 1100;
    const int maxNegPwmFrequency = 1465; // slightly greater than 1464 so that 1464 is not zero
    int pwmMagnitude;
    double multiplier;

    if (pwmFrequency >= 1465 && pwmFrequency <= 1535) {
        return 0;
    }
    if (pwmFrequency <= 1464) {
        multiplier = ((double)(maxPwmValue) / (double)(maxNegPwmFrequency - minNegPwmFrequency));
        pwmMagnitude = maxPwmValue - (int)((double)(pwmFrequency - minNegPwmFrequency) * multiplier);
    }
    else {
        multiplier = ((double)(maxPwmValue) / (double)(maxPosPwmFrequency - minPosPwmFrequency));
        pwmMagnitude = (int)((double)(pwmFrequency - minPosPwmFrequency) * multiplier);
    }
    return pwmMagnitude;
}


bool WiringControl::initializeGPIO() {
    if (wiringPi::wiringPiSetupGpio() == 0) {
        return true;
    }
    else {
        return false;
    }
}

void WiringControl::setPinType(int pinNumber, PinType pinType) {
    switch (pinType) {
        case Digital:
            wiringPi::pinMode(pinNumber, OUTPUT);
            digitalPinStatuses[pinNumber] = Low;
            break;
        case HardwarePWM:
            pwmPinStatuses[pinNumber] = PwmPinStatus {scalePwm(1500, MAX_HARDWARE_PWM_VALUE), 0};
            wiringPi::pinMode(pinNumber, PWM_OUTPUT);
            break;
        case SoftwarePWM:
            pwmPinStatuses[pinNumber] = PwmPinStatus {scalePwm(1500, MAX_SOFTWARE_PWM_VALUE), 0};
            wiringPi::pinMode(pinNumber, OUTPUT);
            break;
        default:
            std::cerr << "Impossible pin type " << pinType << "! Exiting." << std::endl;
            exit(42);
    }
    pinTypes[pinNumber] = pinType;
}

void WiringControl::digitalWrite(int pinNumber, DigitalPinStatus digitalPinStatus) {
    switch (digitalPinStatus) {
        case Low:
            wiringPi::digitalWrite(pinNumber, LOW);
            break;
        case High:
            wiringPi::digitalWrite(pinNumber, HIGH);
            break;
        default:
            std::cerr << "Impossible digital pin status " << digitalPinStatus << "! Exiting." << std::endl;
            exit(42);
    }
    digitalPinStatuses[pinNumber] = digitalPinStatus;
}

DigitalPinStatus WiringControl::digitalRead(int pinNumber) {
    return digitalPinStatuses[pinNumber];
}

void WiringControl::pwmWrite(int pinNumber, int pwmFrequency) {
    switch (pinTypes[pinNumber]) {
        case HardwarePWM:
            wiringPi::pwmWrite(pinNumber, scalePwm(pwmFrequency, MAX_HARDWARE_PWM_VALUE));
            pwmPinStatuses[pinNumber].frequency = scalePwm(pwmFrequency, MAX_HARDWARE_PWM_VALUE);
            break;
        case SoftwarePWM:
            wiringPi::softPwmWrite(pinNumber, scalePwm(pwmFrequency, MAX_SOFTWARE_PWM_VALUE));
            pwmPinStatuses[pinNumber].frequency = scalePwm(pwmFrequency, MAX_SOFTWARE_PWM_VALUE);
            break;
        case Digital:
            std::cerr << "Invalid pin type \"Digital\". Digital pin type cannot be used for PWM. Exiting." << std::endl;
            exit(42);
        default:
            std::cerr << "Impossible pin type " << pinTypes[pinNumber] << "! Exiting." << std::endl;
            exit(42);
    }
}

PwmPinStatus WiringControl::pwmRead(int pinNumber) {
    return pwmPinStatuses[pinNumber];
}

void WiringControl::pwmWriteMaximum(int pinNumber) {
    pwmWrite(pinNumber, 1900);
}

void WiringControl::pwmWriteOff(int pinNumber) {
    pwmWrite(pinNumber, 1500);
}

#else // Use Pi Pico library by default
#include "pico/stdlib.h"
#include "hardware/pwm.h"

bool WiringControl::initializeGPIO() {
    return true;
}

void WiringControl::setPinType(int pinNumber, PinType pinType) {
    switch (pinType) {
        case HardwarePWM:
            pwmPinStatuses[pinNumber] = PwmPinStatus {scalePwm(1500, MAX_HARDWARE_PWM_VALUE), 0};
            mockPinMode(pinNumber, PWM_OUTPUT);
            mockSoftPwmCreate(pinNumber, 0, 100);
            break;
        default:
            std::cerr << "Impossible pin type " << pinType << ". Only hardware PWM is supported on Pi Pico. Exiting." << std::endl;
            exit(42);
    }
    pinTypes[pinNumber] = pinType;
}

void WiringControl::digitalWrite(int pinNumber, DigitalPinStatus digitalPinStatus) {
    switch (digitalPinStatus) {
        case Low:
            mockDigitalWrite(pinNumber, LOW);
            break;
        case High:
            mockDigitalWrite(pinNumber, HIGH);
            break;
        default:
            std::cerr << "Impossible digital pin status " << digitalPinStatus << "! Exiting." << std::endl;
            exit(42);
    }
    digitalPinStatuses[pinNumber] = digitalPinStatus;
}

DigitalPinStatus WiringControl::digitalRead(int pinNumber) {
    return digitalPinStatuses[pinNumber];
}

void WiringControl::pwmWrite(int pinNumber, int pwmFrequency) {
    switch (pinTypes[pinNumber]) {
        case HardwarePWM:
            mockPwmWrite(pinNumber, scalePwm(pwmFrequency, MAX_HARDWARE_PWM_VALUE));
            pwmPinStatuses[pinNumber].frequency = scalePwm(pwmFrequency, MAX_HARDWARE_PWM_VALUE);
            break;
        case SoftwarePWM:
            mockSoftPwmWrite(pinNumber, scalePwm(pwmFrequency, MAX_SOFTWARE_PWM_VALUE));
            pwmPinStatuses[pinNumber].frequency = scalePwm(pwmFrequency, MAX_SOFTWARE_PWM_VALUE);
            break;
        case Digital:
            std::cerr << "Invalid pin type \"Digital\". Digital pin type cannot be used for PWM. Exiting." << std::endl;
            exit(42);
            break;
        default:
            std::cerr << "Impossible pin type " << pinTypes[pinNumber] << "! Exiting." << std::endl;
            exit(42);
    }
}

void WiringControl::pwmWriteMaximum(int pinNumber) {
    pwmWrite(pinNumber, 1900);
}

void WiringControl::pwmWriteOff(int pinNumber) {
    pwmWrite(pinNumber, 1500);
}

PwmPinStatus WiringControl::pwmRead(int pinNumber) {
    return pwmPinStatuses[pinNumber];
}

#endif
#endif