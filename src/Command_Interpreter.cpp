// William Barber
#include <iostream>
#include <fstream>
#include <ctime>
#include <utility>
#include "Serial.hpp"
#include "Command_Interpreter.hpp"
#include "Wiring.hpp"

void DigitalPin::initialize(WiringControl &wiringControl) {
    switch (enableType) {
        case ActiveLow:
            wiringControl.setPinType(gpioNumber, DigitalActiveLow);
            break;
        case ActiveHigh:
            wiringControl.setPinType(gpioNumber, DigitalActiveHigh);
            break;
        default:
            errorLog << "Impossible digital pin type " << enableType << "! Exiting." << std::endl;
            exit(42);
    }
}

void DigitalPin::activate(WiringControl &wiringControl) {
    switch (enableType) {
        case ActiveHigh:
            wiringControl.digitalWrite(gpioNumber, High);
            break;
        case ActiveLow:
            wiringControl.digitalWrite(gpioNumber, Low);
            break;
        default:
            errorLog << "Impossible pin mode!" << std::endl;
            exit(42);
    }
}

void DigitalPin::deactivate(WiringControl &wiringControl) {
    switch (enableType) {
        case ActiveHigh:
            wiringControl.digitalWrite(gpioNumber, Low);
            break;
        case ActiveLow:
            wiringControl.digitalWrite(gpioNumber, High);
            break;
        default:
            errorLog << "Impossible pin mode!" << std::endl;
            exit(42);
    }
}

bool DigitalPin::enabled(WiringControl &wiringControl) {
    switch (enableType) {
        case ActiveHigh:
            return wiringControl.digitalRead(gpioNumber) == High;
        case ActiveLow:
            return wiringControl.digitalRead(gpioNumber) == Low;
        default:
            errorLog << "Impossible pin enable type! Exiting." << std::endl;
            exit(42);
    }
}

int DigitalPin::read(WiringControl &wiringControl) {
    return wiringControl.digitalRead(gpioNumber);
}



void PwmPin::setPwm(int pulseWidth, WiringControl &wiringControl) {
    if (pulseWidth > maxPwmValue || pulseWidth < minPwmValue) {
        errorLog << "PWM out of bounds! Value " << pulseWidth << " is out of bounds for range [" <<
            minPwmValue << "," << maxPwmValue << "]. Setting to closest valid value." << std::endl;
        pulseWidth = (pulseWidth > maxPwmValue) ? maxPwmValue : minPwmValue;
    }
    setPowerAndDirection(pulseWidth, wiringControl);
    std::time_t currentTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    outLog << "Current time: " << std::ctime(&currentTime) << std::endl;
    outLog << "Thruster at pin " << gpioNumber << ": " << pulseWidth << std::endl;
}

void PwmPin::setPwmLimits(int min, int max) {
    // These are hardware-enforced limits, i.e. a PWM value of less than 1100 or greater than 1900 don't make sense.
    // If we want to restrict our range, this should be done by calling this function. If that's bothersome because they
    // have to be set for every pin, I recommend changing the default values in the constructor for PwmPin rather than
    // the bounds here.
    if (max < min) {
        errorLog << "Invalid limits! max (value " << max << ") is smaller than min (value " <<
            min << "). Exiting." << std::endl;
        exit(42);
    }
    if (min >= 1100 && min <= 1900) {
        minPwmValue = min;
    }
    else {
        errorLog << "Invalid min pwm value! Attempted to set to " << min <<
            " which is out of range [1100,1900]. Exiting." << std::endl;
        exit(42);
    }
    if (max >= 1100 && max <= 1900) {
        maxPwmValue = max;
    }
    else {
        errorLog << "Invalid max pwm value! Attempted to set to " << max <<
            " which is out of range [1100,1900]. Exiting." << std::endl;
        exit(42);
    }
}

void PwmPin::initialize(WiringControl &wiringControl) {
    wiringControl.setPinType(gpioNumber, PWM);
}

void PwmPin::setPowerAndDirection(int pwmValue, WiringControl &wiringControl) {
    wiringControl.pwmWrite(gpioNumber, pwmValue);
}

int PwmPin::read(WiringControl &wiringControl) {
    return wiringControl.pwmRead(gpioNumber).pulseWidth;
}



Command_Interpreter_RPi5::Command_Interpreter_RPi5(std::vector<PwmPin *> thrusterPins,
                                                   std::vector<DigitalPin *> digitalPins,
                                                   const WiringControl &wiringControl, std::ostream &output,
                                                   std::ostream &outLog, std::ostream &errorLog) :
        thrusterPins(std::move(thrusterPins)), digitalPins(std::move(digitalPins)), wiringControl(wiringControl),
        errorLog(errorLog), outLog(outLog), output(output), isInterruptTimed_Execute(false) {
    if (this->thrusterPins.size() != 8) {
        errorLog << "Incorrect number of thruster pwm pins given! Need 8, given " << this->thrusterPins.size()
                 << std::endl;
        exit(42);
    }
}

std::vector<Pin *> Command_Interpreter_RPi5::allPins() {
    auto allPins = std::vector<Pin *>{};
    allPins.insert(allPins.end(), this->thrusterPins.begin(), this->thrusterPins.end());
    allPins.insert(allPins.end(), this->digitalPins.begin(), this->digitalPins.end());
    return allPins;
}

void Command_Interpreter_RPi5::initializePins() {
    if (!wiringControl.initializeSerial()) {
        errorLog << "Failure to configure serial!" << std::endl;
        exit(42);
    }
    for (Pin *pin: allPins()) {
        pin->initialize(wiringControl);
    }
}

std::vector<int> Command_Interpreter_RPi5::readPins() {
    std::vector<int> pinValues;
    for (auto pin: allPins()) {
        pinValues.push_back(pin->read(wiringControl));
    }
    return pinValues;
}

Command_Interpreter_RPi5::~Command_Interpreter_RPi5() {
    for (auto pin: allPins()) {
        delete pin;
    }
}

void Command_Interpreter_RPi5::timed_execute(const Command &commandComponent) {
    isInterruptTimed_Execute = false;
    auto endTime = std::chrono::system_clock::now() + commandComponent.duration;
    auto currentTime = std::chrono::system_clock::now();
    untimed_execute(commandComponent.thruster_pwms);
    while (currentTime < endTime && !isInterruptTimed_Execute) {
        currentTime = std::chrono::system_clock::now();
    }
    isInterruptTimed_Execute = false;
}

void Command_Interpreter_RPi5::untimed_execute(pwm_array thrusterPwms) {
    int i = 0;
    for (int pulseWidth: thrusterPwms.pwm_signals) {
        thrusterPins.at(i)->setPwm(pulseWidth, wiringControl);
        i++;
    }
}