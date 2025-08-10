// William Barber

#include "Wiring.hpp"

#include <iostream>
#include <string>

// When compiling for non-RPI devices which cannot run wiringPi library,
// use -MOCK_RPI flag to enable mock functions
#ifdef MOCK_RPI

bool WiringControl::initializeSerial() {
    return true;
}


void WiringControl::printToSerial(const std::string& message) {
    output << message;
}

#else

#include "Serial.hpp"

bool WiringControl::initializeSerial() {
    if ((serial = serialOpen("/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6614864d3798738-if00", 115200)) < 0) {
        return false;
    }
    return true;
}

void WiringControl::printToSerial(const std::string &message) {
    if (serial == -1) {
        output << message;
    } else {
        serialPuts(serial, message.c_str());
    }
}

#endif

WiringControl::WiringControl(std::ostream &output, std::ostream &outLog, std::ostream &errorLog) : output(output),
                                                                                                      outLog(outLog),
                                                                                                      errorLog(
                                                                                                              errorLog) {};

void WiringControl::setPinType(int pinNumber, PinType pinType) {
    std::string message = "Configure ";
    message.append(std::to_string(pinNumber));
    switch (pinType) {
        case DigitalActiveHigh:
            message.append(" Digital\n");
            printToSerial(message);
            pinTypes[pinNumber] = DigitalActiveHigh;
            digitalWrite(pinNumber, Low);
            digitalPinStatuses[pinNumber] = Low;
            break;
        case DigitalActiveLow:
            message.append(" Digital\n");
            printToSerial(message);
            pinTypes[pinNumber] = DigitalActiveLow;
            digitalWrite(pinNumber, High);
            digitalPinStatuses[pinNumber] = High;
            break;
        case HardwarePWM:
            message.append(" HardPwm\n");
            printToSerial(message);
            pinTypes[pinNumber] = HardwarePWM;
            pwmWrite(pinNumber, 1500);
            pwmPinStatuses[pinNumber] = PwmPinStatus{1500, 0};
            break;
        case SoftwarePWM:
            message.append(" SoftPwm\n");
            printToSerial(message);
            pinTypes[pinNumber] = SoftwarePWM;
            pwmWrite(pinNumber, 1500);
            pwmPinStatuses[pinNumber] = PwmPinStatus{1500, 0};
            break;
        default:
            errorLog << "Impossible pin type " << pinType << "! Exiting." << std::endl;
            exit(42);
    }
    pinTypes[pinNumber] = pinType;
}

void WiringControl::digitalWrite(int pinNumber, DigitalPinStatus digitalPinStatus) {
    std::string message = "Set ";
    message.append(std::to_string(pinNumber));
    switch (digitalPinStatus) {
        case Low:
            message.append(" Digital Low\n");
            printToSerial(message);
            digitalPinStatuses[pinNumber] = Low;
            break;
        case High:
            message.append(" Digital High\n");
            printToSerial(message);
            digitalPinStatuses[pinNumber] = High;
            break;
        default:
            errorLog << "Impossible digital pin status " << digitalPinStatus << "! Exiting." << std::endl;
            exit(42);
    }
    digitalPinStatuses[pinNumber] = digitalPinStatus;
}

DigitalPinStatus WiringControl::digitalRead(int pinNumber) {
    return digitalPinStatuses[pinNumber];
}

void WiringControl::pwmWrite(int pinNumber, int pulseWidth) {
    std::string message = "Set ";
    message.append(std::to_string(pinNumber));
    switch (pinTypes[pinNumber]) {
        case HardwarePWM:
        case SoftwarePWM:
            message.append(" PWM ");
            message.append(std::to_string(pulseWidth));
            message.append("\n");
            printToSerial(message);
            pwmPinStatuses[pinNumber].pulseWidth = pulseWidth;
            break;
        case DigitalActiveHigh:
        case DigitalActiveLow:
            errorLog << "Invalid pin type \"Digital\". Digital pin type cannot be used for PWM. Exiting." << std::endl;
            exit(42);
        default:
            errorLog << "Impossible pin type " << pinTypes[pinNumber] << "! Exiting." << std::endl;
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

WiringControl::~WiringControl() {
#ifndef MOCK_RPI
    close(serial);
#endif
}
