// William Barber
#include "Command_Interpreter.h"
#include "Wiring.h"
#include <iostream>
#include <fstream>
#include <ctime>
#include <utility>

DigitalPin::DigitalPin(int gpioNumber, EnableType enableType): Pin(gpioNumber), enableType(enableType) {}

void DigitalPin::initialize(WiringControl& wiringControl) {
    wiringControl.setPinType(gpioNumber, Digital);
}

void DigitalPin::enable(WiringControl& wiringControl) {
    switch (enableType) {
        case ActiveHigh:
            wiringControl.digitalWrite(gpioNumber, High);
            break;
        case ActiveLow:
            wiringControl.digitalWrite(gpioNumber, Low);
            break;
        default:
            std::cerr << "Impossible pin mode!" << std::endl;
            exit(42);
    }
}

void DigitalPin::disable(WiringControl& wiringControl) {
    switch (enableType) {
        case ActiveHigh:
            wiringControl.digitalWrite(gpioNumber, Low);
            break;
        case ActiveLow:
            wiringControl.digitalWrite(gpioNumber, High);
            break;
        default:
            std::cerr << "Impossible pin mode!" << std::endl;
            exit(42);
    }
}

bool DigitalPin::enabled(WiringControl& wiringControl) {
    switch (enableType) {
        case ActiveHigh:
            return wiringControl.digitalRead(gpioNumber) == High;
            break;
        case ActiveLow:
            return wiringControl.digitalRead(gpioNumber) == Low;
            break;
        default:
            std::cerr << "Impossible pin enable type! Exiting." << std::endl;
            exit(42);
    }
}

int DigitalPin::read(WiringControl& wiringControl) {
    return wiringControl.digitalRead(gpioNumber);
}

void PwmPin::setPwm(int frequency, WiringControl& wiringControl, std::ofstream &logFile) {
    setPowerAndDirection(frequency, wiringControl);
    std::time_t currentTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    logFile << "Current time: " << std::ctime(&currentTime) << std::endl;
    logFile << "Thruster at pin " << gpioNumber << ": " << frequency << std::endl;
}

HardwarePwmPin::HardwarePwmPin(int gpioNumber): PwmPin(gpioNumber) {}

void HardwarePwmPin::initialize(WiringControl& wiringControl) {
    wiringControl.setPinType(gpioNumber, HardwarePWM);
}

void HardwarePwmPin::enable(WiringControl& wiringControl) {
    wiringControl.pwmWriteMaximum(gpioNumber);
}

void HardwarePwmPin::disable(WiringControl& wiringControl) {
    wiringControl.pwmWriteOff(gpioNumber);
}

bool HardwarePwmPin::enabled(WiringControl& wiringControl) {
    return wiringControl.pwmRead(gpioNumber).frequency != 1500;
}

void HardwarePwmPin::setPowerAndDirection(int pwmValue, WiringControl& wiringControl) {
    wiringControl.pwmWrite(gpioNumber, pwmValue);
}

int HardwarePwmPin::read(WiringControl& wiringControl) {
    return wiringControl.pwmRead(gpioNumber).frequency;
}

SoftwarePwmPin::SoftwarePwmPin(int gpioNumber): PwmPin(gpioNumber) {}

void SoftwarePwmPin::initialize(WiringControl& wiringControl) {
    wiringControl.setPinType(gpioNumber, SoftwarePWM);
}

void SoftwarePwmPin::enable(WiringControl& wiringControl) {
    wiringControl.pwmWriteMaximum(gpioNumber);
}

void SoftwarePwmPin::disable(WiringControl& wiringControl) {
    wiringControl.pwmWriteOff(gpioNumber);
}

bool SoftwarePwmPin::enabled(WiringControl& wiringControl) {
    return wiringControl.pwmRead(gpioNumber).frequency != 1500;
}

void SoftwarePwmPin::setPowerAndDirection(int pwmValue, WiringControl& wiringControl) {
    wiringControl.pwmWrite(gpioNumber, pwmValue);
}

int SoftwarePwmPin::read(WiringControl& wiringControl) {
    return wiringControl.pwmRead(gpioNumber).frequency;
}

Command_Interpreter_RPi5::Command_Interpreter_RPi5(std::vector<PwmPin*> thrusterPins, std::vector<DigitalPin*> digitalPins):
                                                thrusterPins(std::move(thrusterPins)), digitalPins(std::move(digitalPins)) {
    if (this->thrusterPins.size() != 8) {
        std::cerr << "Incorrect number of thruster pwm pins given! Need 8, given " << this->thrusterPins.size() << std::endl;
        exit(42);
    }
}

std::vector<Pin*> Command_Interpreter_RPi5::allPins() {
    auto allPins = std::vector<Pin*>{};
    allPins.insert(allPins.end(), this->thrusterPins.begin(), this->thrusterPins.end());
    allPins.insert(allPins.end(), this->digitalPins.begin(), this->digitalPins.end());
    return allPins;
}

void Command_Interpreter_RPi5::initializePins() {
    if (!wiringControl.initializeGPIO()) {
        std::cerr << "Failure to configure GPIO pins!" << std::endl;
        exit(42);
    }
    for (Pin* pin : allPins()) {
        pin->initialize(wiringControl);
    }
}

void Command_Interpreter_RPi5::execute(pwm_array thrusterPwms, std::ofstream& logFile) {
    int i = 0;
    for (int frequency : thrusterPwms.pwm_signals) {
        thrusterPins.at(i)->setPwm(frequency, wiringControl, logFile);
        i++;
    }
}

std::vector<int> Command_Interpreter_RPi5::readPins() {
    std::vector<int> pinValues;
    for (auto pin : allPins()) {
        pinValues.push_back(pin->read(wiringControl));
    }
    return pinValues;
}

Command_Interpreter_RPi5::~Command_Interpreter_RPi5() {
    for (auto pin: allPins()) {
        delete pin;
    }
}

//move_forwards(float distance, std::ofstream logfile) {
//    command = calculate_stuff(distance, current_robot_data);
//    blind_execute(command.accel_command, logfile); //accel
//    blind_execute(command.ss_command, logfile); //stead-state
//    blind_execute(command.dec_command, logfile); //decel
//    // At this point, a new thread/command sends new commands. This can be stop, or it can be a new direction, etc.
//}

void Command_Interpreter_RPi5::blind_execute(const CommandComponent& commandComponent, std::ofstream& logfile) {
    auto endTime = std::chrono::system_clock::now() + commandComponent.duration;
    auto currentTime = std::chrono::system_clock::now();
    execute(commandComponent.thruster_pwms, logfile);
    while (currentTime < endTime) {
        currentTime = std::chrono::system_clock::now();
    }
}

//void Command_Interpreter_RPi5::corrective_execute(command_component command, std::ofstream logfile) {
//    adjusted_command = copy(command)
//    while (!time_is_up()) {
//        adjusted_command = correct_command();
//        execute(command);
//    }
//}
