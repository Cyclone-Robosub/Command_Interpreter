// William Barber
#include "Command_Interpreter.h"
#include <iostream>
#include <utility>

// When compiling for non-RPI devices which cannot run wiringPi library,  
// use -MOCK_RPI flag to enable mock functions
#ifndef MOCK_RPI
#include <wiringPi.h>  // Include wiringPi library by default

#else
#define PWM_OUTPUT 1
#define OUTPUT 0
#define HIGH 1
#define LOW 0
//TODO: wrapper class for wiringPi GTest?
#include <map>
std::map<int,int> pinStatus = {};

int wiringPiSetupGpio() {
    std::cout << "[Mock] wiringPi GPIO set up!" << std::endl;
    return 0;
}

void pinMode(int pinNumber, int mode) {
    if (pinStatus.find(pinNumber) == pinStatus.end()) {
        pinStatus[pinNumber] = 0;
    }
    std::cout << "[Mock] pinMode: pin " << pinNumber << " set to mode " << mode << std::endl;
}

void digitalWrite(int pinNumber, int voltage) {
    pinStatus[pinNumber] = voltage;
    std::cout << "[Mock] digitalWrite: pin " << pinNumber << ", voltage " << voltage << std::endl;
}

void pwmWrite(int pinNumber, int pwm) {
    pinStatus[pinNumber] = pwm;
    std::cout << "[Mock] pwmWrite: pin " << pinNumber << ", PWM " << pwm << std::endl;
}

int digitalRead(int pinNumber) {
    return (pinStatus.find(pinNumber) != pinStatus.end() || pinStatus[pinNumber] > 1) ? pinStatus[pinNumber] : -1;
}

#endif

DigitalPin::DigitalPin(int gpioNumber, EnableType enableType): Pin(gpioNumber), pinStatus(Disabled), enableType(enableType) {}

void DigitalPin::initialize() {
    pinMode(gpioNumber, OUTPUT);
}

void DigitalPin::enable() {
    switch (enableType) {
        case ActiveHigh:
            digitalWrite(gpioNumber, HIGH);
            break;
        case ActiveLow:
            digitalWrite(gpioNumber, LOW);
            break;
        default:
            std::cerr << "Impossible pin mode!" << std::endl;
            exit(42);
    }
    pinStatus = Enabled;
}

void DigitalPin::disable() {
    switch (enableType) {
        case ActiveHigh:
            digitalWrite(gpioNumber, LOW);
            break;
        case ActiveLow:
            digitalWrite(gpioNumber, HIGH);
            break;
        default:
            std::cerr << "Impossible pin mode!" << std::endl;
            exit(42);
    }
    pinStatus = Disabled;
}

bool DigitalPin::enabled() {
    return pinStatus == Enabled;
}

int DigitalPin::read() {
    return digitalRead(gpioNumber);
}


PwmPin::PwmPin(int gpioNumber): Pin(gpioNumber), currentPwm(0) {}

void PwmPin::initialize() {
    pinMode(gpioNumber, PWM_OUTPUT);
}

void PwmPin::enable() {
    setPowerAndDirection(MAX_PWM_VALUE, Forwards);
    currentPwm = MAX_PWM_VALUE;
}

void PwmPin::disable() {
    setPowerAndDirection(0, Forwards);
    currentPwm = 0;
}

bool PwmPin::enabled() {
    return currentPwm != 0;
}

void PwmPin::setPowerAndDirection(int pwmValue, Direction direction) {
    //TODO: set direction (not sure how to do this w/ pwm. High/low?)
    pwmWrite(gpioNumber, pwmValue);
    currentPwm = pwmValue;
}

int PwmPin::read() {
    return currentPwm;
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
    if (wiringPiSetupGpio() == -1) {
        std::cerr << "Failure to configure GPIO pins through wiringPi!" << std::endl;
        exit(42);
    }
    for (Pin* pin : allPins()) {
        pin->initialize();
    }
}

Command_Interpreter_RPi5::ThrusterSpec Command_Interpreter_RPi5::convertPwmValue(double pwmFrequency) {

    // 1100 - 1464 = negative
    // 1536 - 1900 = positive
    const int minPosPwmFrequency = 1535; // slightly less than 1536 so that 1536 is not zero
    const int maxPosPwmFrequency = 1900;
    const int minNegPwmFrequency = 1100;
    const int maxNegPwmFrequency = 1465; // slightly greater than 1464 so that 1464 is not zero
    int pwmMagnitude;
    double multiplier;

    if (pwmFrequency >= 1465 && pwmFrequency <= 1535) {
        return ThrusterSpec{0, Forwards};
    }
    if (pwmFrequency <= 1464) {
        multiplier = ((double)(MAX_PWM_VALUE)/(double)(maxNegPwmFrequency - minNegPwmFrequency));
        pwmMagnitude = MAX_PWM_VALUE - (int)((double)(pwmFrequency - minNegPwmFrequency) * multiplier);
        return ThrusterSpec{pwmMagnitude, Backwards};
    }
    else {
        multiplier = ((double)(MAX_PWM_VALUE)/(double)(maxPosPwmFrequency - minPosPwmFrequency));
        pwmMagnitude = (int)((double)(pwmFrequency - minPosPwmFrequency) * multiplier);
        return ThrusterSpec{pwmMagnitude, Forwards};
    }
}

void Command_Interpreter_RPi5::execute(const Command& command) {
    int i = 0;
    for (int frequency : command.thruster_pwms.pwm_signals) {
        ThrusterSpec thrusterSpec = convertPwmValue(frequency);
        thrusterPins.at(i)->setPowerAndDirection(thrusterSpec.pwm_value, thrusterSpec.direction);
        i++;
    }
    //TODO: duration
}

std::vector<int> Command_Interpreter_RPi5::readPins() {
    std::vector<int> pinValues;
    for (auto pin : allPins()) {
        pinValues.push_back(pin->read());
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
//
//void blind_execute(command_component command, std::ofstream logfile) {
//    execute(command);
//    while (!(time_is_up() || interrupt())) {
//    }
//}
//
//void corrective_execute(command_component command, std::ofstream logfile) {
//    adjusted_command = copy(command)
//    while (!time_is_up()) {
//        adjusted_command = correct_command();
//        execute(command);
//    }
//}
