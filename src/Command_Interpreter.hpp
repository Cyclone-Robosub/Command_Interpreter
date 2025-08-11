// William Barber
#pragma once

#include "Command.hpp"
#include "Wiring.hpp"
#include <vector>
#include <fstream>

///@brief Whether a digital pin is active high or active low
enum EnableType {
    ActiveHigh, ActiveLow
};

/*
 * NOTE: We may not need DigitalPin, in which case both DigitalPin and abstract Pin classes are not useful, and can
 * be replaced with just the HardwarePwmPin class (probably renamed to Pin). This would also necessitate the removal of allPins
 * and digitalPins data members from Command_Interpreter_RPi5. Additionally, we probably won't use SoftwarePWM.
 */

/// @brief A Raspberry Pi Pico GPIO pin, as specified by its GPIO pin number (see https://pico.pinout.xyz/)
class Pin {
protected:
    int gpioNumber{};
    std::ostream &output;
    std::ostream &outLog;
    std::ostream &errorLog;
public:
    /// @brief Initializes pin through Wiring Control class (i.e. to output, PWM, etc.)
    virtual void initialize(WiringControl &wiringControl) = 0;

    /// @brief Sets pin to maximum (positive) power
    virtual void enable(WiringControl &wiringControl) = 0;

    /// @brief Sets pin to be unpowered (stopped)
    virtual void disable(WiringControl &wiringControl) = 0;

    /// @brief Whether a pin is currently powered (whether its power level is not zero)
    /// @return True if power is not zero, false otherwise
    virtual bool enabled(WiringControl &wiringControl) = 0;

    /// @brief The pin's current state
    /// @return The current pin status
    virtual int read(WiringControl &wiringControl) = 0;

    /// @param gpioNumber the Pico GPIO number for the pin (see https://pico.pinout.xyz/ and look for GPX labels in green)
    /// @param output where you want output (not logging) messages to be sent (probably std::cout)
    /// @param outLog where you want logging (not error) messages to be logged
    /// @param errorLog where you want error messages to be logged
    explicit Pin(int gpioNumber, std::ostream &output, std::ostream &outLog, std::ostream &errorLog) : gpioNumber(
            gpioNumber),
                                                                                                          output(output),
                                                                                                          outLog(outLog),
                                                                                                          errorLog(
                                                                                                                  errorLog) {}

    virtual ~Pin() = default;
};

/// @brief A digital (two-state) Raspberry Pi Pico GPIO pin
class DigitalPin : public Pin {
private:
    EnableType enableType;
public:
    void initialize(WiringControl &wiringControl) override;

    void enable(WiringControl &wiringControl) override;

    void disable(WiringControl &wiringControl) override;

    bool enabled(WiringControl &wiringControl) override;

    int read(WiringControl &wiringControl) override;

    /// @param gpioNumber the Pico GPIO number for the pin (see https://pico.pinout.xyz/ and look for GPX labels in green)
    /// @param enableType whether the pin is active high or active low
    /// @param output where you want output (not logging) messages to be sent (probably std::cout)
    /// @param outLog where you want logging (not error) messages to be logged
    /// @param errorLog where you want error messages to be logged
    DigitalPin(int gpioNumber, EnableType enableType, std::ostream &output, std::ostream &outLog,
               std::ostream &errorLog) : Pin(gpioNumber, output, outLog, errorLog), enableType(enableType) {};
};

/// @brief A PWM pin which may or may not be hardware-supported
class PwmPin : public Pin {
protected:
    /// @brief Sets pin to the specified pwm value and direction
    /// @param pwmValue a pwm value dictating the power and direction for the thruster
    virtual void setPowerAndDirection(int pwmValue, WiringControl &wiringControl) = 0;
    int maxPwmValue;
    int minPwmValue;

public:
    /// @brief Sets pin to given pwm frequency
    /// @param frequency the desired frequency, between 1100 and 1900
    virtual void setPwm(int frequency, WiringControl &wiringControl);

    /// @brief Sets the minimum and maximum allowed PWM values
    /// @param min the minimum PWM value allowed. Must be at least 1100 and at most 1900.
    /// @param max the maximum PWM value allowed. Must be at least 1100 and at most 1900.
    void setPwmLimits(int min, int max);

    /// @param gpioNumber the Pico GPIO number for the pin (see https://pico.pinout.xyz/ and look for GPX labels in green)
    /// @param output where you want output (not logging) messages to be sent (probably std::cout)
    /// @param outLog where you want logging (not error) messages to be logged
    /// @param errorLog where you want error messages to be logged
    explicit PwmPin(int gpioNumber, std::ostream &output, std::ostream &outLog, std::ostream &errorLog) : Pin(
            gpioNumber, output, outLog, errorLog), maxPwmValue(1800), minPwmValue(1200) {} // Range [1200,1800] to avoid motors drawing too much power
    
    /// @param gpioNumber the Pico GPIO number for the pin (see https://pico.pinout.xyz/ and look for GPX labels in green)
    /// @param output where you want output (not logging) messages to be sent (probably std::cout)
    /// @param outLog where you want logging (not error) messages to be logged
    /// @param errorLog where you want error messages to be logged
    /// @param min the minimum PWM value allowed.
    /// @param max the maximum PWM value allowed.
    explicit PwmPin(int gpioNumber, std::ostream &output, std::ostream &outLog, std::ostream &errorLog,
                    int minPwmValue, int maxPwmValue) : Pin(
            gpioNumber, output, outLog, errorLog), minPwmValue(minPwmValue), maxPwmValue(maxPwmValue) {}


    virtual ~PwmPin() = default;
};

/// @brief a pwm-capable Raspberry Pi Pico GPIO pin (supports analogue output)
class HardwarePwmPin : public PwmPin {
protected:
    /// @brief Sets pin to the specified pwm value and direction
    /// @param pwmValue a pwm value dictating the power and direction for the thruster
    void setPowerAndDirection(int pwmValue, WiringControl &wiringControl) override;

public:
    void initialize(WiringControl &wiringControl) override;

    void enable(WiringControl &wiringControl) override;

    void disable(WiringControl &wiringContro) override;

    bool enabled(WiringControl &wiringControl) override;

    int read(WiringControl &wiringControl) override;

    /// @param gpioNumber the Pico GPIO number for the pin (see https://pico.pinout.xyz/ and look for GPX labels in green)
    /// @param output where you want output (not logging) messages to be sent (probably std::cout)
    /// @param outLog where you want logging (not error) messages to be logged
    /// @param errorLog where you want error messages to be logged
    explicit HardwarePwmPin(int gpioNumber, std::ostream &output, std::ostream &outLog, std::ostream &errorLog)
            : PwmPin(gpioNumber, output, outLog, errorLog) {};
    
    /// @param gpioNumber the Pico GPIO number for the pin (see https://pico.pinout.xyz/ and look for GPX labels in green)
    /// @param output where you want output (not logging) messages to be sent (probably std::cout)
    /// @param outLog where you want logging (not error) messages to be logged
    /// @param errorLog where you want error messages to be logged
    /// @param min the minimum PWM value allowed.
    /// @param max the maximum PWM value allowed.
    explicit HardwarePwmPin(int gpioNumber, std::ostream &output, std::ostream &outLog, std::ostream &errorLog,
                            int minPwmValue, int maxPwmValue)
            : PwmPin(gpioNumber, output, outLog, errorLog, minPwmValue, maxPwmValue) {};
};

/// @brief a Raspberry Pi Pico GPIO pin that doesn't natively support PWM, but that will simulate analogue output
/// through software pwm control.
class SoftwarePwmPin : public PwmPin {
public:
    void initialize(WiringControl &wiringControl) override;

    void enable(WiringControl &wiringControl) override;

    void disable(WiringControl &wiringControl) override;

    bool enabled(WiringControl &wiringControl) override;

    /// @brief Sets pin to the specified pwm value and direction
    /// @param pwmValue a pwm value dictating the power and direction for the thruster
    void setPowerAndDirection(int pwmValue, WiringControl &wiringControl) override;

    int read(WiringControl &wiringControl) override;

    /// @param gpioNumber the Pico GPIO number for the pin (see https://pico.pinout.xyz/ and look for GPX labels in green)
    /// @param output where you want output (not logging) messages to be sent (probably std::cout)
    /// @param outLog where you want logging (not error) messages to be logged
    /// @param errorLog where you want error messages to be logged
    explicit SoftwarePwmPin(int gpioNumber, std::ostream &output, std::ostream &outLog, std::ostream &errorLog)
            : PwmPin(gpioNumber, output, outLog, errorLog) {};
    
    /// @param gpioNumber the Pico GPIO number for the pin (see https://pico.pinout.xyz/ and look for GPX labels in green)
    /// @param output where you want output (not logging) messages to be sent (probably std::cout)
    /// @param outLog where you want logging (not error) messages to be logged
    /// @param errorLog where you want error messages to be logged
    /// @param min the minimum PWM value allowed.
    /// @param max the maximum PWM value allowed.

    explicit SoftwarePwmPin(int gpioNumber, std::ostream &output, std::ostream &outLog, std::ostream &errorLog,
                            int minPwmValue, int maxPwmValue)
            : PwmPin(gpioNumber, output, outLog, errorLog, minPwmValue, maxPwmValue) {};

};

/// @brief The purpose of this class is toggle the GPIO pins on the Raspberry Pi based on a command object.
/// Requires information about wiring, etc.
class Command_Interpreter_RPi5 {
private:
    std::vector<Pin *> allPins();

    std::vector<PwmPin *> thrusterPins;
    std::vector<DigitalPin *> digitalPins;
    WiringControl wiringControl;
    std::ostream &output;
    std::ostream &outLog;
    std::ostream &errorLog;

    bool isInterruptTimed_Execute;

public:
    /// @param thrusterPins the PWM pins that will drive robot thrusters
    /// @param digitalPins non-PWM pins to be used for digital (2-state) output
    /// @param output where you want output (not logging) messages to be sent (probably std::cout)
    /// @param outLog where you want logging (not error) messages to be logged
    /// @param errorLog where you want error messages to be logged
    explicit Command_Interpreter_RPi5(std::vector<PwmPin *> thrusterPins,
                                      std::vector<DigitalPin *> digitalPins,
                                      const WiringControl &wiringControl, std::ostream &output,
                                      std::ostream &outLog, std::ostream &errorLog);

    /// @brief Sends the initialize commands to the Pico
    void initializePins();

    /// @brief Executes a command by sending the specified pwm values to the Pico for the specified duration
    /// @param thrusterPwms a C-style array of pwm frequency integers
    void untimed_execute(pwm_array thrusterPwms);

    /// @brief Executes a command without self-correction. Sets pwm values for the duration specified. Does not stop
    /// thrusters after execution.
    /// @param command a command struct with three sub-components: the acceleration, steady-state, and deceleration.
    void timed_execute(const CommandComponent &command);

    /// @brief Get the current pwm values of all the pins.
    /// @return A vector containing the current value of all pins. PWM pins will return a value in the range [1100, 1900]
    std::vector<int> readPins();

    /// @brief Set an interrupt for the blind_execute function while running. Calling this function sets the interrupt to occur.
    void  interruptTimed_Execute() {isInterruptTimed_Execute = true;}

    ~Command_Interpreter_RPi5(); //TODO this also deletes all its pins. Not sure if this is desirable or not?
};

