// William Barber
#pragma once
#include "Command.h"
#include <vector>

/// @brief Whether a pin is currently in an enabled (powered) or disabled (unpowered) state
enum PinStatus {Enabled, Disabled};

///@brief Whether a digital pin is active high or active low
enum EnableType {ActiveHigh, ActiveLow};

const int MAX_PWM_VALUE = 1023; //TODO: check pwm range (currently assuming 0-1023)

/*
 * NOTE: We may not need DigitalPin, in which case both DigitalPin and abstract Pin classes are not useful, and can
 * be replaced with just the PwmPin class (probably renamed to Pin). This would also necessitate the removal of allPins
 * and digitalPins data members from Command_Interpreter_RPi5
 */

/// @brief A Raspberry Pi 5 GPIO pin, as specified by its GPIO pin number (see https://pinout.xyz)
class Pin {
protected:
    int gpioNumber{};
public:
    /// @brief Initializes pin through WiringPi (i.e. to output, PWM, etc.)
    virtual void initialize()= 0;

    /// @brief Sets pin to maximum (positive) power
    virtual void enable() = 0;

    /// @brief Sets pin to be unpowered (stopped)
    virtual void disable() = 0;

    /// @brief Whether a pin is currently powered (whether its power level is not zero)
    /// @return True if power is not zero, false otherwise
    virtual bool enabled() = 0;

    /// @brief The pin's current state
    /// @return The current pin status
    virtual int read() = 0;
    explicit Pin(int gpioNumber) : gpioNumber(gpioNumber) {}
    virtual ~Pin() = default;
};

/// @brief A digital (two-state) Raspberry Pi 5 GPIO pin
class DigitalPin : public Pin {
private:
    EnableType enableType;
    PinStatus pinStatus; //TODO: can remove this and just rely on digitalReads from the pins directly
public:
    void initialize() override;
    void enable() override;
    void disable() override;
    bool enabled() override;
    int read() override;
    DigitalPin(int gpioNumber, EnableType enableType);
};

/// @brief a pwm-capable Raspberry Pi 5 GPIO pin (supports analogue output)
class PwmPin : public Pin {
private:
    int currentPwm;
public:
    void initialize() override;
    void enable() override;
    void disable() override;
    bool enabled() override;

    /// @brief Sets pin to the specified pwm value and direction
    /// @param pwmValue a pwm value between 0 and 1023
    /// @param direction which direction the motor should spin (Forwards or Backwards)
    void setPowerAndDirection(int pwmValue, Direction direction);
    int read() override;
    explicit PwmPin(int gpioNumber);
};

/// @brief The purpose of this class is toggle the GPIO pins on the Raspberry Pi based on a command object.
/// Requires information about wiring, etc.
class Command_Interpreter_RPi5 {
private:
    /// @brief The spec for a thruster command to be sent to GPIO
    struct ThrusterSpec {
        int pwm_value;
        Direction direction;
    };
    std::vector<Pin*> allPins();
    std::vector<PwmPin*> thrusterPins;
    std::vector<DigitalPin*> digitalPins;

    /// @brief Converts a pwm frequency between 1100 and 1900 into a Thruster Spec.
    /// @param pwmFrequency a pwm frequency between 1100 and 1900
    /// @return A Thruster Spec with a pwm magnitude between 0 and MAX_PWM_VALUE and a Direction
    static ThrusterSpec convertPwmValue(double pwmFrequency);
public:
    explicit Command_Interpreter_RPi5(std::vector<PwmPin*> thrusterPins, std::vector<DigitalPin*> digitalPins);

    /// @brief Initializes the pins in allPins through the WiringPi library to be either PWM or digital pins
    /// depending on their types
    void initializePins();

    /// @brief Executes a command by sending the specified pwm values to the Pi's GPIO for the specified duration
    /// @param command a command struct with a C-style array of pwm frequency integers and a duration float
    void execute(const Command& command);

    /// @brief Executes a command without self-correction. Sets pwm values for the duration specified. Does not stop
    /// thrusters after execution.
    /// @param command a command struct with three sub-components: the acceleration, steady-state, and deceleration.
    void blind_execute(const Command& command);

    std::vector<int> readPins();

    ~Command_Interpreter_RPi5(); //TODO this also deletes all its pins. Not sure if this is desirable or not?
};

