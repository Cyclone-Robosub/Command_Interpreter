#include "Command_Interpreter.hpp"
#include <gtest/gtest.h>

#ifndef MOCK_RPI

#include "Serial.hpp"

#else

bool initializeSerial(int* serial, bool testing) {
    return true;
}

int getSerialChar(int* serial, bool testing) {
    return EOF;
}

#endif

TEST(CommandInterpreterTest, CreateCommandInterpreter) {
    testing::internal::CaptureStdout();
    std::ofstream outLog("/dev/null");
    int serial = -1;
    initializeSerial(&serial, true);

    auto pinNumbers = std::vector<int>{8, 9, 6, 7, 13, 11, 12, 10};

    auto pins = std::vector<PwmPin *>{};

    for (int pinNumber: pinNumbers) {
        pins.push_back(new PwmPin(pinNumber, std::cout, outLog, std::cerr));
    }

    WiringControl wiringControl = WiringControl(std::cout, outLog, std::cerr);

    auto interpreter = new Command_Interpreter_RPi5(pins, std::vector<DigitalPin *>{}, wiringControl, std::cout, outLog,
                                                    std::cerr);
    interpreter->initializePins();
    auto pinStatus = interpreter->readPins();
    std::string output = testing::internal::GetCapturedStdout();

    delete interpreter;

    std::string expectedOutput;
    for (int pinNumber: pinNumbers) {
        expectedOutput.append("Configure ");
        expectedOutput.append(std::to_string(pinNumber));
        expectedOutput.append(" PWM\nSet ");
        expectedOutput.append(std::to_string(pinNumber));
        expectedOutput.append(" PWM 1500\n");
    }

    int charRead = EOF;
    std::string serialOutput;
    while ((charRead = getSerialChar(&serial, true)) != EOF) {
        serialOutput.push_back((char) charRead);
    }

    ASSERT_EQ(pinStatus.size(), 8);
    ASSERT_EQ(pinStatus, (std::vector<int>{1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}));
    if (serialOutput.empty()) {
        ASSERT_EQ(output, expectedOutput);
    } else {
        expectedOutput.insert(0, "echo on\n");
        ASSERT_EQ(serialOutput, expectedOutput);
    }
}

TEST(CommandInterpreterTest, CreateCommandInterpreterWithDigitalPins) {
    testing::internal::CaptureStdout();
    std::ofstream outLog("/dev/null");
    int serial = -1;
    initializeSerial(&serial, true);

    auto pinNumbers = std::vector<int>{8, 9, 6, 7, 13, 11, 12, 10};

    auto pwmPins = std::vector<PwmPin *>{};

    for (int pinNumber: pinNumbers) {
        pwmPins.push_back(new PwmPin(pinNumber, std::cout, outLog, std::cerr, 1100, 1900));
    }

    WiringControl wiringControl = WiringControl(std::cout, outLog, std::cerr);

    auto digital1 = new DigitalPin(8, ActiveLow, std::cout, outLog, std::cerr);
    auto digital2 = new DigitalPin(9, ActiveHigh, std::cout, outLog, std::cerr);
    auto digitalPins = std::vector<DigitalPin *>{digital1, digital2};

    auto interpreter = new Command_Interpreter_RPi5(pwmPins, digitalPins, wiringControl, std::cout, outLog,
                                                    std::cerr);
    interpreter->initializePins();
    std::string output = testing::internal::GetCapturedStdout();
    auto pinStatus = interpreter->readPins();

    delete interpreter;

    std::string expectedOutput;
    for (int pinNumber: pinNumbers) {
        expectedOutput.append("Configure ");
        expectedOutput.append(std::to_string(pinNumber));
        expectedOutput.append(" PWM\nSet ");
        expectedOutput.append(std::to_string(pinNumber));
        expectedOutput.append(" PWM 1500\n");
    }
    expectedOutput.append("Configure 8 Digital\nSet 8 Digital High\n");
    expectedOutput.append("Configure 9 Digital\nSet 9 Digital Low\n");

    int charRead = EOF;
    std::string serialOutput;
    while ((charRead = getSerialChar(&serial, true)) != EOF) {
        serialOutput.push_back((char) charRead);
    }
    ASSERT_EQ(pinStatus.size(), 10);
    ASSERT_EQ(pinStatus, (std::vector<int>{1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1, 0}));
    if (serialOutput.empty()) {
        ASSERT_EQ(output, expectedOutput);
    } else {
        expectedOutput.insert(0, "echo on\n");
        ASSERT_EQ(serialOutput, expectedOutput);
    }
}

TEST(CommandInterpreterTest, UntimedExecute) {
    testing::internal::CaptureStdout();
    std::ofstream outLog("/dev/null");
    int serial = -1;
    initializeSerial(&serial, true);

    const pwm_array pwms = {1900, 1900, 1100, 1250, 1300, 1464, 1535, 1536};

    auto pinNumbers = std::vector<int>{8, 9, 6, 7, 13, 11, 12, 10};

    auto pins = std::vector<PwmPin *>{};

    for (int pinNumber: pinNumbers) {
        pins.push_back(new PwmPin(pinNumber, std::cout, outLog, std::cerr, 1100, 1900));
    }

    WiringControl wiringControl = WiringControl(std::cout, outLog, std::cerr);

    auto interpreter = new Command_Interpreter_RPi5(pins, std::vector<DigitalPin *>{}, wiringControl, std::cout, outLog,
                                                    std::cerr);
    interpreter->initializePins();
    interpreter->untimed_execute(pwms);
    std::string output = testing::internal::GetCapturedStdout();
    auto pinStatus = interpreter->readPins();

    delete interpreter;

    std::string expectedOutput;
    for (int pinNumber: pinNumbers) {
        expectedOutput.append("Configure ");
        expectedOutput.append(std::to_string(pinNumber));
        expectedOutput.append(" PWM\nSet ");
        expectedOutput.append(std::to_string(pinNumber));
        expectedOutput.append(" PWM 1500\n");
    }
    expectedOutput.append("Set 8 PWM 1900\n");
    expectedOutput.append("Set 9 PWM 1900\n");
    expectedOutput.append("Set 6 PWM 1100\n");
    expectedOutput.append("Set 7 PWM 1250\n");
    expectedOutput.append("Set 13 PWM 1300\n");
    expectedOutput.append("Set 11 PWM 1464\n");
    expectedOutput.append("Set 12 PWM 1535\n");
    expectedOutput.append("Set 10 PWM 1536\n");

    int charRead = EOF;
    std::string serialOutput;
    while ((charRead = getSerialChar(&serial, true)) != EOF) {
        serialOutput.push_back((char) charRead);
    }
    ASSERT_EQ(pinStatus, (std::vector<int>{1900, 1900, 1100, 1250, 1300, 1464, 1535, 1536}));
    if (serialOutput.empty()) {
        ASSERT_EQ(output, expectedOutput);
    } else {
        expectedOutput.insert(0, "echo on\n");
        ASSERT_EQ(serialOutput, expectedOutput);
    }
}

TEST(CommandInterpreterTest, TimedExecute) {
    testing::internal::CaptureStdout();
    std::ofstream outLog("/dev/null");
    int serial = -1;
    initializeSerial(&serial, true);

    const Timed_Command command = {1900, 1900, 1100,
                                           1250, 1300, 1464, 1535,
                                           1536, std::chrono::milliseconds(2000)};

    auto pinNumbers = std::vector<int>{8, 9, 6, 7, 13, 11, 12, 10};

    auto pins = std::vector<PwmPin *>{};

    for (int pinNumber: pinNumbers) {
        pins.push_back(new PwmPin(pinNumber, std::cout, outLog, std::cerr, 1100, 1900));
    }

    WiringControl wiringControl = WiringControl(std::cout, outLog, std::cerr);

    auto interpreter = new Command_Interpreter_RPi5(pins, std::vector<DigitalPin *>{}, wiringControl, std::cout, outLog,
                                                    std::cerr);
    interpreter->initializePins();
    auto startTime = std::chrono::system_clock::now();
    interpreter->timed_execute(command);
    auto endTime = std::chrono::system_clock::now();
    std::string output = testing::internal::GetCapturedStdout();
    auto pinStatus = interpreter->readPins();

    delete interpreter;

    std::string expectedOutput;
    for (int pinNumber: pinNumbers) {
        expectedOutput.append("Configure ");
        expectedOutput.append(std::to_string(pinNumber));
        expectedOutput.append(" PWM\nSet ");
        expectedOutput.append(std::to_string(pinNumber));
        expectedOutput.append(" PWM 1500\n");
    }
    expectedOutput.append("Set 8 PWM 1900\n");
    expectedOutput.append("Set 9 PWM 1900\n");
    expectedOutput.append("Set 6 PWM 1100\n");
    expectedOutput.append("Set 7 PWM 1250\n");
    expectedOutput.append("Set 13 PWM 1300\n");
    expectedOutput.append("Set 11 PWM 1464\n");
    expectedOutput.append("Set 12 PWM 1535\n");
    expectedOutput.append("Set 10 PWM 1536\n");

    int charRead = EOF;
    std::string serialOutput;
    while ((charRead = getSerialChar(&serial, true)) != EOF) {
        serialOutput.push_back((char) charRead);
    }
    ASSERT_NEAR((endTime - startTime) / std::chrono::milliseconds(1), std::chrono::milliseconds(2000) /
                                                                      std::chrono::milliseconds(1),
                std::chrono::milliseconds(10) / std::chrono::milliseconds(1));
    ASSERT_EQ(pinStatus, (std::vector<int>{1900, 1900, 1100, 1250, 1300, 1464, 1535, 1536}));
    if (serialOutput.empty()) {
        ASSERT_EQ(output, expectedOutput);
    } else {
        expectedOutput.insert(0, "echo on\n");
        ASSERT_EQ(serialOutput, expectedOutput);
    }
}

TEST(CommandInterpreterTest, LimitTooLow) {
    std::ofstream outLog("/dev/null");

    auto newPin = new PwmPin(0, std::cout, outLog, std::cerr);
    ASSERT_EXIT(newPin->setPwmLimits(1099,1800), testing::ExitedWithCode(42), 
        "Invalid min pwm value! Attempted to set to 1099 which is out of range \\[1100,1900\\]. Exiting.");
}

TEST(CommandInterpreterTest, LimitTooHigh) {
    std::ofstream outLog("/dev/null");

    auto newPin = new PwmPin(0, std::cout, outLog, std::cerr);
    ASSERT_EXIT(newPin->setPwmLimits(1100,1901), testing::ExitedWithCode(42), 
        "Invalid max pwm value! Attempted to set to 1901 which is out of range \\[1100,1900\\]. Exiting.");
}

TEST(CommandInterpreterTest, BadLimitsMaxLessThanMin) {
    std::ofstream outLog("/dev/null");

    auto newPin = new PwmPin(0, std::cout, outLog, std::cerr);
    ASSERT_EXIT(newPin->setPwmLimits(1900,1100), testing::ExitedWithCode(42), 
        "Invalid limits! max \\(value 1100\\) is smaller than min \\(value 1900\\). Exiting.");
}

TEST(CommandInterpreterTest, PWMTooLarge) {
    testing::internal::CaptureStdout();
    testing::internal::CaptureStderr();
    std::ofstream outLog("/dev/null");
    int serial = -1;
    initializeSerial(&serial, true);

    const pwm_array pwms = {1900, 1900, 1900, 1900, 1900, 1900, 1900, 1900};

    auto pinNumbers = std::vector<int>{8, 9, 6, 7, 13, 11, 12, 10};

    auto pins = std::vector<PwmPin *>{};

    for (int pinNumber: pinNumbers) {
        auto newPin = new PwmPin(pinNumber, std::cout, outLog, std::cerr);
        newPin->setPwmLimits(1200,1800);
        pins.push_back(newPin);
    }

    WiringControl wiringControl = WiringControl(std::cout, outLog, std::cerr);

    auto interpreter = new Command_Interpreter_RPi5(pins, std::vector<DigitalPin *>{}, wiringControl, std::cout, outLog,
                                                    std::cerr);
    interpreter->initializePins();
    interpreter->untimed_execute(pwms);
    std::string error = testing::internal::GetCapturedStderr();
    testing::internal::GetCapturedStdout();
    auto pinStatus = interpreter->readPins();

    delete interpreter;

    std::string expectedError;
    for (int pinNumber : pinNumbers){
        expectedError.append("PWM out of bounds! Value 1900 is out of bounds for range [1200,1800]. Setting to closest valid value.\n");
    } 

    ASSERT_EQ(pinStatus, (std::vector<int>{1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800}));
    ASSERT_EQ(error, expectedError);
}

TEST(CommandInterpreterTest, PWMTooSmall) {
    testing::internal::CaptureStdout();
    testing::internal::CaptureStderr();
    std::ofstream outLog("/dev/null");
    int serial = -1;
    initializeSerial(&serial, true);

    const pwm_array pwms = {1100, 1100, 1100, 1100, 1100, 1100, 1100, 1100};

    auto pinNumbers = std::vector<int>{8, 9, 6, 7, 13, 11, 12, 10};

    auto pins = std::vector<PwmPin *>{};

    for (int pinNumber: pinNumbers) {
        auto newPin = new PwmPin(pinNumber, std::cout, outLog, std::cerr, 1000, 1900);
        newPin->setPwmLimits(1200,1800);
        pins.push_back(newPin);
    }

    WiringControl wiringControl = WiringControl(std::cout, outLog, std::cerr);

    auto interpreter = new Command_Interpreter_RPi5(pins, std::vector<DigitalPin *>{}, wiringControl, std::cout, outLog,
                                                    std::cerr);
    interpreter->initializePins();
    interpreter->untimed_execute(pwms);
    std::string error = testing::internal::GetCapturedStderr();
    testing::internal::GetCapturedStdout();
    auto pinStatus = interpreter->readPins();

    delete interpreter;

    std::string expectedError;
    for (int pinNumber : pinNumbers){
        expectedError.append("PWM out of bounds! Value 1100 is out of bounds for range [1200,1800]. Setting to closest valid value.\n");
    } 

    ASSERT_EQ(pinStatus, (std::vector<int>{1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200}));
    ASSERT_EQ(error, expectedError);
}
