#include "Command_Interpreter.h"
#include <gtest/gtest.h>

TEST(CommandInterpreterTest, CreateCommandInterpreter) {
    testing::internal::CaptureStdout();

    auto pin1 = new HardwarePwmPin(0);
    auto pin2 = new HardwarePwmPin(1);
    auto pin3 = new HardwarePwmPin(2);
    auto pin4 = new HardwarePwmPin(3);
    auto pin5 = new HardwarePwmPin(4);
    auto pin6 = new HardwarePwmPin(5);
    auto pin7 = new HardwarePwmPin(6);
    auto pin8 = new HardwarePwmPin(7);
    auto pins = std::vector<PwmPin *>{pin1, pin2, pin3, pin4, pin5, pin6, pin7, pin8};


    auto interpreter = new Command_Interpreter_RPi5(pins, std::vector<DigitalPin *>{});
    interpreter->initializePins();
    auto pinStatus = interpreter->readPins();
    std::string output = testing::internal::GetCapturedStdout();

    delete interpreter;

    ASSERT_EQ(pinStatus.size(), 8);
    ASSERT_EQ(pinStatus, (std::vector<int>{0, 0, 0, 0, 0, 0, 0, 0}));
}

TEST(CommandInterpreterTest, CreateCommandInterpreterWithDigitalPins) {
    testing::internal::CaptureStdout();

    auto pin1 = new HardwarePwmPin(0);
    auto pin2 = new HardwarePwmPin(1);
    auto pin3 = new HardwarePwmPin(2);
    auto pin4 = new HardwarePwmPin(3);
    auto pin5 = new HardwarePwmPin(4);
    auto pin6 = new HardwarePwmPin(5);
    auto pin7 = new HardwarePwmPin(6);
    auto pin8 = new HardwarePwmPin(7);
    auto pwmPins = std::vector<PwmPin *>{pin1, pin2, pin3, pin4, pin5, pin6, pin7, pin8};

    auto digital1 = new DigitalPin(8, ActiveLow);
    auto digital2 = new DigitalPin(9, ActiveHigh);
    auto digitalPins = std::vector<DigitalPin*>{digital1, digital2};

    auto interpreter = new Command_Interpreter_RPi5(pwmPins, digitalPins);
    interpreter->initializePins();
    std::string output = testing::internal::GetCapturedStdout();
    auto pinStatus = interpreter->readPins();

    delete interpreter;

    ASSERT_EQ(pinStatus.size(), 10);
    ASSERT_EQ(pinStatus, (std::vector<int>{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
}

TEST(CommandInterpreterTest, BlindExecuteHardwarePwm) {
    testing::internal::CaptureStdout();

    const CommandComponent acceleration = {1500, 1900, 1100,
                                     1250, 1300, 1464, 1535,
                                     1536, std::chrono::milliseconds(2000)};

    auto pin1 = new HardwarePwmPin(0);
    auto pin2 = new HardwarePwmPin(1);
    auto pin3 = new HardwarePwmPin(2);
    auto pin4 = new HardwarePwmPin(3);
    auto pin5 = new HardwarePwmPin(4);
    auto pin6 = new HardwarePwmPin(5);
    auto pin7 = new HardwarePwmPin(6);
    auto pin8 = new HardwarePwmPin(7);
    auto pins = std::vector<PwmPin*>{pin1, pin2, pin3, pin4, pin5, pin6, pin7, pin8};

    std::ofstream logFile = std::ofstream("/dev/null");

    auto interpreter = new Command_Interpreter_RPi5(pins, std::vector<DigitalPin*>{});
    interpreter->initializePins();
    auto startTime = std::chrono::system_clock::now();
    interpreter->blind_execute(acceleration, logFile);
    auto endTime = std::chrono::system_clock::now();
    std::string output = testing::internal::GetCapturedStdout();
    auto pinStatus = interpreter->readPins();

    delete interpreter;

    ASSERT_NEAR((endTime - startTime) / std::chrono::milliseconds(1), std::chrono::milliseconds(2000) /
        std::chrono::milliseconds(1), std::chrono::milliseconds(10) / std::chrono::milliseconds(1));
    ASSERT_EQ(pinStatus, (std::vector<int>{0, 1023, 1023, 603, 463, 3, 0, 2}));
}

TEST(CommandInterpreterTest, BlindExecuteSoftwarePwm) {
    testing::internal::CaptureStdout();

    const CommandComponent acceleration = {1500, 1900, 1100,
                                           1250, 1300, 1464, 1535,
                                           1536, std::chrono::milliseconds(2000)};

    auto pin1 = new SoftwarePwmPin(0);
    auto pin2 = new SoftwarePwmPin(1);
    auto pin3 = new SoftwarePwmPin(2);
    auto pin4 = new SoftwarePwmPin(3);
    auto pin5 = new SoftwarePwmPin(4);
    auto pin6 = new SoftwarePwmPin(5);
    auto pin7 = new SoftwarePwmPin(6);
    auto pin8 = new SoftwarePwmPin(7);
    auto pins = std::vector<PwmPin*>{pin1, pin2, pin3, pin4, pin5, pin6, pin7, pin8};

    std::ofstream logFile = std::ofstream("/dev/null");

    auto interpreter = new Command_Interpreter_RPi5(pins, std::vector<DigitalPin*>{});
    interpreter->initializePins();
    auto startTime = std::chrono::system_clock::now();
    interpreter->blind_execute(acceleration, logFile);
    auto endTime = std::chrono::system_clock::now();
    std::string output = testing::internal::GetCapturedStdout();
    auto pinStatus = interpreter->readPins();

    delete interpreter;

    ASSERT_NEAR((endTime - startTime) / std::chrono::milliseconds(1), std::chrono::milliseconds(2000) /
        std::chrono::milliseconds(1), std::chrono::milliseconds(10) / std::chrono::milliseconds(1));

    ASSERT_EQ(pinStatus, (std::vector<int>{0, 100, 100, 59, 46, 1, 0, 0}));
}

