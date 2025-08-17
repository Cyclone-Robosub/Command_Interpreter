#ifndef COMMAND_HPP
#define COMMAND_HPP
#include <vector>
#include <chrono>

struct pwm_array {
    int pwm_signals[8];
};

bool inline operator==(const pwm_array& lhs, const pwm_array& rhs) {
    for (int i = 0; i < 8; i++) {
        if (lhs.pwm_signals[i] != rhs.pwm_signals[i]) {
            return false;
        }
    }
    return true;
}

struct Timed_Command {
    pwm_array thruster_pwms; // PWM values for each thruster
    std::chrono::milliseconds duration; // Duration of the command in milliseconds
};

#endif

