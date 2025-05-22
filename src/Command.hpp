#ifndef COMMAND_HPP
#define COMMAND_HPP
#include <vector>
#include <chrono>

// A command is a simple instruction to the vehicle
// Commands will be combined in sequencing to create more complex instructions


enum Direction {Forwards, Backwards};

struct force_array{
	float forces[8];
};
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

struct CommandComponent {
    pwm_array thruster_pwms; // PWM values for each thruster
    std::chrono::milliseconds duration; // Duration of the command in milliseconds
};

struct Command {
    CommandComponent acceleration;
    CommandComponent steadyState;
    CommandComponent deceleration;
};

struct Sequence {
	std::vector<Command> commands;
};

#endif

