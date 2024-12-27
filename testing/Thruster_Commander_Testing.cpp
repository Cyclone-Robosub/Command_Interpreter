#include "Thruster_Commander.h"
#include <gtest/gtest.h>


TEST(ThrusterCommanderTest, Accel_Time_x) {
	auto control = new Thruster_Commander();
	
	float v0 = 0.999 * control->top_speed_x(true);
	float t0 = control->accel_time_x(0, v0);
	ASSERT_TRUE(t0 > 0);

    float v1 = v0 / 2;
    float t1 = control->accel_time_x(0, v1);

    ASSERT_TRUE(t1 > 0);
	ASSERT_TRUE(t1 < t0);

    // todo: fix the recursion error that occours here in some environments
    //float v2 = v0;
    //float t2 = control->accel_time_x(v1, v2);
    //ASSERT_TRUE(t2 > t1);
    //ASSERT_NEAR(t2 + t1, t0, 0.00001);

    float v3 = 0.999 * control->top_speed_x(false);
    float t3 = control->accel_time_x(0, v3);
    ASSERT_TRUE(t3 > 0);

    float v4 = v3 / 2.0;
    float t4 = control->accel_time_x(0, v4);

    ASSERT_TRUE(t4 > 0);
    ASSERT_TRUE(t4 < t3);

    float v5 = v3;
    float t5 = control->accel_time_x(v4, v5);
    ASSERT_TRUE(t5 > t4);
    ASSERT_NEAR(t4 + t5, t3, 0.00001);


	
 

    // TODO: double check math on seperable DE
    // TODO: add assertations based on config file, math
    delete control;
}
TEST(ThrusterCommanderTest, Thrust_Compute_General) {
    auto control = new Thruster_Commander();

    six_axis tolerances = { 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001 };
    six_axis desired = { 3.0f,2.0f,1.0f,8.0f,9.0f,3.0f};

    thruster_set thrusts = control->thrust_compute_general(desired(0), desired(1), desired(2), desired(3), desired(4), desired(5));
    six_axis result = control->net_force_from_thrusters(thrusts);

    for (int i = 0; i < 6; i++) {
        ASSERT_NEAR(result(i), desired(i), tolerances(i));
    }
    delete control;
}
TEST(ThrusterCommanderTest, Top_Speed_X) {
	auto control = new Thruster_Commander();
	float max_fwd = control->top_speed_x(true);
	float max_rev = control->top_speed_x(false);

    // TODO: calculate asserts based on config file
	ASSERT_TRUE(max_fwd > 0);
	ASSERT_TRUE(max_rev < 0);
}

TEST(ThrusterCommanderTest, Thrust_Compute_Fx) {
	auto control = new Thruster_Commander();

    // TODO: decrease tolerances by using vertical thrusters to correct mx, my
	six_axis tolerances = { 0.001, 0.001, 0.001, 0.5, 0.5, 0.001 };
    six_axis desired = { 3,0,0,0,0,0 };
    thruster_set thrusts = control->thrust_compute_fx(desired(0));
    six_axis result = control->net_force_from_thrusters(thrusts);
	for (int i = 0; i < 6; i++) {
		ASSERT_NEAR(result(i), desired(i), tolerances(i));
	}
    delete control;
}
TEST(ThrusterCommanderTest, Thrust_Compute_Fy) {
    auto control = new Thruster_Commander();
    
    // TODO: decrease tolerances by using vertical thrusters to correct mx, my
    six_axis tolerances = { 0.001, 0.001, 0.001, 0.5, 0.5, 0.1 };
    six_axis desired = { 0,3,0,0,0,0 };
    thruster_set thrusts = control->thrust_compute_fy(desired(1));
    six_axis result = control->net_force_from_thrusters(thrusts);
    for (int i = 0; i < 6; i++) {
        ASSERT_NEAR(result(i), desired(i), tolerances(i));
    }
    delete control;
}
TEST(ThrusterCommanderTest, Thrust_Compute_Fz) {
    auto control = new Thruster_Commander();

    six_axis tolerances = { 0.001, 0.001, 0.001, 0.1, 0.1, 0.001 };
    six_axis desired = { 0,0,3,0,0,0 };
    thruster_set thrusts = control->thrust_compute_fz(desired(2));
    six_axis result = control->net_force_from_thrusters(thrusts);

    for (int i = 0; i < 6; i++) {
        ASSERT_NEAR(result(i), desired(i), tolerances(i));
    }
    delete control;
}
TEST(ThursterCommanderTest, Thrust_Compute_fx_fy_mz) {
    auto control = new Thruster_Commander();

    six_axis tolerances = { 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001 };
    six_axis desired = { 3.0f,2.0f,0.0f,0.0f,0.0f,1.0f};

    thruster_set thrusts = control->thrust_compute_fx_fy_mz(desired(0), desired(1), desired(5));
    six_axis result = control->net_force_from_thrusters(thrusts);

    for (int i = 0; i < 6; i++) {
        ASSERT_NEAR(result(i), desired(i), tolerances(i));
    }

    delete control;
}
TEST(ThrusterCommanderTest, GetPwmNoInterpolation) {
    auto thrusterCommander = new Thruster_Commander();
    double result = thrusterCommander->get_pwm(1, -3.52);
    ASSERT_DOUBLE_EQ(result, 1100);

    result = thrusterCommander->get_pwm(1, -3);
    ASSERT_DOUBLE_EQ(result, 1148);

    result = thrusterCommander->get_pwm(1, -0.03);
    ASSERT_DOUBLE_EQ(result, 1464);

    result = thrusterCommander->get_pwm(1, 0);
    ASSERT_DOUBLE_EQ(result, 1468);

    result = thrusterCommander->get_pwm(1, 0.05);
    ASSERT_DOUBLE_EQ(result, 1536);

    result = thrusterCommander->get_pwm(1, 1);
    ASSERT_DOUBLE_EQ(result, 1652);

    result = thrusterCommander->get_pwm(1, 4.52);
    ASSERT_DOUBLE_EQ(result, 1900);

    result = thrusterCommander->get_pwm(1, 4.53);
    ASSERT_DOUBLE_EQ(result, 1896);

    delete thrusterCommander;
}

TEST(ThrusterCommanderTest, GetPwmWithInterpolation) {
    auto thrusterCommander = new Thruster_Commander();
    double result = thrusterCommander->get_pwm(1, -3.51);
    ASSERT_NEAR(result, 1102.0000, 0.1);

    result = thrusterCommander->get_pwm(1, -3.46);
    ASSERT_NEAR(result, 1111.0000, 0.1);

    result = thrusterCommander->get_pwm(1, -2.99);
    ASSERT_NEAR(result, 1148.6667, 0.1);

    result = thrusterCommander->get_pwm(1, -0.264);
    ASSERT_NEAR(result, 1419.4667, 0.1);

    result = thrusterCommander->get_pwm(1, -0.029);
    ASSERT_NEAR(result, 1464.1333, 0.1);

    result = thrusterCommander->get_pwm(1, -0.01);
    ASSERT_NEAR(result, 1466.6667, 0.1);

    result = thrusterCommander->get_pwm(1, 0.02);
    ASSERT_NEAR(result, 1533.6000, 0.1);

    result = thrusterCommander->get_pwm(1, 0.051);
    ASSERT_NEAR(result, 1536.4000, 0.1);

    result = thrusterCommander->get_pwm(1, 1.1);
    ASSERT_NEAR(result, 1660.8000, 0.1);

    result = thrusterCommander->get_pwm(1, 4.5);
    ASSERT_NEAR(result, 1891.5556, 0.1);

    result = thrusterCommander->get_pwm(1, 4.525);
    ASSERT_NEAR(result, 1898.0000, 0.1);

    delete thrusterCommander;
}

TEST(ThrusterCommanderTest, GetPwmOutOfBounds) {
    auto thrusterCommander = new Thruster_Commander();
    ASSERT_EXIT(thrusterCommander->get_pwm(1, -3.521), ::testing::ExitedWithCode(42),
                "Force too large of a negative number! No corresponding PWM found.");

    ASSERT_EXIT(thrusterCommander->get_pwm(1, -500), ::testing::ExitedWithCode(42),
                "Force too large of a negative number! No corresponding PWM found.");

    ASSERT_EXIT(thrusterCommander->get_pwm(1, 4.531), ::testing::ExitedWithCode(42),
                "Force too large of a positive number! No corresponding PWM found.");

    ASSERT_EXIT(thrusterCommander->get_pwm(1, 500), ::testing::ExitedWithCode(42),
                "Force too large of a positive number! No corresponding PWM found.");

    delete thrusterCommander;
}


TEST(ThrusterCommandTest, BasicSequenceTo) {
    auto thrusterCommander = new Thruster_Commander();
    six_axis target = { 5, 5, 5, 0, 0, 0 };
    command_sequence sequence = thrusterCommander->basic_sequence(target);

    // TODO: run pencil and paper calulations to determine if the sequence is correct
    // TODO: add asserts
    delete thrusterCommander;
}
TEST(ThrusterCommanderTest, Thrust_Compute_fz) {
    testing::internal::CaptureStdout();
    Thruster_Commander thruster_commander = Thruster_Commander();
    thruster_commander.print_info();
    thruster_commander.thrust_compute_fz(5);

    std::string output = testing::internal::GetCapturedStdout(); // TODO: check that output matches expected output
}
TEST(ThrusterCommanderTest, CompareConstructors) {
    Thruster_Commander commander = Thruster_Commander();
    Thruster_Commander commander2 = Thruster_Commander("/data/config1.yaml");
  //  std::cout << "HELLO!";
    commander.print_info();
    commander2.print_info();
    }