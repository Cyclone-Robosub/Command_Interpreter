// Zenyn Ethridge, 2024
// zjethridge@ucdavis.edu

#include <iostream>
#include <cmath>
#include <string>
#include "Thruster_Commander.h"
#include "eigen-3.4.0/Eigen/Dense"
#include <fstream>
#include <sstream>




Thruster_Commander::Thruster_Commander()
{
	// TODO: Move all the hardcoded values in this constructor to a config file
	//       this will make unit testing simpler

	// Values come from Onshape 2024 Vehicle V10 11/12/24
	num_thrusters = 8;
	three_axis mass_center_inches = { 0.466, 0, 1.561 };
	mass_center = mass_center_inches * 0.0254; // convert to meters

	volume_center = mass_center; // volume center, currently, is a complete guess
	volume_center(0, 2) += 0.1; // add 0.1 meters to z coordinate to account for the volume center being slightly above the mass center

	// avg(max distance, min distance) of motor part 4 cylindrical surface to orgin
	// front left top, front right top, rear left top, rear right top, front left bottom, front right bottom, rear left bottom, rear right bottom
	// x, y, z coordinates here are how the appear on onshape. May need to be corrected to match surge, sway, heave

	thruster_positions = thruster_set_3D::Zero();
	thruster_positions.row(0) <<   .2535, -.2035, .042 ;
	thruster_positions.row(1) <<  .2535, .2035, .042; 
	thruster_positions.row(2) <<  -.2545, -.2035, .042;
	thruster_positions.row(3) <<  -.2545, .2035, .042;
	thruster_positions.row(4) <<  .167, -.1375, -.049;
	thruster_positions.row(5) <<  .167, .1375, -.049;
	thruster_positions.row(6) <<  -.1975, -.1165, -.049;
	thruster_positions.row(7) <<  -.1975, .1165, -.049;

	// torques will be calulated about the center of mass
	thruster_moment_arms = thruster_positions - mass_center.replicate(8, 1);

	// directionality recorded as the direction the front of thruster is facing
	// force direction will be reversed
	const double PI = 3.141592653589793;
	float sin45 = sin(45 * PI / 180);
	thruster_directions = thruster_set_3D::Zero();
	thruster_directions.row(0) << 0, 0, 1;
	thruster_directions.row(1) << 0, 0, 1;
	thruster_directions.row(2) << 0, 0, 1;
	thruster_directions.row(3) << 0, 0, 1;
	thruster_directions.row(4) << -sin45, -sin45, 0;
	thruster_directions.row(5) <<  -sin45, sin45, 0;
	thruster_directions.row(6) << -sin45, sin45, 0;
	thruster_directions.row(7) << -sin45, -sin45, 0;

	thruster_torques = thruster_set_3D::Zero();
	for (int i = 0; i < num_thrusters; i++)
	{
		thruster_torques.row(i) = thruster_moment_arms.row(i).cross(thruster_directions.row(i));
	}


    wrench_matrix_transposed = thruster_set_6D::Zero();
    for (int i = 0; i < num_thrusters; i++)
    {
        wrench_matrix_transposed.row(i).segment(0, 3) = thruster_directions.row(i);
        wrench_matrix_transposed.row(i).segment(3, 3) = thruster_torques.row(i);
    }
	wrench_matrix = wrench_matrix_transposed.transpose();


	float volume_inches = 449.157;            // volume of vehicle in inches^3, from onshape. This is likley less than the displacement volume and should be corrected
	volume = volume_inches * pow(0.0254, 3); // convert to meters^3
	mass = 5.51; // mass of vehicle in kg, from onshape
	gravity = -9.81;
	rho_water = 1025; // Density of water (kg/m^3)
	weight_magnitude = mass * gravity;
	buoyant_magnitude = -rho_water * gravity * volume;
	
	max_thruster_level = 0.9;

	// TODO: read these in from csv file based on voltage
	max_thruster_force = 4.5;
	min_thruster_force = -3.5;

	// values are relative to start values (zeros)
	position = six_axis::Zero();
	velocity = six_axis::Zero();
	acceleration = six_axis::Zero();

	// todo: replace math operation with it's direct result in config file
	six_axis c_inf = { 0.041, 0.05, 0.125, 0.005, 0.005, 0.005 };
	combined_drag_coefs = c_inf * rho_water;

	// int standard_voltages[6] = {}
	// std::string thruster_files = {}
	// Eigen::Matrix<float, 2, 6> min_max_voltages = {}
}
Thruster_Commander::~Thruster_Commander(){}

void Thruster_Commander::print_info()
{

	std::cout << "Mass Center: \n" << mass_center << std::endl;
	std::cout << "Volume Center: \n" << volume_center << std::endl;
	std::cout << "Thruster Positions: \n" << thruster_positions << std::endl;
	std::cout << "Thruster Moment Arms: \n" << thruster_moment_arms << std::endl;
	std::cout << "Thruster Directions: \n" << thruster_directions << std::endl;
	std::cout << "Thruster Torques: \n" << thruster_torques << std::endl;
	std::cout << "Mass: \n" << mass << std::endl;
	std::cout << "Volume: \n" << volume << std::endl;
	std::cout << "Combined drag coefs: \n" << combined_drag_coefs << std::endl;
	std::cout << "Wrench Matrix: \n" << wrench_matrix << std::endl;
}
void parseCsv(const std::string& filePath, double** numericData, int numRows, int numCols) {
    std::ifstream file(filePath, std::ios::in); // Replace with your CSV file name

    if (!file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return; //TODO: throw exception?
    }

    std::string line;
    std::string empty;
    int row = 0;
    getline(file, empty); // eat table headers at top of CSV
    while (getline(file, line) && row < numRows) {
        std::stringstream ss(line);
        std::string cell;
        int col = 0;
        while (getline(ss, cell, ',') && col < numCols) {
            (numericData)[row][col] = std::stod(cell);  // Convert string to double
            col++;
        }
        row++;
    }
    file.close();
}
double determinePwmValue(double force, double **numericData, double smallestDifference,
                         int closestRowIndex) {
    double x1;
    double y1;
    double x2;
    double y2;

    double pwmValue;

    if (smallestDifference < 0) { //Case when closest row/force is smaller than desired force
        if (numericData[(closestRowIndex)][0] == 0 && numericData[(closestRowIndex + 1)][0] == 0) {
            closestRowIndex++;
        }

        x1 = numericData[(closestRowIndex)][0];
        y1 = numericData[(closestRowIndex)][5];
        x2 = numericData[(closestRowIndex + 1)][0];
        y2 = numericData[(closestRowIndex + 1)][5];
        pwmValue = y1 + (force - x1) * ((y2 - y1) / (x2 - x1));
    }
    else if (smallestDifference > 0) { //Case when closest row/force value is larger than desired force
        if (numericData[(closestRowIndex)][0] == 0 && numericData[(closestRowIndex - 1)][0] == 0) {
            closestRowIndex--;
        }

        x1 = numericData[(closestRowIndex - 1)][0];
        y1 = numericData[(closestRowIndex - 1)][5];
        x2 = numericData[(closestRowIndex)][0];
        y2 = numericData[(closestRowIndex)][5];
        pwmValue = y1 + (force - x1) * ((y2 - y1) / (x2 - x1));
    }
    else {
        pwmValue = numericData[closestRowIndex][5];
    }
    return pwmValue;
}
double Thruster_Commander::get_pwm(int thruster_num, double force) {
    // TODO: the thruster number will be taken in to determine the voltage and thereby the CSV files to be used. Interpolation between CSV file value will be used to find in between voltages.

    int csvRows = 186;
    int csvColumns = 7;

    double** numericData = (double**)malloc(csvRows * sizeof(double*));
    for (int i = 0; i < csvRows; i++) {
        numericData[i] = (double*)malloc(csvColumns * sizeof(double));
    }

    parseCsv("14V_Correlation.csv", numericData, csvRows, csvColumns);

    if (force < numericData[0][0]) {
        std::cerr << "Force too large of a negative number! No corresponding PWM found." << std::endl;
        exit(42);
    }
    if (force > numericData[csvRows - 1][0]) {
        std::cerr << "Force too large of a positive number! No corresponding PWM found." << std::endl;
        exit(42);
    }

    double smallestDifference = INT_MAX; //Arbitrarily large number to ensure it runs the first time
    double difference;
    int closestRowIndex;

    for (int i = 0; i < csvRows; i++) {
        difference = numericData[i][0] - force;
        if (std::abs(difference) < std::abs(smallestDifference)) {
            smallestDifference = difference;
            closestRowIndex = i;
        }
    }

    return determinePwmValue(force, numericData, smallestDifference, closestRowIndex);
}
six_axis Thruster_Commander::weight_force(three_axis orientation)
{
	six_axis result = six_axis::Zero();
	result.segment(0, 3) = weight_magnitude * three_axis::UnitZ();
	std::cout << "Weight force: \n" << result << std::endl;

	result.segment(0,3) *= - Eigen::AngleAxisf(orientation(0),
		Eigen::Vector3f::UnitX()).toRotationMatrix();
	result.segment(0, 3) *= - Eigen::AngleAxisf(orientation(1),
		Eigen::Vector3f::UnitY()).toRotationMatrix();
	result.segment(0, 3) *= - Eigen::AngleAxisf(orientation(2),
		Eigen::Vector3f::UnitZ()).toRotationMatrix();

	std::cout << "Weight force: \n" << result << std::endl;
	return result;
}
six_axis Thruster_Commander::bouyant_force(three_axis orientation)
{
	six_axis result = six_axis::Zero();

	three_axis bouyant_force_linear = buoyant_magnitude * three_axis::UnitZ();
	bouyant_force_linear *= - Eigen::AngleAxisf(orientation(0),
		Eigen::Vector3f::UnitX()).toRotationMatrix();
	bouyant_force_linear *= - Eigen::AngleAxisf(orientation(1),
		Eigen::Vector3f::UnitY()).toRotationMatrix();
	bouyant_force_linear *= - Eigen::AngleAxisf(orientation(2),
		Eigen::Vector3f::UnitZ()).toRotationMatrix();

	three_axis bouyant_force_rotational =
		bouyant_force_linear.cross(volume_center - mass_center);

	result.segment(0, 3) = bouyant_force_linear;
	result.segment(3, 3) = bouyant_force_rotational;
	return result;
}
six_axis Thruster_Commander::gravitational_forces(three_axis orientation)
{
	six_axis result = six_axis::Zero();
	result += bouyant_force(orientation);
	result += weight_force(orientation);
	std::cout << "Gravitational forces: \n" << result << std::endl;
	return result;
}
three_axis Thruster_Commander::compute_forces(force_array forces)
{
	three_axis total_force = three_axis::Zero();
	for (int i = 0; i < num_thrusters; i++)
	{
		total_force += forces.forces[i] * thruster_directions.row(i);
	}
	return total_force;
}
three_axis Thruster_Commander::compute_torques(force_array forces)
{
	three_axis total_torque = three_axis::Zero();
	for (int i = 0; i < num_thrusters; i++)
	{
		total_torque += forces.forces[i] * thruster_torques.row(i);
	}
	return total_torque;
}
six_axis Thruster_Commander::predict_drag_forces(six_axis velocity)
{
	six_axis drag_force = six_axis::Zero();

	// explanation on notion:
	// https://www.notion.so/crsucd/
	// Rotational-drag-analyssi-1478a3eca2f0801d86f2e0c8fb675c0d
	// theses values are estimates and should be improveded experimentally
	float c_inf[6] = { 0.041, 0.05, 0.125, 0.005, 0.005, 0.005 };

	for (int i = 0; i < 6; i++)
	{
		drag_force(0, i) = rho_water * pow(velocity(i), 2) * c_inf[i];
	}

	std::cout << "Drag Force: " << drag_force << std::endl;
	return drag_force;
}
six_axis Thruster_Commander::net_env_forces(six_axis velocity, three_axis orientation)
{
	six_axis result = six_axis::Zero();
	result += predict_drag_forces(velocity);
	result += gravitational_forces(orientation);
	std::cout << "Net Environmental Forces: \n" << result << std::endl;
	return result;
}
six_axis Thruster_Commander::net_force_from_thrusters(thruster_set& thrusters)
{
	six_axis result = six_axis::Zero();
	return wrench_matrix * thrusters;
}
thruster_set Thruster_Commander::thrust_compute_fz(float z_force)
{

    // Force is euqally divided by the 4 vertical thrusters for this function

	float force_per_thruster = z_force / 4;
	
	thruster_set thrusters = thruster_set::Zero();
	thrusters(0) = force_per_thruster;
	thrusters(1) = force_per_thruster;
	thrusters(2) = force_per_thruster;
	thrusters(3) = force_per_thruster;
	
	return thrusters;
}
thruster_set Thruster_Commander::thrust_compute_fy(float y_force)
{

	// fx, fz and mz should be zero
	// my and mz should be small enough to keep the vehicle stable

	float sin_45 = sin(45 * 3.141592653589793 / 180);

    float force_per_thruster = y_force / (4*sin_45);
    thruster_set forces = thruster_set::Zero();
  
    forces(4) = -force_per_thruster;
    forces(5) = force_per_thruster;
    forces(6) = force_per_thruster;
    forces(7) = -force_per_thruster;

    return forces;
}
thruster_set Thruster_Commander::thrust_compute_fx(float x_force)
{
	// fy, fz and mz should be zero
	// mx and my should be small enough to keep the vehicle stable

	float sin_45 = sin(45 * 3.141592653589793 / 180);
    float force_per_thruster = x_force / (4*sin_45);
	
	thruster_set forces = thruster_set::Zero();
	forces(4) = -force_per_thruster;
	forces(5) = -force_per_thruster;
	forces(6) = -force_per_thruster;
	forces(7) = -force_per_thruster;

    return forces;

}
thruster_set Thruster_Commander::thrust_compute_fx_fy(float x_force, float y_force)
{
	// fz and mz should be zero
	// mx and my should be small enough to keep the vehicle stable
	thruster_set forces;
	return forces;
}
thruster_set Thruster_Commander::thrust_compute_mz(float z_torque)
{
	// fx, fy and fz should be zero
	// mx and my should be small enough to keep the vehicle stable
	thruster_set forces;
	return forces;
}
thruster_set Thruster_Commander::thrust_compute_fz_mz(float z_force, float z_torque)
{
	// fx and fy should be zero
	// mx and my should be small enough to keep the vehicle stable
	thruster_set forces;
	return forces;
}
thruster_set Thruster_Commander::thrust_compute_fy_mz(float y_force, float z_torque)
{
	// fx and fz should be zero
	// mx and my should be small enough to keep the vehicle stable
	thruster_set forces;
	return forces;
}
thruster_set Thruster_Commander::thrust_compute_fx_mz(float x_force, float z_torque)
{
	// fy and fz should be zero
	// mx and my should be small enough to keep the vehicle stable
	thruster_set forces;
	return forces;
}

thruster_set Thruster_Commander::thrust_compute_fx_fy_mz(float x_force, float y_force, float z_torque)
{
	// fz should be zero
	// mx and my should be small enough to keep the vehicle stable
	Eigen::VectorXf inputs(6);
	inputs << x_force, y_force, 0.0f, 0.0f, 0.0f, z_torque;

	// Coefficient matrix (8 x 6) calculated using a python script
	Eigen::MatrixXf coefficients(8, 6);
	coefficients <<
		-8.72533307e-02f,  1.08905877e-01f,  2.62141732e-01f, -1.22850123e+00f, -9.84251969e-01f,  4.77297359e-17f,
		-8.72533307e-02f, -1.08905877e-01f,  2.62141732e-01f,  1.22850123e+00f, -9.84251969e-01f, -9.09843115e-17f,
		 8.72533307e-02f,  1.08905877e-01f,  2.37858268e-01f, -1.22850123e+00f,  9.84251969e-01f, -9.54417065e-17f,
		 8.72533307e-02f, -1.08905877e-01f,  2.37858268e-01f,  1.22850123e+00f,  9.84251969e-01f,  1.38809793e-16f,
		-3.53553281e-01f, -3.72516112e-01f,  6.51614355e-18f,  2.10534883e-16f,  3.48410154e-18f, -1.14326218e+00f,
		-3.53553281e-01f,  3.72516112e-01f, -4.08974566e-18f, -6.72643856e-17f, -6.94539325e-18f,  1.14326218e+00f,
		-3.53553281e-01f,  3.34590450e-01f, -4.95362255e-18f, -1.01890400e-17f, -4.07608820e-18f, -1.14326218e+00f,
		-3.53553281e-01f, -3.34590450e-01f,  7.38002044e-18f,  2.75362748e-17f,  6.14796494e-19f,  1.14326218e+00f;

	// Compute the thruster outputs: T = coefficients * inputs
	Eigen::VectorXf T = coefficients * inputs;
	// Assign the computed thruster outputs to the forces vector
	thruster_set forces = T;
	return forces;
}
thruster_set Thruster_Commander::thrust_compute_fx_fy_fz(float x_force, float y_force)
{
	// mz should be zero
	// mx and my should be small enough to keep the vehicle stable
	thruster_set forces;
	return forces;
}
thruster_set Thruster_Commander::thrust_compute_fx_fy_fz_mz(float x_distance, float y_force, float z_force, float z_torque)
{
	// mx and my should be small enough to keep the vehicle stable
	thruster_set forces;
	return forces;
}
thruster_set Thruster_Commander::thrust_compute_fx_fy_fz_mx_my_mz(six_axis force_torques)
{

	// this is the most general case
	// all forces and torques are specified
	thruster_set forces;
	return forces;
}
thruster_set Thruster_Commander::thrust_compute_general(float x_force, float y_force, float z_force, float x_torque, float y_torque, float z_torque)
{
	Eigen::VectorXf forces(6);
	forces << x_force, y_force, z_force, x_torque, y_torque, z_torque;
	Eigen::Matrix<float, 8, 6> WInverse = wrench_matrix.completeOrthogonalDecomposition().pseudoInverse();
	thruster_set thruster_forces;
	thruster_forces = WInverse*forces;
	return thruster_forces;
	//Eigen::Matrix<float, 6, 8> W_Inverse= wrench_matrix.inverse();

	// Eigen::MatrixXd T = wrench_matrix.cast<float>().jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(forces);
	// //Eigen::VectorXf T = wrench_matrix.completeOrthogonalDecomposition().solve(forces);
	// thruster_set values;
	// values << T(0),T(1),T(2),T(3),T(4), T(5), T(6), T(7);
	// return values;
}
thruster_set Thruster_Commander::thrust_compute(six_axis force_torque, bool simple)
{
	// this is a general case force function
	// it will call other force functions depending on what forces and torques are specified
	// if simple=true, mz and my will be neglected
	thruster_set forces;
	return forces;
}
six_axis Thruster_Commander::velocity_at_time(thruster_set thruster_sets, float duration)
{
	six_axis result = six_axis::Zero();
	/*
	* For linear forces:
	C - ln(|C_d * V - sqrt(C_d * F_t)| / |C_d * V + sqrt(C_d * F_t)|) 
	    / (2 * sqrt(C_d * F_t) * m)
	where C is the constant of integration, 
	      C_d is combined drag coefficient (F_d / V^2)
		  F_t is force of thrusters, gravity, ect, m is mass
	* For rotational forces:
	same as above, but using I instead of m

	Does not account for changes in orientation!
	*/
	
	//todo
	return result;
}
float Thruster_Commander::accel_time_x(float v_i, float v) 
{
	float cd = combined_drag_coefs(0);
	float m = mass;

	bool forward = v > v_i;
	float force_per_thruster;

	if (forward) { force_per_thruster = min_thruster_force; }
	else { force_per_thruster = max_thruster_force; }

	thruster_set forces = thruster_set::Zero();
	forces(4) = force_per_thruster;
	forces(5) = force_per_thruster;
	forces(6) = force_per_thruster;
	forces(7) = force_per_thruster;

	float fx = net_force_from_thrusters(forces)(0);
	
	float t = physics::accel_time(v_i, v, cd, m, fx);

	return t;
}
float Thruster_Commander::top_speed_x(bool forward)
{
	// F = F_t - F_d = ma = 0  ->  F_t = F_d = C_d * v^2
	// v = sqrt(F_t / C_d)
	float cd = combined_drag_coefs(0);
	float force_per_thruster;

	if (forward) { force_per_thruster = min_thruster_force; }
	else { force_per_thruster = max_thruster_force; }

	thruster_set forces = thruster_set::Zero();
	forces(4) = force_per_thruster;
	forces(5) = force_per_thruster;
	forces(6) = force_per_thruster;
	forces(7) = force_per_thruster;

	float fx = net_force_from_thrusters(forces)(0);
	return (fx/abs(fx)) * sqrt(abs(fx) / cd);
}
void Thruster_Commander::basic_rotate_z(float angle_z, command_sequence& sequence) {}
void Thruster_Commander::basic_travel_z(float distance_z, command_sequence& sequence) {}
void Thruster_Commander::basic_travel_x(float distance_x, command_sequence& sequence) 
{
	bool forward = distance_x > 0;
	float speed_limiter = 0.9;

	float steady_speed = speed_limiter * top_speed_x(forward);
	
	float accel_time = accel_time_x(0, steady_speed);
	float accel_distance; // need a dist function
	// sequence.push_back(accelate_x(0, steady_speed));
	
	float deccel_time = accel_time_x(steady_speed, 0);
	float deccel_distance; // need a dist function

	float steady_speed_distance; // dist - accel dist - deccel dist
	float steady_speed_time; // ss_dist / ss_speed

}
command_sequence Thruster_Commander::basic_sequence(six_axis target_position)
{
	Eigen::Matrix<float, 1, 6> start_position;
	start_position << position;


	six_axis distance = target_position - start_position;
	float angle_z = std::atan(distance(1) / distance(0));
	command_sequence commands;
	basic_rotate_z(angle_z, commands);
	basic_travel_z(distance(2), commands);
	basic_travel_x(distance(0), commands);
	return commands;
}