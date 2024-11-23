#include <iostream>
#include <cmath>
#include <string>
#include "Thruster_Commander.h"
#include "eigen-3.4.0/Eigen/Dense"
#include <fstream>
#include <iostream>
#include <sstream>

Thruster_Commander::Thruster_Commander()
{
	// Values come from Onshape 2024 Vehicle V10 11/12/24
	num_thrusters = 8;
	Eigen::Matrix<float, 1, 3> mass_center_inches = { 0, 0.466, 1.561 };
	mass_center = mass_center_inches * 0.0254; // convert to meters
	
	volume_center = mass_center; // volume center, currently, is a complete guess
	volume_center(0, 2) += 0.1; // add 0.1 meters to z coordinate to account for the volume center being slightly above the mass center

	// avg(max distance, min distance) of motor part 4 cylindrical surface to orgin
	// front left top, front right top, rear left top, rear right top, front left bottom, front right bottom, rear left bottom, rear right bottom
	// x, y, z coordinates here are how the appear on onshape. May need to be corrected to match surge, sway, heave
	thruster_positions = Eigen::Matrix<float, 8, 3>::Zero();
	thruster_positions.row(0) <<  -.2035, .2535, .042 ;
	thruster_positions.row(1) << .2035, .2535, -.042;
	thruster_positions.row(2) << -.2035, -.2545, .042;
	thruster_positions.row(3) << .2035, -.2545, .042;
	thruster_positions.row(4) << -.1375, .167, -.049;
	thruster_positions.row(5) << .1375, .167, -.049;
	thruster_positions.row(6) << -.1165, -.1975, -.049;
	thruster_positions.row(7) << .1165, -.1975, -.049;
	
	// torques will be calulated about the center of mass
	thruster_moment_arms = thruster_positions - mass_center.replicate(8, 1);
	

	// directionality recorded as the direction the front of thruster is facing
	// force direction will be reversed
	const double PI = 3.141592653589793;
	float sin45 = sin(45 * PI / 180);
	thruster_directions = Eigen::Matrix<float, 8, 3>::Zero();
	thruster_directions.row(0) << 0, 0, 1;
	thruster_directions.row(1) << 0, 0, 1;
	thruster_directions.row(2) << 0, 0, 1;
	thruster_directions.row(3) << 0, 0, 1;
	thruster_directions.row(4) << -sin45, -sin45, 0;
	thruster_directions.row(5) <<  sin45, -sin45, 0;
	thruster_directions.row(6) << sin45, -sin45, 0;
	thruster_directions.row(7) << -sin45, -sin45, 0;


	thruster_torques = Eigen::Matrix<float, 8, 3>::Zero();
	for (int i = 0; i < num_thrusters; i++)
	{
		thruster_torques.row(i) = thruster_moment_arms.row(i).cross(thruster_directions.row(i));
	}

	float volume_inches = 449.157;            // volume of vehicle in inches^3, from onshape. This is likley less than the displacement volume and should be corrected
	volume = volume_inches * pow(0.0254, 3); // convert to meters^3	
	mass = 5.51; // mass of vehicle in kg, from onshape

}

Thruster_Commander::~Thruster_Commander()
{
}

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
}

int get_pwm(int thruster_num, float force){
	int PWM_value;
	std::ifstream dataset("data/14V_PWM_Correlation.csv"); // Replace with your CSV file name

		//Number of Rows and Columns of the CSV file
		int csvRows = 201;
		int csvColumns = 7;

		double numericData[csvRows][csvColumns];  // Numeric array for storing the converted values

		using namespace std;

		//Checks to see if file is open
		if (!file.is_open()) {
			cerr << "Error opening file!" << endl;
			return 1;
		}


		//Parses through csv lines and reads the numbers prior to commas as a string in a 2D array
		string line;
		int row = 1;
		while (getline(file, line) && row < csvRows) {
			stringstream ss(line);
			string cell;
			int col = 0;
			while (getline(ss, cell, ',') && col < csvColumns) {
				numericData[row][col] = stod(cell);  // Convert string to double
				col++;
			}
			row++;
		}

file.close();

		double smallestDifference = 100000; //Arbitrarily large number to ensure it runs the first time
		double difference;
		int closestRowValue;

		for (int i = 0; i < (csvRows - 1); i++){
			difference = force - numericData[i][6];
			if (std::abs(difference) < smallestDifference){
				smallestDifference = difference;
				closestRowValue = i;
			}
		}

		//Variables used for linear interpolation calculation
		double y1;
		double y2;
		double x1;
		double x2;

		if (difference < 0){ //Case when closest row/force is larger than inputted force
			y1 = numericData[(closestRowValue - 1)][1];
			y2 = numericData[(closestRowValue)][1];
			x1 = numericData[(closestRowValue - 1)][7];
			x2 = numericData[(closestRowValue)][7];

		} else { //Case when closest row/force value is smaller than inputted force
			y1 = numericData[(closestRowValue)][1];
			y2 = numericData[(closestRowValue + 1)][1];
			x1 = numericData[(closestRowValue)][7];
			x2 = numericData[(closestRowValue + 1)][7];
		}

		PWM_value = y1 + (force - x1) * ((y2 - y1)/(x2 - x1));
	
		return PWM_value;
	}

}
 pwm_array Thruster_Commander::simple_vertical(float force){

    // If we reach here, then element was not present
    
	force =/ 4;
	int resultingPWMfromForce[4];
	for(auto f: resultingPWMfromForce)
	{
		
	}
	// The force is going to tell us how much the PWM is going to be for each pin. The pins are stored in pwmsignals Array. 
	//Fet Force -> deduce PWM from force for each pin -> tell each pin by iterating over the array what th
	for(int i = 0; i < pwm_array.length(); ++i)
	{
		pwm_array[i]= resultingPWMfromForce[i];
	}
	
}
 pwm_array::simple_forward(float force){
	
}
void pwm_array::simple_sideways(float force){
	
}
void pwm_array::simple_horizontal(float x_force, float y_force)
{

}