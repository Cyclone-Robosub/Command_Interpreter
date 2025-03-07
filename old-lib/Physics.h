#pragma once
#include <cmath>
#include <stdexcept>
#include <iostream>

namespace physics {
	double accel_time(double vi, double vt, double cd, double m, double f);
	double accel_dist(double vi, double vt, double cd, double m, double f, double t);
}
