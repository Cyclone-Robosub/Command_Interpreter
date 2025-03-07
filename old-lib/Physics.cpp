   #include "Physics.h"

double physics::accel_time(double vi, double vt, double cd, double m, double f)
{
	// Drag force is normally calulated as F_D = 0.5 * rho * v^2 * C_D * A
	// for simplicity, cd here refers to the combined drag coefficient, 
	// in which the drag force is equal to cd * v^2. cd = 0.5 * rho * C_D * A

	if (cd <= 0) {
		throw std::invalid_argument("Drag coefficient must be positive");
	}
	if (m <= 0) {
		throw std::invalid_argument("Mass must be positive");
	}

	if (f == 0) {
		double A = m / (cd * vt);
		double B = m / (cd * vi);
		return A - B;
	}

	// set f to be positive
	double direction = f / std::abs(f);
	f *= direction;
	vi *= direction;
	vt *= direction;

	if (vi == 0) {

		double A = m / (2 * std::sqrt(cd * f));
		double B = std::abs(cd * vt - std::sqrt(cd * f));
		double C = std::abs(cd * vt + std::sqrt(cd * f));

		return -A * std::log(B / C);
	}
	if (vi < 0 && vt <= 0) {

		double A = m / (2 * std::sqrt(cd * f));
		double B = cd * vi / std::sqrt(cd * f);
		double C = cd * vt / std::sqrt(cd * f);

		return A * std::atan(B) - A * std::atan(C);
	}
	if (vi < 0 && vt > 0) {
		double t1 = accel_time(vi, 0, cd, m, f);
		double t2 = accel_time(0, vt, cd, m, f);
		return t2 + t1;
	}
	if (vi > 0 && vt >= 0) {
		double t1 = accel_time(0, vi, cd, m, f);
		double t2 = accel_time(0, vt, cd, m, f);

		return t2 - t1;
	}
	if (vi > 0 && vt < 0) {
		double t1 = accel_time(vi, 0, cd, m, f);
		double t2 = accel_time(0, vt, cd, m, f);
		return t2 + t1;
	}
	std::cout << "vi: " << vi << " vt: " << vt << "cd: " << cd << "f: " << f << "m: " << m << std::endl;
	throw std::invalid_argument("Invalid input");
}
double physics::accel_dist(double vi, double vt, double cd, double m, double f, double t)
{
	if (cd <= 0) {
		throw std::invalid_argument("Drag coefficient must be positive");
	}
	if (m <= 0) {
		throw std::invalid_argument("Mass must be positive");
	}

	if (f == 0) {

	}

	// set f to be positive
	double direction = f / std::abs(f);
	f *= direction;


	if (vi == 0 && cd * vt > std::sqrt(cd * f)) {

		double e = 2.7182818284;
		double A = -m / (2 * std::sqrt(cd * f));
		double B = 1 + exp(t / A);
		double C = std::sqrt(cd * f) * B;
		double D = cd * exp(t / A) - cd;

		return C / D;


	}

	return 0;
}