#include <cmath>

const double L = 0.25; // TODO: measure it

double bicycle_model(double theta) {
	return tan(theta) / L; // kappa
}

double bicycle_model_inv(double kappa) {
	return atan(kappa * L);
}

