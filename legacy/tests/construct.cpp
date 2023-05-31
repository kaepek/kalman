#include <iostream>
#include "../../lib/jerk/kalman_jerk.cpp"


int main() {
    double alpha = 1.0;
    double x_res_error = 4.0;
    double x_jerk_error = 1.0;
    kaepek::KalmanJerk1D kalman_normal = kaepek::KalmanJerk1D(alpha,x_res_error, x_jerk_error, false);
    double x_mod = 16384.0;
    kaepek::KalmanJerk1D kalman_modular = kaepek::KalmanJerk1D(alpha,x_res_error, x_jerk_error, x_mod, false);
    std::cout << "done";
}