/*
 * Jerk filter for a point in a plane measured as range and angle, (44) and (45) of [Ref1] at zero elevation.
 * The measurement and its covariance are converted to Cartesian coordinates and passed to KalmanJerk2D.
 *
 * Developed for the Kaepek Project
 */

#ifndef KAEPEK_KALMAN_JERK_2D_POLAR_H
#define KAEPEK_KALMAN_JERK_2D_POLAR_H

#include "kalman_jerk_cartesian.hpp"
#include "generated/polar_spherical.hpp"

namespace kaepek
{
    template <class Form = JerkSmallAlphaT, class Diag = NoDiagnostics>
    class KalmanJerk2DPolar
    {
    private:
        KalmanJerk2D<Form, Diag> cartesian;
        double range_variance;
        double angle_variance;

    public:
        KalmanJerk2DPolar() : range_variance(0.0), angle_variance(0.0) {}

        /**
         * KalmanJerk2DPolar constructor with parameters.
         *
         * @param alpha alpha used to calculate q factor with q = 2 * alpha * jerk_error^2
         * @param range_error The standard deviation of a range measurement
         * @param angle_error The standard deviation of an angle measurement in radians
         * @param jerk_error The standard deviation of the jerk of each Cartesian coordinate
         * @param time_is_relative Indicates whether time values parsed to the step function are absolute temporal values or temporal differences between consecutive steps.
         * @param acceleration_error The standard deviation of the acceleration of each Cartesian coordinate, used by the JerkExact initialisation (28)
         */
        KalmanJerk2DPolar(double alpha, double range_error, double angle_error, double jerk_error, bool time_is_relative, double acceleration_error = 0.0)
            : cartesian(alpha, 0.0, jerk_error, time_is_relative, acceleration_error),
              range_variance(range_error * range_error), angle_variance(angle_error * angle_error)
        {
        }

        /**
         * @param time_or_dt absolute time or relative time since last step (depending on how time_is_relative is set)
         * @param range measured range
         * @param angle measured angle in radians
         */
        void step(double time_or_dt, double range, double angle)
        {
            double M[2];
            double R[2][2];
            polar_position(range, angle, M);
            polar_covariance(range, angle, range_variance, angle_variance, R);
            cartesian.step(time_or_dt, M, R);
        }

        /**
         * @brief Get Kalman state estimate [x, vx, ax, jx, y, vy, ay, jy].
         */
        double (&get_kalman_vector())[8] { return cartesian.get_kalman_vector(); }

        /**
         * @brief Get the covariance matrix P.
         */
        double (&get_covariance_matrix())[8][8] { return cartesian.get_covariance_matrix(); }

        /**
         * @brief Get the internal Cartesian filter, which holds the diagnostics when enabled.
         */
        KalmanJerk2D<Form, Diag> &get_cartesian() { return cartesian; }
    };
}

#endif
