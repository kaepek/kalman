/*
 * Jerk filter for a point in space measured as range, azimuth and elevation, (44) and (45) of [Ref1].
 * The measurement and its covariance are converted to Cartesian coordinates and passed to KalmanJerk3D.
 *
 * Developed for the Kaepek Project
 */

#ifndef KAEPEK_KALMAN_JERK_3D_SPHERICAL_H
#define KAEPEK_KALMAN_JERK_3D_SPHERICAL_H

#include "kalman_jerk_cartesian.hpp"
#include "generated/polar_spherical.hpp"

namespace kaepek
{
    template <class Form = JerkSmallAlphaT, class Diag = NoDiagnostics>
    class KalmanJerk3DSpherical
    {
    private:
        KalmanJerk3D<Form, Diag> cartesian;
        double range_variance;
        double azimuth_variance;
        double elevation_variance;

    public:
        KalmanJerk3DSpherical() : range_variance(0.0), azimuth_variance(0.0), elevation_variance(0.0) {}

        /**
         * KalmanJerk3DSpherical constructor with parameters.
         *
         * @param alpha alpha used to calculate q factor with q = 2 * alpha * jerk_error^2
         * @param range_error The standard deviation of a range measurement
         * @param azimuth_error The standard deviation of an azimuth measurement in radians
         * @param elevation_error The standard deviation of an elevation measurement in radians
         * @param jerk_error The standard deviation of the jerk of each Cartesian coordinate
         * @param time_is_relative Indicates whether time values parsed to the step function are absolute temporal values or temporal differences between consecutive steps.
         * @param acceleration_error The standard deviation of the acceleration of each Cartesian coordinate, used by the JerkExact initialisation (28)
         */
        KalmanJerk3DSpherical(double alpha, double range_error, double azimuth_error, double elevation_error, double jerk_error, bool time_is_relative, double acceleration_error = 0.0)
            : cartesian(alpha, 0.0, jerk_error, time_is_relative, acceleration_error),
              range_variance(range_error * range_error), azimuth_variance(azimuth_error * azimuth_error), elevation_variance(elevation_error * elevation_error)
        {
        }

        /**
         * @param time_or_dt absolute time or relative time since last step (depending on how time_is_relative is set)
         * @param range measured range
         * @param azimuth measured azimuth in radians
         * @param elevation measured elevation in radians
         */
        void step(double time_or_dt, double range, double azimuth, double elevation)
        {
            double M[3];
            double R[3][3];
            spherical_position(range, azimuth, elevation, M);
            spherical_covariance(range, azimuth, elevation, range_variance, azimuth_variance, elevation_variance, R);
            cartesian.step(time_or_dt, M, R);
        }

        /**
         * @brief Get Kalman state estimate [x, vx, ax, jx, y, vy, ay, jy, z, vz, az, jz].
         */
        double (&get_kalman_vector())[12] { return cartesian.get_kalman_vector(); }

        /**
         * @brief Get the covariance matrix P.
         */
        double (&get_covariance_matrix())[12][12] { return cartesian.get_covariance_matrix(); }

        /**
         * @brief Get the internal Cartesian filter, which holds the diagnostics when enabled.
         */
        KalmanJerk3D<Form, Diag> &get_cartesian() { return cartesian; }
    };
}

#endif
