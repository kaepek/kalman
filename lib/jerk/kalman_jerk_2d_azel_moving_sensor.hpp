/*
 * Jerk filter for a target moving in space, measured as azimuth and elevation from a sensor with known kinematic state.
 *
 * The target follows the per axis jerk model of [Ref1]. The state is expressed in modified spherical coordinates
 * of jerk order: the direction state of KalmanJerk2DAzEl, [d lambda, d2 lambda, d3 lambda] with lambda = ln r,
 * and 1/r, see model/derivation.md. The error is [8 direction components, d lambda, d2 lambda, d3 lambda, 1/r].
 *
 * Developed for the Kaepek Project
 */

#ifndef KAEPEK_KALMAN_JERK_2D_AZEL_MOVING_SENSOR_H
#define KAEPEK_KALMAN_JERK_2D_AZEL_MOVING_SENSOR_H

#include "kalman_jerk_policies.hpp"
#include "kalman_jerk_linalg.hpp"
#include "kalman_jerk_autodiff.hpp"
#include "kalman_jerk_sphere.hpp"
#include "kalman_jerk_direction_init.hpp"
#include "kalman_jerk_map_propagation.hpp"
#include "generated/msc.hpp"

namespace kaepek
{
    /**
     * @brief Prediction map of the modified spherical state for one sampling interval.
     */
    struct MscPredictionMap
    {
        const double *x;
        const double *b;
        const double *lam;
        double q;
        const double (*F)[4];
        const double *sensor_term;
        const double *x_pred;
        const double *b_pred;
        const double *lam_pred;
        double q_pred;

        template <typename S>
        void propagate(const S *zeta, S *x_out, S *lam_out, S &q_out) const
        {
            S xs[12], bs[6], lams[3], xi_in[12], xi[12], y[16];
            direction_retract(x, b, zeta, xs, bs);
            for (int i = 0; i < 3; i++)
                lams[i] = S(lam[i]) + zeta[8 + i];
            S qs = S(q) + zeta[11];
            msc_to_xi(xs, lams, xi_in);
            for (int axis = 0; axis < 3; axis++)
                for (int r = 0; r < 4; r++)
                {
                    S s = qs * (sensor_term[4 * axis + r] + zeta[12 + 4 * axis + r]);
                    for (int m = 0; m < 4; m++)
                        s += F[r][m] * xi_in[4 * axis + m];
                    xi[4 * axis + r] = s;
                }
            msc_from_xi(xi, y);
            for (int i = 0; i < 12; i++)
                x_out[i] = y[i];
            for (int i = 0; i < 3; i++)
                lam_out[i] = y[12 + i];
            q_out = qs / y[15];
        }

        template <typename S>
        void operator()(const S *zeta, S *out) const
        {
            S xn[12], lamn[3], qn;
            propagate(zeta, xn, lamn, qn);
            direction_inverse_retract(x_pred, b_pred, xn, out);
            for (int i = 0; i < 3; i++)
                out[8 + i] = lamn[i] - lam_pred[i];
            out[11] = qn - q_pred;
        }

        void reexpress(const double *mean, const double *xi, double *out) const
        {
            double xs[12], bs[6], xm[12], bm[6];
            direction_retract(x_pred, b_pred, xi, xs, bs);
            direction_retract(x_pred, b_pred, mean, xm, bm);
            direction_inverse_retract(xm, bm, xs, out);
            for (int i = 8; i < 12; i++)
                out[i] = xi[i] - mean[i];
        }
    };

    template <class Form = JerkSmallAlphaT, class Order = FirstOrder, class Diag = NoDiagnostics>
    class KalmanJerk2DAzElMovingSensor : public DiagnosticsStore<Diag, 2>
    {
    private:
        double alpha;
        double q_scale;
        double direction_variance;
        double jerk_error;
        double acceleration_error;
        double range_min;
        double range_max;
        double log_range_variances[3];
        bool time_is_relative;
        int current_idx;

        double time;
        double dts[2];
        double measurements[3][3];
        double measurement_covariances[3][2][2];
        bool measurement_isotropic[3];
        double sensor_state[12];

        double x[12];
        double b[6];
        double lam[3];
        double q;
        double P[12][12];
        double kalman_vector[12];

        void initialise()
        {
            double q_min = 1.0 / range_max;
            double q_max = 1.0 / range_min;
            double angular_jerk = jerk_error * q_max;
            double angular_acceleration = acceleration_error * q_max;
            double P8[8][8];
            direction_initialise<Form>(measurements, measurement_covariances, measurement_isotropic, dts, alpha,
                                       2.0 * alpha * angular_jerk * angular_jerk, angular_acceleration * angular_acceleration,
                                       angular_jerk * angular_jerk, x, b, P8);
            mat_zero(P);
            for (int i = 0; i < 8; i++)
                for (int k = 0; k < 8; k++)
                    P[i][k] = P8[i][k];
            for (int i = 0; i < 3; i++)
            {
                lam[i] = 0.0;
                P[8 + i][8 + i] = log_range_variances[i];
            }
            q = 0.5 * (q_min + q_max);
            P[11][11] = (q_max - q_min) * (q_max - q_min) / 12.0;
        }

        void predict(double dt, const double (&sensor)[12])
        {
            double F[4][4], Q[4][4];
            Form::fill(dt, alpha, F, Q);
            double sensor_term[12];
            for (int axis = 0; axis < 3; axis++)
                for (int r = 0; r < 4; r++)
                {
                    double s = -sensor[4 * axis + r];
                    for (int m = 0; m < 4; m++)
                        s += F[r][m] * sensor_state[4 * axis + m];
                    sensor_term[4 * axis + r] = s;
                }
            double Qv[12][12];
            mat_zero(Qv);
            for (int axis = 0; axis < 3; axis++)
                for (int r = 0; r < 4; r++)
                    for (int s = 0; s < 4; s++)
                        Qv[4 * axis + r][4 * axis + s] = q_scale * Q[r][s];
            MscPredictionMap fn;
            fn.x = x;
            fn.b = b;
            fn.lam = lam;
            fn.q = q;
            fn.F = F;
            fn.sensor_term = sensor_term;
            double zeta0[24], x_pred[12], b_pred[6], lam_pred[3], q_pred;
            for (int i = 0; i < 24; i++)
                zeta0[i] = 0.0;
            fn.propagate(zeta0, x_pred, lam_pred, q_pred);
            transport_basis(x, x_pred, b, b_pred);
            fn.x_pred = x_pred;
            fn.b_pred = b_pred;
            fn.lam_pred = lam_pred;
            fn.q_pred = q_pred;
            double mean[12], P_out[12][12];
            map_propagate<12, 12>(fn, P, Qv, mean, P_out, Order());
            direction_retract(x_pred, b_pred, mean, x, b);
            direction_normalise(x, b);
            for (int i = 0; i < 3; i++)
                lam[i] = lam_pred[i] + mean[8 + i];
            q = q_pred + mean[11];
            mat_copy(P_out, P);
        }

        void update(const double *m, const double (&R)[2][2], bool isotropic)
        {
            double y[2], R_b[2][2];
            direction_innovation(x, b, m, R, isotropic, y, R_b);
            const int idx[2] = {0, 4};
            double S[2][2], S_inv[2][2], det_S, correction[12];
            if (!kalman_update(P, idx, y, R_b, S, S_inv, det_S, correction))
                return;
            double xn[12], bn[6];
            direction_retract(x, b, correction, xn, bn);
            for (int i = 0; i < 12; i++)
                x[i] = xn[i];
            for (int i = 0; i < 6; i++)
                b[i] = bn[i];
            direction_normalise(x, b);
            for (int i = 0; i < 3; i++)
                lam[i] += correction[8 + i];
            q += correction[11];
            this->store_diagnostics(y, S, S_inv, det_S);
        }

        void step_measurement(double time_or_dt, double azimuth, double elevation, const double (&sensor)[12], const double (&R)[2][2], bool isotropic)
        {
            double m[3];
            unit_from_azel(azimuth, elevation, m);
            if (current_idx == -1)
                time = time_or_dt;
            else
            {
                double dt = time_is_relative ? time_or_dt : time_or_dt - time;
                time = time_is_relative ? time + time_or_dt : time_or_dt;
                if (current_idx < 2)
                    dts[current_idx] = dt;
                else
                {
                    predict(dt, sensor);
                    update(m, R, isotropic);
                }
            }
            if (current_idx < 2)
            {
                int n = current_idx + 1;
                for (int i = 0; i < 3; i++)
                    measurements[n][i] = m[i];
                for (int i = 0; i < 2; i++)
                    for (int k = 0; k < 2; k++)
                        measurement_covariances[n][i][k] = R[i][k];
                measurement_isotropic[n] = isotropic;
                if (n == 2)
                    initialise();
                current_idx++;
            }
            for (int i = 0; i < 12; i++)
                sensor_state[i] = sensor[i];
        }

    public:
        /**
         * KalmanJerk2DAzElMovingSensor default constructor.
         */
        KalmanJerk2DAzElMovingSensor() : alpha(0.0), q_scale(0.0), direction_variance(0.0), jerk_error(0.0), acceleration_error(0.0), range_min(1.0), range_max(1.0), time_is_relative(false), current_idx(-1), time(0.0), q(0.0)
        {
        }

        /**
         * KalmanJerk2DAzElMovingSensor constructor with parameters.
         *
         * @param alpha alpha used to calculate q factor with q = 2 * alpha * jerk_error^2
         * @param direction_error The standard deviation of the angular error of a measured direction in radians, isotropic on the sphere
         * @param jerk_error The standard deviation of the jerk of each Cartesian coordinate of the target
         * @param time_is_relative Indicates whether time values parsed to the step function are absolute temporal values or temporal differences between consecutive steps.
         * @param range_min Lower bound of the initial range
         * @param range_max Upper bound of the initial range
         * @param log_range_rate_error Initial standard deviation of d lambda, lambda = ln r
         * @param log_range_acceleration_error Initial standard deviation of d2 lambda
         * @param log_range_jerk_error Initial standard deviation of d3 lambda
         * @param acceleration_error The standard deviation of the acceleration of each Cartesian coordinate, used by the JerkExact initialisation (28)
         */
        KalmanJerk2DAzElMovingSensor(double alpha, double direction_error, double jerk_error, bool time_is_relative, double range_min, double range_max,
                                     double log_range_rate_error, double log_range_acceleration_error, double log_range_jerk_error, double acceleration_error = 0.0)
            : alpha(alpha), q_scale(2.0 * alpha * jerk_error * jerk_error), direction_variance(direction_error * direction_error), jerk_error(jerk_error),
              acceleration_error(acceleration_error), range_min(range_min), range_max(range_max), time_is_relative(time_is_relative), current_idx(-1), time(0.0), q(0.0)
        {
            log_range_variances[0] = log_range_rate_error * log_range_rate_error;
            log_range_variances[1] = log_range_acceleration_error * log_range_acceleration_error;
            log_range_variances[2] = log_range_jerk_error * log_range_jerk_error;
            for (int i = 0; i < 12; i++)
            {
                x[i] = 0.0;
                sensor_state[i] = 0.0;
                kalman_vector[i] = 0.0;
            }
            for (int i = 0; i < 6; i++)
                b[i] = 0.0;
            for (int i = 0; i < 3; i++)
                lam[i] = 0.0;
            mat_zero(P);
        }

        /**
         * Step with the isotropic measurement covariance direction_error^2 I.
         * @param time_or_dt absolute time or relative time since last step (depending on how time_is_relative is set)
         * @param azimuth measured azimuth in radians
         * @param elevation measured elevation in radians
         * @param sensor sensor state per axis [x, vx, ax, jx, y, vy, ay, jy, z, vz, az, jz] at the measurement time
         */
        void step(double time_or_dt, double azimuth, double elevation, const double (&sensor)[12])
        {
            double R[2][2] = {{direction_variance, 0.0}, {0.0, direction_variance}};
            step_measurement(time_or_dt, azimuth, elevation, sensor, R, true);
        }

        /**
         * Step with a measurement covariance in the tangent plane of the measured direction, in the frame of
         * increasing azimuth and increasing elevation.
         * @param time_or_dt absolute time or relative time since last step (depending on how time_is_relative is set)
         * @param azimuth measured azimuth in radians
         * @param elevation measured elevation in radians
         * @param sensor sensor state per axis [x, vx, ax, jx, y, vy, ay, jy, z, vz, az, jz] at the measurement time
         * @param R measurement covariance in radians squared
         */
        void step(double time_or_dt, double azimuth, double elevation, const double (&sensor)[12], const double (&R)[2][2])
        {
            step_measurement(time_or_dt, azimuth, elevation, sensor, R, false);
        }

        /**
         * @brief Get Kalman state estimate [azimuth, w_a, a_a, j_a, elevation, w_e, a_e, j_e, d lambda, d2 lambda, d3 lambda, 1/r]
         *  with tangent components along increasing azimuth and increasing elevation, given in the internal basis at the zenith and nadir.
         */
        double (&get_kalman_vector())[12]
        {
            double az, el, e_a[3], e_e[3];
            azel_from_unit(x, az, el);
            if (!azel_frame(x, e_a, e_e))
                for (int i = 0; i < 3; i++)
                {
                    e_a[i] = b[i];
                    e_e[i] = b[3 + i];
                }
            kalman_vector[0] = az;
            kalman_vector[4] = el;
            for (int k = 1; k < 4; k++)
            {
                kalman_vector[k] = dot3(x + 3 * k, e_a);
                kalman_vector[4 + k] = dot3(x + 3 * k, e_e);
            }
            for (int i = 0; i < 3; i++)
                kalman_vector[8 + i] = lam[i];
            kalman_vector[11] = q;
            return kalman_vector;
        }

        /**
         * @brief Get the state [u, w, a, j]: unit direction, angular velocity, angular acceleration and angular jerk vectors.
         */
        double (&get_state_vector())[12] { return x; }

        /**
         * @brief Get the tangent basis [b1, b2] of the internal covariance.
         */
        double (&get_basis())[6] { return b; }

        /**
         * @brief Get the covariance matrix in the internal tangent basis.
         */
        double (&get_covariance_matrix())[12][12] { return P; }

        /**
         * @brief Get the range estimate 1 / (1/r).
         */
        double get_range() const { return 1.0 / q; }
    };
}

#endif
