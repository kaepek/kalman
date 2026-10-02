/*
 * Jerk filter for a target moving in a plane, measured as bearings from a sensor with known kinematic state.
 *
 * The target follows the per axis jerk model of [Ref1]. The state is expressed in modified polar coordinates
 * of jerk order: Y = [beta, d beta, d2 beta, d3 beta, d lambda, d2 lambda, d3 lambda, 1/r], lambda = ln r,
 * see model/derivation.md.
 *
 * Developed for the Kaepek Project
 */

#ifndef KAEPEK_KALMAN_JERK_1D_BEARING_MOVING_SENSOR_H
#define KAEPEK_KALMAN_JERK_1D_BEARING_MOVING_SENSOR_H

#include <math.h>
#include "kalman_jerk_policies.hpp"
#include "kalman_jerk_linalg.hpp"
#include "kalman_jerk_init.hpp"
#include "kalman_jerk_autodiff.hpp"
#include "kalman_jerk_map_propagation.hpp"
#include "generated/mpc.hpp"

namespace kaepek
{
    /**
     * @brief Angle difference a - b mapped into (-pi, pi].
     */
    template <typename S>
    inline S wrapped_angle_difference(const S &a, double b)
    {
        const double two_pi = 6.283185307179586;
        double raw = value_of(a) - b;
        double turns = floor(raw / two_pi + 0.5);
        if (raw - turns * two_pi <= -0.5 * two_pi)
            turns -= 1.0;
        return a - b - turns * two_pi;
    }

    /**
     * @brief Prediction map of the modified polar state for one sampling interval.
     */
    struct MpcPredictionMap
    {
        const double *Y;
        const double *Y_pred;
        const double (*F)[4];
        const double *sensor_term;

        template <typename S>
        void propagate(const S *zeta, S *out) const
        {
            S Ys[7], xi_in[8], xi[8], y[8];
            for (int i = 0; i < 7; i++)
                Ys[i] = S(Y[i]) + zeta[i];
            S q = S(Y[7]) + zeta[7];
            mpc_to_xi(Ys, xi_in);
            for (int axis = 0; axis < 2; axis++)
                for (int r = 0; r < 4; r++)
                {
                    S s = q * (sensor_term[4 * axis + r] + zeta[8 + 4 * axis + r]);
                    for (int m = 0; m < 4; m++)
                        s += F[r][m] * xi_in[4 * axis + m];
                    xi[4 * axis + r] = s;
                }
            mpc_from_xi(xi, y);
            for (int i = 0; i < 7; i++)
                out[i] = y[i];
            out[7] = q / y[7];
        }

        template <typename S>
        void operator()(const S *zeta, S *out) const
        {
            S raw[8];
            propagate(zeta, raw);
            out[0] = wrapped_angle_difference(raw[0], Y_pred[0]);
            for (int i = 1; i < 8; i++)
                out[i] = raw[i] - Y_pred[i];
        }

        void reexpress(const double *mean, const double *xi, double *out) const
        {
            for (int i = 0; i < 8; i++)
                out[i] = xi[i] - mean[i];
        }
    };

    template <class Form = JerkSmallAlphaT, class Order = FirstOrder, class Diag = NoDiagnostics>
    class KalmanJerk1DBearingMovingSensor : public DiagnosticsStore<Diag, 1>
    {
    private:
        double alpha;
        double q_scale;
        double bearing_variance;
        double jerk_error;
        double acceleration_error;
        double range_min;
        double range_max;
        double log_range_variances[3];
        bool time_is_relative;
        int current_idx;

        double time;
        double dts[2];
        double bearings[3];
        double sensor_state[8];

        double Y[8];
        double P[8][8];

        void initialise()
        {
            double m2 = bearings[0] + value_of(wrapped_angle_difference(bearings[1], bearings[0]));
            double m3 = m2 + value_of(wrapped_angle_difference(bearings[2], bearings[1]));
            double est[4];
            init_estimate(bearings[0], m2, m3, dts[0], dts[1], est);
            Y[0] = value_of(wrapped_angle_difference(est[0], 0.0));
            Y[1] = est[1];
            Y[2] = est[2];
            Y[3] = 0.0;
            Y[4] = 0.0;
            Y[5] = 0.0;
            Y[6] = 0.0;
            double q_min = 1.0 / range_max;
            double q_max = 1.0 / range_min;
            Y[7] = 0.5 * (q_min + q_max);
            mat_zero(P);
            double c[3][4];
            init_coefficients(dts[0], dts[1], c);
            for (int r = 0; r < 4; r++)
                for (int s = 0; s < 4; s++)
                    for (int n = 0; n < 3; n++)
                        P[r][s] += bearing_variance * c[n][r] * c[n][s];
            double angular_jerk = jerk_error * q_max;
            double angular_acceleration = acceleration_error * q_max;
            double P_proc[4][4];
            init_process_covariance<Form>(dts[0], dts[1], alpha, 2.0 * alpha * angular_jerk * angular_jerk,
                                          angular_acceleration * angular_acceleration, angular_jerk * angular_jerk, P_proc);
            for (int r = 0; r < 4; r++)
                for (int s = 0; s < 4; s++)
                    P[r][s] += P_proc[r][s];
            for (int i = 0; i < 3; i++)
                P[4 + i][4 + i] = log_range_variances[i];
            P[7][7] = (q_max - q_min) * (q_max - q_min) / 12.0;
        }

        void predict(double dt, const double (&sensor)[8])
        {
            double F[4][4], Q[4][4];
            Form::fill(dt, alpha, F, Q);
            double sensor_term[8];
            for (int axis = 0; axis < 2; axis++)
                for (int r = 0; r < 4; r++)
                {
                    double s = -sensor[4 * axis + r];
                    for (int m = 0; m < 4; m++)
                        s += F[r][m] * sensor_state[4 * axis + m];
                    sensor_term[4 * axis + r] = s;
                }
            double Qv[8][8];
            mat_zero(Qv);
            for (int axis = 0; axis < 2; axis++)
                for (int r = 0; r < 4; r++)
                    for (int s = 0; s < 4; s++)
                        Qv[4 * axis + r][4 * axis + s] = q_scale * Q[r][s];
            MpcPredictionMap fn;
            fn.Y = Y;
            fn.F = F;
            fn.sensor_term = sensor_term;
            double Y_pred[8];
            double zeta0[16];
            for (int i = 0; i < 16; i++)
                zeta0[i] = 0.0;
            fn.Y_pred = Y;
            fn.propagate(zeta0, Y_pred);
            fn.Y_pred = Y_pred;
            double mean[8], P_out[8][8];
            map_propagate<8, 8>(fn, P, Qv, mean, P_out, Order());
            for (int i = 0; i < 8; i++)
                Y[i] = Y_pred[i] + mean[i];
            Y[0] = value_of(wrapped_angle_difference(Y[0], 0.0));
            mat_copy(P_out, P);
        }

        void update(double bearing)
        {
            const int idx[1] = {0};
            double y[1] = {value_of(wrapped_angle_difference(bearing, Y[0]))};
            double R[1][1] = {{bearing_variance}};
            double S[1][1], S_inv[1][1], det_S, correction[8];
            if (!kalman_update(P, idx, y, R, S, S_inv, det_S, correction))
                return;
            for (int i = 0; i < 8; i++)
                Y[i] += correction[i];
            Y[0] = value_of(wrapped_angle_difference(Y[0], 0.0));
            this->store_diagnostics(y, S, S_inv, det_S);
        }

    public:
        /**
         * KalmanJerk1DBearingMovingSensor default constructor.
         */
        KalmanJerk1DBearingMovingSensor() : alpha(0.0), q_scale(0.0), bearing_variance(0.0), jerk_error(0.0), acceleration_error(0.0), range_min(1.0), range_max(1.0), time_is_relative(false), current_idx(-1), time(0.0)
        {
        }

        /**
         * KalmanJerk1DBearingMovingSensor constructor with parameters.
         *
         * @param alpha alpha used to calculate q factor with q = 2 * alpha * jerk_error^2
         * @param bearing_error The standard deviation of a bearing measurement in radians
         * @param jerk_error The standard deviation of the jerk of each Cartesian coordinate of the target
         * @param time_is_relative Indicates whether time values parsed to the step function are absolute temporal values or temporal differences between consecutive steps.
         * @param range_min Lower bound of the initial range
         * @param range_max Upper bound of the initial range
         * @param log_range_rate_error Initial standard deviation of d lambda, lambda = ln r
         * @param log_range_acceleration_error Initial standard deviation of d2 lambda
         * @param log_range_jerk_error Initial standard deviation of d3 lambda
         * @param acceleration_error The standard deviation of the acceleration of each Cartesian coordinate, used by the JerkExact initialisation (28)
         */
        KalmanJerk1DBearingMovingSensor(double alpha, double bearing_error, double jerk_error, bool time_is_relative, double range_min, double range_max,
                                        double log_range_rate_error, double log_range_acceleration_error, double log_range_jerk_error, double acceleration_error = 0.0)
            : alpha(alpha), q_scale(2.0 * alpha * jerk_error * jerk_error), bearing_variance(bearing_error * bearing_error), jerk_error(jerk_error),
              acceleration_error(acceleration_error), range_min(range_min), range_max(range_max), time_is_relative(time_is_relative), current_idx(-1), time(0.0)
        {
            log_range_variances[0] = log_range_rate_error * log_range_rate_error;
            log_range_variances[1] = log_range_acceleration_error * log_range_acceleration_error;
            log_range_variances[2] = log_range_jerk_error * log_range_jerk_error;
            for (int i = 0; i < 8; i++)
            {
                Y[i] = 0.0;
                sensor_state[i] = 0.0;
            }
            mat_zero(P);
        }

        /**
         * @param time_or_dt absolute time or relative time since last step (depending on how time_is_relative is set)
         * @param bearing measured bearing in radians
         * @param sensor sensor state per axis [x, vx, ax, jx, y, vy, ay, jy] at the measurement time
         */
        void step(double time_or_dt, double bearing, const double (&sensor)[8])
        {
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
                    update(bearing);
                }
            }
            if (current_idx < 2)
            {
                bearings[current_idx + 1] = bearing;
                if (current_idx + 1 == 2)
                    initialise();
                current_idx++;
            }
            for (int i = 0; i < 8; i++)
                sensor_state[i] = sensor[i];
        }

        /**
         * @brief Get Kalman state estimate [beta, d beta, d2 beta, d3 beta, d lambda, d2 lambda, d3 lambda, 1/r].
         */
        double (&get_kalman_vector())[8] { return Y; }

        /**
         * @brief Get the covariance matrix P.
         */
        double (&get_covariance_matrix())[8][8] { return P; }

        /**
         * @brief Get the range estimate 1 / (1/r).
         */
        double get_range() const { return 1.0 / Y[7]; }
    };
}

#endif
