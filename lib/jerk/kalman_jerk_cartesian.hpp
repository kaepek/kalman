/*
 * Jerk filter for N Cartesian axes, Section V of [Ref1].
 *
 * State per axis [position, velocity, acceleration, jerk], stacked by axis.
 * F_N = I_N (x) F, Q_N = I_N (x) Q, H selects the positions, R is a full N by N measurement covariance.
 *
 * Developed for the Kaepek Project
 */

#ifndef KAEPEK_KALMAN_JERK_CARTESIAN_H
#define KAEPEK_KALMAN_JERK_CARTESIAN_H

#include "kalman_jerk_policies.hpp"
#include "kalman_jerk_linalg.hpp"
#include "kalman_jerk_init.hpp"

namespace kaepek
{
    template <int N, class Form = JerkSmallAlphaT, class Diag = NoDiagnostics>
    class KalmanJerkCartesian : public DiagnosticsStore<Diag, N>
    {
    private:
        double alpha;
        double x_variance;
        double jerk_variance;
        double acceleration_variance;
        double q_scale;
        bool time_is_relative;
        int current_idx;

        double time;
        double dts[2];
        double measurements[3][N];
        double measurement_covariances[3][N][N];

        double X[4 * N];
        double P[4 * N][4 * N];
        double eular_state[1 + 4 * N];

        void initialise()
        {
            for (int i = 0; i < N; i++)
                init_estimate(measurements[0][i], measurements[1][i], measurements[2][i], dts[0], dts[1], X + 4 * i);
            double c[3][4];
            init_coefficients(dts[0], dts[1], c);
            double P_proc[4][4];
            init_process_covariance<Form>(dts[0], dts[1], alpha, q_scale, acceleration_variance, jerk_variance, P_proc);
            for (int i = 0; i < N; i++)
                for (int k = 0; k < N; k++)
                    for (int r = 0; r < 4; r++)
                        for (int s = 0; s < 4; s++)
                        {
                            double v = 0.0;
                            for (int n = 0; n < 3; n++)
                                v += measurement_covariances[n][i][k] * c[n][r] * c[n][s];
                            if (i == k)
                                v += P_proc[r][s];
                            P[4 * i + r][4 * k + s] = v;
                        }
        }

        void predict(double dt)
        {
            double F[4][4], Q[4][4];
            Form::fill(dt, alpha, F, Q);
            double Xn[4 * N];
            for (int i = 0; i < N; i++)
                for (int r = 0; r < 4; r++)
                {
                    double s = 0.0;
                    for (int m = 0; m < 4; m++)
                        s += F[r][m] * X[4 * i + m];
                    Xn[4 * i + r] = s;
                }
            for (int i = 0; i < 4 * N; i++)
                X[i] = Xn[i];
            for (int i = 0; i < N; i++)
                for (int k = 0; k < N; k++)
                {
                    double block[4][4], out[4][4];
                    for (int r = 0; r < 4; r++)
                        for (int s = 0; s < 4; s++)
                            block[r][s] = P[4 * i + r][4 * k + s];
                    mat_sandwich(F, block, out);
                    for (int r = 0; r < 4; r++)
                        for (int s = 0; s < 4; s++)
                            P[4 * i + r][4 * k + s] = out[r][s] + (i == k ? q_scale * Q[r][s] : 0.0);
                }
        }

        void update(const double (&x)[N], const double (&R)[N][N])
        {
            int idx[N];
            double y[N];
            for (int i = 0; i < N; i++)
            {
                idx[i] = 4 * i;
                y[i] = x[i] - X[4 * i];
            }
            double S[N][N], S_inv[N][N], det_S, correction[4 * N];
            if (!kalman_update(P, idx, y, R, S, S_inv, det_S, correction))
                return;
            for (int i = 0; i < 4 * N; i++)
                X[i] += correction[i];
            this->store_diagnostics(y, S, S_inv, det_S);
        }

        void update_eular(double dt, const double (&x)[N])
        {
            for (int i = 0; i < N; i++)
            {
                double *e = eular_state + 1 + 4 * i;
                double v = (x[i] - e[0]) / dt;
                double a = (v - e[1]) / dt;
                double j = (a - e[2]) / dt;
                e[0] = x[i];
                e[1] = v;
                e[2] = current_idx >= 1 ? a : 0.0;
                e[3] = current_idx >= 2 ? j : 0.0;
            }
        }

    public:
        /**
         * KalmanJerkCartesian default constructor.
         */
        KalmanJerkCartesian() : alpha(0.0), x_variance(0.0), jerk_variance(0.0), acceleration_variance(0.0), q_scale(0.0), time_is_relative(false), current_idx(-1), time(0.0)
        {
        }

        /**
         * KalmanJerkCartesian constructor with parameters.
         *
         * @param alpha alpha used to calculate q factor with q = 2 * alpha * x_jerk_error^2
         * @param x_resolution_error The standard deviation of each measured coordinate, used when no measurement covariance is given
         * @param x_jerk_error The standard deviation of the jerk of each coordinate
         * @param time_is_relative Indicates whether time values parsed to the step function are absolute temporal values or temporal differences between consecutive steps.
         * @param x_acceleration_error The standard deviation of the acceleration of each coordinate, used by the JerkExact initialisation (28)
         */
        KalmanJerkCartesian(double alpha, double x_resolution_error, double x_jerk_error, bool time_is_relative, double x_acceleration_error = 0.0)
            : alpha(alpha), x_variance(x_resolution_error * x_resolution_error), jerk_variance(x_jerk_error * x_jerk_error),
              acceleration_variance(x_acceleration_error * x_acceleration_error), q_scale(2.0 * alpha * x_jerk_error * x_jerk_error),
              time_is_relative(time_is_relative), current_idx(-1), time(0.0)
        {
            mat_zero(P);
            for (int i = 0; i < 4 * N; i++)
                X[i] = 0.0;
            for (int i = 0; i < 1 + 4 * N; i++)
                eular_state[i] = 0.0;
        }

        /**
         * Step with the measurement covariance x_resolution_error^2 I.
         * @param time_or_dt absolute time or relative time since last step (depending on how time_is_relative is set)
         * @param x measured coordinates
         */
        void step(double time_or_dt, const double (&x)[N])
        {
            double R[N][N];
            mat_zero(R);
            for (int i = 0; i < N; i++)
                R[i][i] = x_variance;
            step(time_or_dt, x, R);
        }

        /**
         * Step with a full measurement covariance.
         * @param time_or_dt absolute time or relative time since last step (depending on how time_is_relative is set)
         * @param x measured coordinates
         * @param R measurement covariance of x
         */
        void step(double time_or_dt, const double (&x)[N], const double (&R)[N][N])
        {
            if (current_idx == -1)
            {
                time = time_or_dt;
                eular_state[0] = time;
                for (int i = 0; i < N; i++)
                    eular_state[1 + 4 * i] = x[i];
            }
            else
            {
                double dt = time_is_relative ? time_or_dt : time_or_dt - time;
                time = time_is_relative ? time + time_or_dt : time_or_dt;
                eular_state[0] = time;
                update_eular(dt, x);
                if (current_idx < 2)
                    dts[current_idx] = dt;
                else
                {
                    predict(dt);
                    update(x, R);
                }
            }
            if (current_idx < 2)
            {
                int n = current_idx + 1;
                for (int i = 0; i < N; i++)
                {
                    measurements[n][i] = x[i];
                    for (int k = 0; k < N; k++)
                        measurement_covariances[n][i][k] = R[i][k];
                }
                if (n == 2)
                    initialise();
                current_idx++;
            }
        }

        /**
         * @brief Get Kalman state estimate, per axis [position, velocity, acceleration, jerk].
         */
        double (&get_kalman_vector())[4 * N] { return X; }

        /**
         * @brief Get the covariance matrix P.
         */
        double (&get_covariance_matrix())[4 * N][4 * N] { return P; }

        /**
         * @brief Get Eular state estimate [time, then per axis position, velocity, acceleration, jerk].
         */
        double (&get_eular_vector())[1 + 4 * N] { return eular_state; }
    };

    template <class Form = JerkSmallAlphaT, class Diag = NoDiagnostics>
    using KalmanJerk2D = KalmanJerkCartesian<2, Form, Diag>;

    template <class Form = JerkSmallAlphaT, class Diag = NoDiagnostics>
    using KalmanJerk3D = KalmanJerkCartesian<3, Form, Diag>;
}

#endif
