/*
 * Jerk filter for a direction measured as azimuth and elevation, the range unknown.
 *
 * The direction moves on the unit sphere with the jerk model of [Ref1] posed intrinsically:
 * Dw/dt = a, Da/dt = j, Dj/dt = -alpha j + w_noise, with the noise isotropic in the tangent plane.
 * The state is held as a unit vector with tangent vectors, see model/derivation.md.
 *
 * Developed for the Kaepek Project
 */

#ifndef KAEPEK_KALMAN_JERK_2D_AZEL_H
#define KAEPEK_KALMAN_JERK_2D_AZEL_H

#include "kalman_jerk_policies.hpp"
#include "kalman_jerk_linalg.hpp"
#include "kalman_jerk_init.hpp"
#include "kalman_jerk_autodiff.hpp"
#include "kalman_jerk_sphere.hpp"
#include "kalman_jerk_direction_init.hpp"
#include "generated/azel.hpp"

namespace kaepek
{
    /**
     * @brief Integrates the noise free direction dynamics over dt with n Runge Kutta steps.
     */
    template <typename S>
    inline void azel_flow(S x[12], double alpha, double dt, int n)
    {
        double h = dt / n;
        for (int step = 0; step < n; step++)
        {
            S k1[12], k2[12], k3[12], k4[12], t[12];
            azel_dynamics(x, alpha, k1);
            for (int i = 0; i < 12; i++)
                t[i] = x[i] + 0.5 * h * k1[i];
            azel_dynamics(t, alpha, k2);
            for (int i = 0; i < 12; i++)
                t[i] = x[i] + 0.5 * h * k2[i];
            azel_dynamics(t, alpha, k3);
            for (int i = 0; i < 12; i++)
                t[i] = x[i] + h * k3[i];
            azel_dynamics(t, alpha, k4);
            for (int i = 0; i < 12; i++)
                x[i] = x[i] + (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
    }

    /**
     * @brief Error at the end of an interval of the trajectory starting at error d from the estimate (x0, b0),
     *  relative to the propagated estimate (x1, b1).
     */
    template <typename S>
    inline void azel_psi(const double *x0, const double *b0, const double *x1, const double *b1, double alpha, double dt, int n, const S *d, S *out)
    {
        S xs[12], bs[6];
        direction_retract(x0, b0, d, xs, bs);
        azel_flow(xs, alpha, dt, n);
        direction_inverse_retract(x1, b1, xs, out);
    }

    template <class Form = JerkSmallAlphaT, class Order = FirstOrder, class Diag = NoDiagnostics>
    class KalmanJerk2DAzEl : public DiagnosticsStore<Diag, 2>
    {
    private:
        double alpha;
        double q_scale;
        double direction_variance;
        double jerk_variance;
        double acceleration_variance;
        bool time_is_relative;
        int substeps;
        int current_idx;

        double time;
        double dts[2];
        double measurements[3][3];
        double measurement_covariances[3][2][2];
        bool measurement_isotropic[3];

        double x[12];
        double b[6];
        double P[8][8];
        double kalman_vector[8];
        double covariance_out[8][8];

        void initialise()
        {
            direction_initialise<Form>(measurements, measurement_covariances, measurement_isotropic, dts, alpha, q_scale, acceleration_variance, jerk_variance, x, b, P);
        }

        /**
         * @brief Integrates the state, the basis, the transition matrix and the process noise covariance over dt.
         */
        void integrate(double dt, double (&x1)[12], double (&b1)[6], double (&F)[8][8], double (&Qd)[8][8])
        {
            const double a_d = Form::drift_alpha(alpha);
            const int n = substeps;
            const double h = dt / n;
            const int Z_SIZE = 12 + 6 + 64 + 64;
            double Z[Z_SIZE];
            for (int i = 0; i < 12; i++)
                Z[i] = x[i];
            for (int i = 0; i < 6; i++)
                Z[12 + i] = b[i];
            for (int i = 0; i < 8; i++)
                for (int k = 0; k < 8; k++)
                {
                    Z[18 + 8 * i + k] = (i == k) ? 1.0 : 0.0;
                    Z[82 + 8 * i + k] = 0.0;
                }
            for (int step = 0; step < n; step++)
            {
                double k1[Z_SIZE], k2[Z_SIZE], k3[Z_SIZE], k4[Z_SIZE], t[Z_SIZE];
                rate(Z, a_d, k1);
                for (int i = 0; i < Z_SIZE; i++)
                    t[i] = Z[i] + 0.5 * h * k1[i];
                rate(t, a_d, k2);
                for (int i = 0; i < Z_SIZE; i++)
                    t[i] = Z[i] + 0.5 * h * k2[i];
                rate(t, a_d, k3);
                for (int i = 0; i < Z_SIZE; i++)
                    t[i] = Z[i] + h * k3[i];
                rate(t, a_d, k4);
                for (int i = 0; i < Z_SIZE; i++)
                    Z[i] += (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
            }
            for (int i = 0; i < 12; i++)
                x1[i] = Z[i];
            for (int i = 0; i < 6; i++)
                b1[i] = Z[12 + i];
            for (int i = 0; i < 8; i++)
                for (int k = 0; k < 8; k++)
                {
                    F[i][k] = Z[18 + 8 * i + k];
                    Qd[i][k] = Z[82 + 8 * i + k];
                }
            mat_symmetrise(Qd);
            direction_normalise(x1, b1);
        }

        void rate(const double *Z, double a_d, double *dZ)
        {
            azel_dynamics(Z, a_d, dZ);
            azel_basis_rate(Z, Z + 12, dZ + 12);
            double A[8][8];
            azel_error_dynamics(Z, Z + 12, a_d, A);
            const double *Phi = Z + 18;
            const double *Q = Z + 82;
            for (int i = 0; i < 8; i++)
                for (int k = 0; k < 8; k++)
                {
                    double s_phi = 0.0, s_q = 0.0;
                    for (int m = 0; m < 8; m++)
                    {
                        s_phi += A[i][m] * Phi[8 * m + k];
                        s_q += A[i][m] * Q[8 * m + k] + Q[8 * i + m] * A[k][m];
                    }
                    dZ[18 + 8 * i + k] = s_phi;
                    dZ[82 + 8 * i + k] = s_q;
                }
            dZ[82 + 8 * 3 + 3] += q_scale;
            dZ[82 + 8 * 7 + 7] += q_scale;
        }

        void propagate(double dt, double (&x1)[12], double (&b1)[6], double (&F)[8][8], double (&Qd)[8][8], FirstOrder)
        {
            (void)dt;
            double FPF[8][8];
            mat_sandwich(F, P, FPF);
            for (int i = 0; i < 8; i++)
                for (int k = 0; k < 8; k++)
                    P[i][k] = FPF[i][k] + Qd[i][k];
            for (int i = 0; i < 12; i++)
                x[i] = x1[i];
            for (int i = 0; i < 6; i++)
                b[i] = b1[i];
        }

        void propagate(double dt, double (&x1)[12], double (&b1)[6], double (&F)[8][8], double (&Qd)[8][8], SecondOrder)
        {
            const double a_d = Form::drift_alpha(alpha);
            double H[8][8][8];
            for (int k = 0; k < 8; k++)
                for (int l = k; l < 8; l++)
                {
                    HyperDual d[8], out[8];
                    for (int i = 0; i < 8; i++)
                        d[i] = HyperDual(0.0, i == k ? 1.0 : 0.0, i == l ? 1.0 : 0.0, 0.0);
                    azel_psi(x, b, x1, b1, a_d, dt, substeps, d, out);
                    for (int i = 0; i < 8; i++)
                    {
                        H[i][k][l] = out[i].e12;
                        H[i][l][k] = out[i].e12;
                    }
                }
            double HP[8][8][8];
            for (int i = 0; i < 8; i++)
                mat_mul(H[i], P, HP[i]);
            double mean[8];
            for (int i = 0; i < 8; i++)
            {
                double s = 0.0;
                for (int k = 0; k < 8; k++)
                    s += HP[i][k][k];
                mean[i] = 0.5 * s;
            }
            double FPF[8][8];
            mat_sandwich(F, P, FPF);
            for (int i = 0; i < 8; i++)
                for (int k = 0; k < 8; k++)
                {
                    double s = 0.0;
                    for (int r = 0; r < 8; r++)
                        for (int c = 0; c < 8; c++)
                            s += HP[i][r][c] * HP[k][c][r];
                    P[i][k] = FPF[i][k] + 0.5 * s + Qd[i][k];
                }
            mat_symmetrise(P);
            direction_retract(x1, b1, mean, x, b);
            direction_normalise(x, b);
        }

        void propagate(double dt, double (&x1)[12], double (&b1)[6], double (&F)[8][8], double (&Qd)[8][8], Unscented)
        {
            const double a_d = Form::drift_alpha(alpha);
            const int n = 8;
            const double lambda = Unscented::alpha_s() * Unscented::alpha_s() * (n + Unscented::kappa_s()) - n;
            double scaled[8][8], L[8][8];
            for (int i = 0; i < 8; i++)
                for (int k = 0; k < 8; k++)
                    scaled[i][k] = (n + lambda) * P[i][k];
            if (!cholesky(scaled, L))
            {
                propagate(dt, x1, b1, F, Qd, FirstOrder());
                return;
            }
            double w_m0 = lambda / (n + lambda);
            double w_c0 = w_m0 + 1.0 - Unscented::alpha_s() * Unscented::alpha_s() + Unscented::beta_s();
            double w_i = 1.0 / (2.0 * (n + lambda));
            double chi[2 * 8 + 1][12];
            for (int p = 0; p < 2 * n + 1; p++)
            {
                double d[8], bs[6];
                for (int i = 0; i < 8; i++)
                    d[i] = p == 0 ? 0.0 : (p <= n ? L[i][p - 1] : -L[i][p - 1 - n]);
                direction_retract(x, b, d, chi[p], bs);
                azel_flow(chi[p], a_d, dt, substeps);
            }
            double mean[8] = {0, 0, 0, 0, 0, 0, 0, 0};
            for (int p = 0; p < 2 * n + 1; p++)
            {
                double xi[8];
                direction_inverse_retract(x1, b1, chi[p], xi);
                double w = p == 0 ? w_m0 : w_i;
                for (int i = 0; i < 8; i++)
                    mean[i] += w * xi[i];
            }
            direction_retract(x1, b1, mean, x, b);
            direction_normalise(x, b);
            double xi[2 * 8 + 1][8], mean2[8] = {0, 0, 0, 0, 0, 0, 0, 0};
            for (int p = 0; p < 2 * n + 1; p++)
            {
                direction_inverse_retract(x, b, chi[p], xi[p]);
                double w = p == 0 ? w_m0 : w_i;
                for (int i = 0; i < 8; i++)
                    mean2[i] += w * xi[p][i];
            }
            for (int i = 0; i < 8; i++)
                for (int k = 0; k < 8; k++)
                {
                    double s = Qd[i][k];
                    for (int p = 0; p < 2 * n + 1; p++)
                        s += (p == 0 ? w_c0 : w_i) * (xi[p][i] - mean2[i]) * (xi[p][k] - mean2[k]);
                    P[i][k] = s;
                }
            mat_symmetrise(P);
        }

        void predict(double dt)
        {
            double x1[12], b1[6], F[8][8], Qd[8][8];
            integrate(dt, x1, b1, F, Qd);
            propagate(dt, x1, b1, F, Qd, Order());
        }

        void update(const double *m, const double (&R)[2][2], bool isotropic)
        {
            double y[2], R_b[2][2];
            direction_innovation(x, b, m, R, isotropic, y, R_b);
            const int idx[2] = {0, 4};
            double S[2][2], S_inv[2][2], det_S, correction[8];
            if (!kalman_update(P, idx, y, R_b, S, S_inv, det_S, correction))
                return;
            double xn[12], bn[6];
            direction_retract(x, b, correction, xn, bn);
            for (int i = 0; i < 12; i++)
                x[i] = xn[i];
            for (int i = 0; i < 6; i++)
                b[i] = bn[i];
            direction_normalise(x, b);
            this->store_diagnostics(y, S, S_inv, det_S);
        }

        void step_measurement(double time_or_dt, double azimuth, double elevation, const double (&R)[2][2], bool isotropic)
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
                    predict(dt);
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
        }

    public:
        /**
         * KalmanJerk2DAzEl default constructor.
         */
        KalmanJerk2DAzEl() : alpha(0.0), q_scale(0.0), direction_variance(0.0), jerk_variance(0.0), acceleration_variance(0.0), time_is_relative(false), substeps(4), current_idx(-1), time(0.0)
        {
        }

        /**
         * KalmanJerk2DAzEl constructor with parameters.
         *
         * @param alpha alpha used to calculate q factor with q = 2 * alpha * jerk_error^2
         * @param direction_error The standard deviation of the angular error of a measured direction in radians, isotropic on the sphere
         * @param jerk_error The standard deviation of the angular jerk in radians per second cubed
         * @param time_is_relative Indicates whether time values parsed to the step function are absolute temporal values or temporal differences between consecutive steps.
         * @param acceleration_error The standard deviation of the angular acceleration, used by the JerkExact initialisation (28)
         * @param substeps Number of Runge Kutta steps per sampling interval
         */
        KalmanJerk2DAzEl(double alpha, double direction_error, double jerk_error, bool time_is_relative, double acceleration_error = 0.0, int substeps = 4)
            : alpha(alpha), q_scale(2.0 * alpha * jerk_error * jerk_error), direction_variance(direction_error * direction_error),
              jerk_variance(jerk_error * jerk_error), acceleration_variance(acceleration_error * acceleration_error),
              time_is_relative(time_is_relative), substeps(substeps), current_idx(-1), time(0.0)
        {
            for (int i = 0; i < 12; i++)
                x[i] = 0.0;
            for (int i = 0; i < 6; i++)
                b[i] = 0.0;
            mat_zero(P);
            for (int i = 0; i < 8; i++)
                kalman_vector[i] = 0.0;
            mat_zero(covariance_out);
        }

        /**
         * Step with the isotropic measurement covariance direction_error^2 I.
         * @param time_or_dt absolute time or relative time since last step (depending on how time_is_relative is set)
         * @param azimuth measured azimuth in radians
         * @param elevation measured elevation in radians
         */
        void step(double time_or_dt, double azimuth, double elevation)
        {
            double R[2][2] = {{direction_variance, 0.0}, {0.0, direction_variance}};
            step_measurement(time_or_dt, azimuth, elevation, R, true);
        }

        /**
         * Step with a measurement covariance in the tangent plane of the measured direction, in the frame of
         * increasing azimuth and increasing elevation.
         * @param time_or_dt absolute time or relative time since last step (depending on how time_is_relative is set)
         * @param azimuth measured azimuth in radians
         * @param elevation measured elevation in radians
         * @param R measurement covariance in radians squared
         */
        void step(double time_or_dt, double azimuth, double elevation, const double (&R)[2][2])
        {
            step_measurement(time_or_dt, azimuth, elevation, R, false);
        }

        /**
         * @brief Tangent plane covariance of a measurement described by azimuth and elevation standard deviations.
         */
        static void azel_noise_to_tangent(double elevation, double azimuth_error, double elevation_error, double (&R)[2][2])
        {
            double c = cos(elevation);
            R[0][0] = azimuth_error * azimuth_error * c * c;
            R[0][1] = 0.0;
            R[1][0] = 0.0;
            R[1][1] = elevation_error * elevation_error;
        }

        /**
         * @brief True when the estimated direction is away from the zenith and nadir, where the azimuth is defined.
         */
        bool azimuth_defined()
        {
            double e_a[3], e_e[3];
            return azel_frame(x, e_a, e_e);
        }

        /**
         * @brief Get Kalman state estimate [azimuth, w_a, a_a, j_a, elevation, w_e, a_e, j_e] with the angular rate,
         *  acceleration and jerk components along increasing azimuth and increasing elevation.
         *  At the zenith and nadir the components are given in the internal basis.
         */
        double (&get_kalman_vector())[8]
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
            return kalman_vector;
        }

        /**
         * @brief Get the covariance matrix in the frame of get_kalman_vector.
         */
        double (&get_covariance_matrix())[8][8]
        {
            double e_a[3], e_e[3];
            if (!azel_frame(x, e_a, e_e))
            {
                mat_copy(P, covariance_out);
                return covariance_out;
            }
            double M[2][2] = {{dot3(b, e_a), dot3(b + 3, e_a)}, {dot3(b, e_e), dot3(b + 3, e_e)}};
            double T[8][8];
            mat_zero(T);
            for (int k = 0; k < 4; k++)
                for (int i = 0; i < 2; i++)
                    for (int c = 0; c < 2; c++)
                        T[4 * i + k][4 * c + k] = M[i][c];
            mat_sandwich(T, P, covariance_out);
            return covariance_out;
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
        double (&get_basis_covariance_matrix())[8][8] { return P; }
    };
}

#endif
