/*
 * Initialisation of the jerk filters from the first three measurements, (22) to (29) of [Ref1].
 *
 * With measurements M(1), M(2), M(3) separated by dt1 and dt2 the estimates (22) are
 *  x = M(3), v = (M(3) - M(2)) / dt2, a = ((M(3) - M(2)) / dt2 - (M(2) - M(1)) / dt1) / dt2, j = 0.
 * Each is linear in the measurements with coefficient vectors c_n over [x, v, a, j].
 *
 * Developed for the Kaepek Project
 */

#ifndef KAEPEK_KALMAN_JERK_INIT_H
#define KAEPEK_KALMAN_JERK_INIT_H

#include "kalman_jerk_linalg.hpp"
#include "kalman_jerk_policies.hpp"
#include "generated/jerk_block.hpp"

namespace kaepek
{
    /**
     * @brief Coefficients of M(n) in the estimates (22), c[n - 1] = [x, v, a, j].
     */
    inline void init_coefficients(double dt1, double dt2, double c[3][4])
    {
        c[0][0] = 0.0;
        c[0][1] = 0.0;
        c[0][2] = 1.0 / (dt1 * dt2);
        c[0][3] = 0.0;
        c[1][0] = 0.0;
        c[1][1] = -1.0 / dt2;
        c[1][2] = -1.0 / (dt2 * dt2) - 1.0 / (dt1 * dt2);
        c[1][3] = 0.0;
        c[2][0] = 1.0;
        c[2][1] = 1.0 / dt2;
        c[2][2] = 1.0 / (dt2 * dt2);
        c[2][3] = 0.0;
    }

    /**
     * @brief Estimates (22) of one axis from three measurements.
     */
    template <typename S>
    inline void init_estimate(const S &m1, const S &m2, const S &m3, double dt1, double dt2, S x[4])
    {
        S v2 = (m3 - m2) / dt2;
        S v1 = (m2 - m1) / dt1;
        x[0] = m3;
        x[1] = v2;
        x[2] = (v2 - v1) / dt2;
        x[3] = S(0.0);
    }

    /**
     * @brief Terms of the initial covariance of one axis arising from the target acceleration, jerk and
     *  process noise: (28) for JerkExact and (29) for JerkSmallAlphaT.
     *  q_scale is 2 alpha sigma_j^2, var_m and var_j the variances of the target acceleration and jerk.
     */
    template <class Form>
    inline void init_process_covariance(double dt1, double dt2, double alpha, double q_scale, double var_m, double var_j, double P[4][4]);

    template <>
    inline void init_process_covariance<JerkSmallAlphaT>(double dt1, double dt2, double alpha, double q_scale, double var_m, double var_j, double P[4][4])
    {
        (void)dt1;
        (void)alpha;
        (void)q_scale;
        (void)var_m;
        jerk_initial_covariance_small(dt2, 0.0, var_j, P);
    }

    template <>
    inline void init_process_covariance<JerkExact>(double dt1, double dt2, double alpha, double q_scale, double var_m, double var_j, double P[4][4])
    {
        // true states X(2) = F1 X(1) + u(1), X(3) = F2 X(2) + u(2); error e = sum_n c_n M(n) - X(3)
        double F1[4][4], F2[4][4], Q1[4][4], Q2[4][4];
        JerkExact::fill(dt1, alpha, F1, Q1);
        JerkExact::fill(dt2, alpha, F2, Q2);
        for (int i = 0; i < 4; i++)
            for (int k = 0; k < 4; k++)
            {
                Q1[i][k] *= q_scale;
                Q2[i][k] *= q_scale;
            }
        double c[3][4];
        init_coefficients(dt1, dt2, c);
        double F21[4][4];
        mat_mul(F2, F1, F21);
        double A[4][4], G1[4][4], G2[4][4];
        for (int i = 0; i < 4; i++)
            for (int k = 0; k < 4; k++)
            {
                A[i][k] = c[0][i] * (k == 0 ? 1.0 : 0.0) + c[1][i] * F1[0][k] + c[2][i] * F21[0][k] - F21[i][k];
                G1[i][k] = c[1][i] * (k == 0 ? 1.0 : 0.0) + c[2][i] * F2[0][k] - F2[i][k];
                G2[i][k] = c[2][i] * (k == 0 ? 1.0 : 0.0) - (i == k ? 1.0 : 0.0);
            }
        double S1[4][4];
        mat_zero(S1);
        S1[2][2] = var_m;
        S1[3][3] = var_j;
        double T1[4][4], T2[4][4], T3[4][4];
        mat_sandwich(A, S1, T1);
        mat_sandwich(G1, Q1, T2);
        mat_sandwich(G2, Q2, T3);
        for (int i = 0; i < 4; i++)
            for (int k = 0; k < 4; k++)
                P[i][k] = T1[i][k] + T2[i][k] + T3[i][k];
    }
}

#endif
