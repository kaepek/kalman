/*
 * Initialisation of a direction state from three measured directions.
 *
 * The estimates (22) of [Ref1] are formed in Riemannian normal coordinates of the sphere centred on the third
 * measured direction, and the initial covariance is the error analysis (23) to (28) carried through these coordinates.
 *
 * Developed for the Kaepek Project
 */

#ifndef KAEPEK_KALMAN_JERK_DIRECTION_INIT_H
#define KAEPEK_KALMAN_JERK_DIRECTION_INIT_H

#include "kalman_jerk_linalg.hpp"
#include "kalman_jerk_init.hpp"
#include "kalman_jerk_autodiff.hpp"
#include "kalman_jerk_sphere.hpp"

namespace kaepek
{
    /**
     * @brief Measurement covariance at the direction m in a basis bn of its tangent plane.
     *  R is isotropic, or given in the frame of increasing azimuth and increasing elevation at m;
     *  at the zenith and nadir the mean of its two variances is used.
     */
    inline void direction_tangent_covariance(const double *m, const double *bn, const double (&R)[2][2], bool isotropic, double (&R_b)[2][2])
    {
        double e_a[3], e_e[3];
        if (isotropic || !azel_frame(m, e_a, e_e))
        {
            double v = isotropic ? R[0][0] : 0.5 * (R[0][0] + R[1][1]);
            R_b[0][0] = v;
            R_b[0][1] = 0.0;
            R_b[1][0] = 0.0;
            R_b[1][1] = v;
            return;
        }
        double M[2][2] = {{dot3(bn, e_a), dot3(bn, e_e)}, {dot3(bn + 3, e_a), dot3(bn + 3, e_e)}};
        mat_sandwich(M, R, R_b);
    }

    /**
     * @brief Tangent basis at measurement n used to express its error: the basis at the third measurement
     *  carried along the great circle to measurement n.
     */
    inline void direction_measurement_basis(const double (&m)[3][3], int n, double *bn)
    {
        double b0[6];
        tangent_basis(m[2], b0);
        if (n == 2)
        {
            for (int i = 0; i < 6; i++)
                bn[i] = b0[i];
            return;
        }
        transport_basis(m[2], m[n], b0, bn);
    }

    /**
     * @brief Estimate (22) of the direction state from three measured directions displaced by tangent errors.
     */
    template <typename S>
    inline void direction_estimate(const double (&m)[3][3], const double (&dts)[2], const S (&noise)[6], S (&x_est)[12], S (&b_est)[6])
    {
        S mp[3][3];
        for (int n = 0; n < 3; n++)
        {
            double bn[6];
            direction_measurement_basis(m, n, bn);
            S mn[3], v[3], r[3];
            for (int i = 0; i < 3; i++)
                mn[i] = S(m[n][i]);
            basis_vector(bn, noise[2 * n], noise[2 * n + 1], v);
            transport_vector(mn, v, r);
            rotate3(r, mn, mp[n]);
        }
        tangent_basis(mp[2], b_est);
        S p[3][2];
        for (int n = 0; n < 3; n++)
        {
            S v[3];
            log_map(mp[2], mp[n], v);
            p[n][0] = dot3(v, b_est);
            p[n][1] = dot3(v, b_est + 3);
        }
        S axis[2][4];
        for (int i = 0; i < 2; i++)
            init_estimate(p[0][i], p[1][i], S(0.0), dts[0], dts[1], axis[i]);
        for (int i = 0; i < 3; i++)
        {
            x_est[i] = mp[2][i];
            x_est[3 + i] = axis[0][1] * b_est[i] + axis[1][1] * b_est[3 + i];
            x_est[6 + i] = axis[0][2] * b_est[i] + axis[1][2] * b_est[3 + i];
            x_est[9 + i] = S(0.0);
        }
    }

    /**
     * @brief Initial direction state, basis and 8 by 8 covariance.
     *  q_scale, var_m and var_j are in angular units and enter through (28) or (29).
     */
    template <class Form>
    inline void direction_initialise(const double (&m)[3][3], const double (&R)[3][2][2], const bool (&isotropic)[3], const double (&dts)[2],
                                     double alpha, double q_scale, double var_m, double var_j, double (&x)[12], double (&b)[6], double (&P)[8][8])
    {
        double zero[6] = {0, 0, 0, 0, 0, 0};
        direction_estimate(m, dts, zero, x, b);
        Dual<6> noise[6];
        for (int i = 0; i < 6; i++)
            noise[i] = Dual<6>::variable(0.0, i);
        Dual<6> x_est[12], b_est[6], d[8];
        direction_estimate(m, dts, noise, x_est, b_est);
        direction_inverse_retract(x, b, x_est, d);
        double G[8][6];
        for (int i = 0; i < 8; i++)
            for (int k = 0; k < 6; k++)
                G[i][k] = d[i].d[k];
        double R_block[6][6];
        mat_zero(R_block);
        for (int n = 0; n < 3; n++)
        {
            double bn[6], R_b[2][2];
            direction_measurement_basis(m, n, bn);
            direction_tangent_covariance(m[n], bn, R[n], isotropic[n], R_b);
            for (int i = 0; i < 2; i++)
                for (int k = 0; k < 2; k++)
                    R_block[2 * n + i][2 * n + k] = R_b[i][k];
        }
        mat_sandwich(G, R_block, P);
        double P_proc[4][4];
        init_process_covariance<Form>(dts[0], dts[1], alpha, q_scale, var_m, var_j, P_proc);
        for (int i = 0; i < 2; i++)
            for (int r = 0; r < 4; r++)
                for (int s = 0; s < 4; s++)
                    P[4 * i + r][4 * i + s] += P_proc[r][s];
    }

    /**
     * @brief Innovation and measurement covariance in the basis b at the predicted direction u.
     */
    inline void direction_innovation(const double *u, const double *b, const double *m, const double (&R)[2][2], bool isotropic, double (&y)[2], double (&R_b)[2][2])
    {
        double v[3];
        log_map(u, m, v);
        y[0] = dot3(v, b);
        y[1] = dot3(v, b + 3);
        if (isotropic)
        {
            R_b[0][0] = R[0][0];
            R_b[0][1] = 0.0;
            R_b[1][0] = 0.0;
            R_b[1][1] = R[1][1];
            return;
        }
        double bm[6];
        transport_basis(u, m, b, bm);
        direction_tangent_covariance(m, bm, R, false, R_b);
    }
}

#endif
