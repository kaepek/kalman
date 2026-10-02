/*
 * Propagation of a mean and covariance through a map psi(zeta), zeta = [error (NE), process noise (NV)],
 * giving the error at the prediction relative to psi(0). The error has covariance P and the noise Qv.
 *
 * Fn provides template <typename S> void operator()(const S *zeta, S *out) const, and
 * void reexpress(const double *mean, const double *xi, double *out) const which expresses an error
 * relative to psi(0) as an error relative to the prediction moved by mean.
 *
 * Developed for the Kaepek Project
 */

#ifndef KAEPEK_KALMAN_JERK_MAP_PROPAGATION_H
#define KAEPEK_KALMAN_JERK_MAP_PROPAGATION_H

#include "kalman_jerk_policies.hpp"
#include "kalman_jerk_linalg.hpp"
#include "kalman_jerk_autodiff.hpp"

namespace kaepek
{
    template <int NE, int NV>
    inline void map_augmented_covariance(const double (&P)[NE][NE], const double (&Qv)[NV][NV], double (&Pz)[NE + NV][NE + NV])
    {
        mat_zero(Pz);
        for (int i = 0; i < NE; i++)
            for (int k = 0; k < NE; k++)
                Pz[i][k] = P[i][k];
        for (int i = 0; i < NV; i++)
            for (int k = 0; k < NV; k++)
                Pz[NE + i][NE + k] = Qv[i][k];
    }

    template <int NE, int NV, class Fn>
    inline void map_jacobian(const Fn &fn, double (&J)[NE][NE + NV])
    {
        const int N = NE + NV;
        Dual<N> zeta[N], out[NE];
        for (int i = 0; i < N; i++)
            zeta[i] = Dual<N>::variable(0.0, i);
        fn(zeta, out);
        for (int i = 0; i < NE; i++)
            for (int k = 0; k < N; k++)
                J[i][k] = out[i].d[k];
    }

    template <int NE, int NV, class Fn>
    inline void map_propagate(const Fn &fn, const double (&P)[NE][NE], const double (&Qv)[NV][NV], double (&mean)[NE], double (&P_out)[NE][NE], FirstOrder)
    {
        double Pz[NE + NV][NE + NV], J[NE][NE + NV];
        map_augmented_covariance(P, Qv, Pz);
        map_jacobian<NE, NV>(fn, J);
        mat_sandwich(J, Pz, P_out);
        mat_symmetrise(P_out);
        for (int i = 0; i < NE; i++)
            mean[i] = 0.0;
    }

    template <int NE, int NV, class Fn>
    inline void map_propagate(const Fn &fn, const double (&P)[NE][NE], const double (&Qv)[NV][NV], double (&mean)[NE], double (&P_out)[NE][NE], SecondOrder)
    {
        const int N = NE + NV;
        double Pz[N][N], J[NE][N];
        map_augmented_covariance(P, Qv, Pz);
        map_jacobian<NE, NV>(fn, J);
        // second derivatives of psi, each replaced in place by its product with Pz
        double HP[NE][N][N];
        for (int k = 0; k < N; k++)
            for (int l = k; l < N; l++)
            {
                HyperDual zeta[N], out[NE];
                for (int i = 0; i < N; i++)
                    zeta[i] = HyperDual(0.0, i == k ? 1.0 : 0.0, i == l ? 1.0 : 0.0, 0.0);
                fn(zeta, out);
                for (int i = 0; i < NE; i++)
                {
                    HP[i][k][l] = out[i].e12;
                    HP[i][l][k] = out[i].e12;
                }
            }
        for (int i = 0; i < NE; i++)
        {
            double product[N][N];
            mat_mul(HP[i], Pz, product);
            mat_copy(product, HP[i]);
        }
        double JPJ[NE][NE];
        mat_sandwich(J, Pz, JPJ);
        for (int i = 0; i < NE; i++)
        {
            double s = 0.0;
            for (int k = 0; k < N; k++)
                s += HP[i][k][k];
            mean[i] = 0.5 * s;
        }
        for (int i = 0; i < NE; i++)
            for (int k = 0; k < NE; k++)
            {
                double s = 0.0;
                for (int r = 0; r < N; r++)
                    for (int c = 0; c < N; c++)
                        s += HP[i][r][c] * HP[k][c][r];
                P_out[i][k] = JPJ[i][k] + 0.5 * s;
            }
        mat_symmetrise(P_out);
    }

    template <int NE, int NV, class Fn>
    inline void map_propagate(const Fn &fn, const double (&P)[NE][NE], const double (&Qv)[NV][NV], double (&mean)[NE], double (&P_out)[NE][NE], Unscented)
    {
        const int N = NE + NV;
        const double lambda = Unscented::alpha_s() * Unscented::alpha_s() * (N + Unscented::kappa_s()) - N;
        double Pz[N][N], scaled[N][N], L[N][N];
        map_augmented_covariance(P, Qv, Pz);
        for (int i = 0; i < N; i++)
            for (int k = 0; k < N; k++)
                scaled[i][k] = (N + lambda) * Pz[i][k];
        if (!cholesky(scaled, L))
        {
            map_propagate<NE, NV>(fn, P, Qv, mean, P_out, FirstOrder());
            return;
        }
        const double w_m0 = lambda / (N + lambda);
        const double w_c0 = w_m0 + 1.0 - Unscented::alpha_s() * Unscented::alpha_s() + Unscented::beta_s();
        const double w_i = 1.0 / (2.0 * (N + lambda));
        double xi[2 * N + 1][NE];
        for (int p = 0; p < 2 * N + 1; p++)
        {
            double zeta[N];
            for (int i = 0; i < N; i++)
                zeta[i] = p == 0 ? 0.0 : (p <= N ? L[i][p - 1] : -L[i][p - 1 - N]);
            fn(zeta, xi[p]);
        }
        for (int i = 0; i < NE; i++)
        {
            double s = 0.0;
            for (int p = 0; p < 2 * N + 1; p++)
                s += (p == 0 ? w_m0 : w_i) * xi[p][i];
            mean[i] = s;
        }
        double mean2[NE];
        for (int i = 0; i < NE; i++)
            mean2[i] = 0.0;
        for (int p = 0; p < 2 * N + 1; p++)
        {
            double moved[NE];
            fn.reexpress(mean, xi[p], moved);
            for (int i = 0; i < NE; i++)
            {
                xi[p][i] = moved[i];
                mean2[i] += (p == 0 ? w_m0 : w_i) * moved[i];
            }
        }
        for (int i = 0; i < NE; i++)
            for (int k = 0; k < NE; k++)
            {
                double s = 0.0;
                for (int p = 0; p < 2 * N + 1; p++)
                    s += (p == 0 ? w_c0 : w_i) * (xi[p][i] - mean2[i]) * (xi[p][k] - mean2[k]);
                P_out[i][k] = s;
            }
        mat_symmetrise(P_out);
    }
}

#endif
