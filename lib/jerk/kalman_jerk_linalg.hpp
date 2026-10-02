/*
 * Fixed size linear algebra used by the jerk filters.
 *
 * Developed for the Kaepek Project
 */

#ifndef KAEPEK_KALMAN_JERK_LINALG_H
#define KAEPEK_KALMAN_JERK_LINALG_H

#include <math.h>

namespace kaepek
{
    template <int R, int C>
    inline void mat_zero(double (&A)[R][C])
    {
        for (int i = 0; i < R; i++)
            for (int k = 0; k < C; k++)
                A[i][k] = 0.0;
    }

    template <int N>
    inline void mat_identity(double (&A)[N][N])
    {
        mat_zero(A);
        for (int i = 0; i < N; i++)
            A[i][i] = 1.0;
    }

    template <int R, int C>
    inline void mat_copy(const double (&A)[R][C], double (&B)[R][C])
    {
        for (int i = 0; i < R; i++)
            for (int k = 0; k < C; k++)
                B[i][k] = A[i][k];
    }

    /**
     * @brief C = A * B
     */
    template <int R, int K, int C>
    inline void mat_mul(const double (&A)[R][K], const double (&B)[K][C], double (&C_)[R][C])
    {
        for (int i = 0; i < R; i++)
            for (int k = 0; k < C; k++)
            {
                double s = 0.0;
                for (int m = 0; m < K; m++)
                    s += A[i][m] * B[m][k];
                C_[i][k] = s;
            }
    }

    /**
     * @brief C = A * B^T
     */
    template <int R, int K, int C>
    inline void mat_mul_bt(const double (&A)[R][K], const double (&B)[C][K], double (&C_)[R][C])
    {
        for (int i = 0; i < R; i++)
            for (int k = 0; k < C; k++)
            {
                double s = 0.0;
                for (int m = 0; m < K; m++)
                    s += A[i][m] * B[k][m];
                C_[i][k] = s;
            }
    }

    /**
     * @brief P_out = F * P * F^T
     */
    template <int R, int N>
    inline void mat_sandwich(const double (&F)[R][N], const double (&P)[N][N], double (&P_out)[R][R])
    {
        double FP[R][N];
        mat_mul(F, P, FP);
        mat_mul_bt(FP, F, P_out);
    }

    template <int N>
    inline void mat_symmetrise(double (&A)[N][N])
    {
        for (int i = 0; i < N; i++)
            for (int k = i + 1; k < N; k++)
            {
                double m = 0.5 * (A[i][k] + A[k][i]);
                A[i][k] = m;
                A[k][i] = m;
            }
    }

    /**
     * @brief Lower triangular L with A = L L^T. Returns false when A is not positive definite.
     */
    template <int N>
    inline bool cholesky(const double (&A)[N][N], double (&L)[N][N])
    {
        mat_zero(L);
        for (int i = 0; i < N; i++)
        {
            for (int k = 0; k <= i; k++)
            {
                double s = A[i][k];
                for (int m = 0; m < k; m++)
                    s -= L[i][m] * L[k][m];
                if (i == k)
                {
                    if (s <= 0.0)
                        return false;
                    L[i][i] = sqrt(s);
                }
                else
                {
                    L[i][k] = s / L[k][k];
                }
            }
        }
        return true;
    }

    /**
     * @brief Inverse and determinant of a symmetric positive definite matrix through its Cholesky factor.
     */
    template <int N>
    inline bool spd_inverse(const double (&A)[N][N], double (&A_inv)[N][N], double &det)
    {
        double L[N][N];
        if (!cholesky(A, L))
            return false;
        det = 1.0;
        for (int i = 0; i < N; i++)
            det *= L[i][i] * L[i][i];
        double L_inv[N][N];
        mat_zero(L_inv);
        for (int i = 0; i < N; i++)
        {
            L_inv[i][i] = 1.0 / L[i][i];
            for (int k = 0; k < i; k++)
            {
                double s = 0.0;
                for (int m = k; m < i; m++)
                    s -= L[i][m] * L_inv[m][k];
                L_inv[i][k] = s / L[i][i];
            }
        }
        for (int i = 0; i < N; i++)
            for (int k = 0; k < N; k++)
            {
                double s = 0.0;
                for (int m = (i > k ? i : k); m < N; m++)
                    s += L_inv[m][i] * L_inv[m][k];
                A_inv[i][k] = s;
            }
        return true;
    }

    /**
     * @brief Kalman update of covariance P with a measurement of state components idx.
     *  y is the innovation and R the measurement covariance; correction receives K y.
     *  Returns false when S is not positive definite.
     */
    template <int N, int M>
    inline bool kalman_update(double (&P)[N][N], const int (&idx)[M], const double (&y)[M], const double (&R)[M][M], double (&S)[M][M], double (&S_inv)[M][M], double &det_S, double (&correction)[N])
    {
        for (int i = 0; i < M; i++)
            for (int k = 0; k < M; k++)
                S[i][k] = P[idx[i]][idx[k]] + R[i][k];
        if (!spd_inverse(S, S_inv, det_S))
            return false;
        double K[N][M];
        for (int i = 0; i < N; i++)
            for (int k = 0; k < M; k++)
            {
                double s = 0.0;
                for (int m = 0; m < M; m++)
                    s += P[i][idx[m]] * S_inv[m][k];
                K[i][k] = s;
            }
        for (int i = 0; i < N; i++)
        {
            double s = 0.0;
            for (int k = 0; k < M; k++)
                s += K[i][k] * y[k];
            correction[i] = s;
        }
        double P_new[N][N];
        for (int i = 0; i < N; i++)
            for (int k = 0; k < N; k++)
            {
                double s = P[i][k];
                for (int m = 0; m < M; m++)
                    s -= K[i][m] * P[idx[m]][k];
                P_new[i][k] = s;
            }
        mat_symmetrise(P_new);
        mat_copy(P_new, P);
        return true;
    }
}

#endif
