/*
 * Compile time policies for the jerk filters.
 *
 * Form selects the evaluation of the per axis transition and process noise matrices:
 *  JerkSmallAlphaT: (16) and (21) of [Ref1], the limit of small alpha T.
 *  JerkExact: (14), (15) and (20) of [Ref1].
 *
 * Order selects the propagation of the mean and covariance through a nonlinear model:
 *  FirstOrder, SecondOrder, Unscented.
 *
 * Diag selects whether the innovation quantities of the last update are stored:
 *  NoDiagnostics, WithDiagnostics.
 *
 * Developed for the Kaepek Project
 */

#ifndef KAEPEK_KALMAN_JERK_POLICIES_H
#define KAEPEK_KALMAN_JERK_POLICIES_H

#include "generated/jerk_block.hpp"

namespace kaepek
{
    struct JerkSmallAlphaT
    {
        static inline void fill(double dt, double alpha, double F[4][4], double Q[4][4])
        {
            (void)alpha;
            jerk_transition_small(dt, F);
            jerk_process_noise_small(dt, Q);
        }

        /**
         * @brief Value of alpha used in the drift of a continuous model, zero in the small alpha T limit.
         */
        static inline double drift_alpha(double alpha)
        {
            (void)alpha;
            return 0.0;
        }
    };

    struct JerkExact
    {
        static inline void fill(double dt, double alpha, double F[4][4], double Q[4][4])
        {
            jerk_transition_exact(dt, alpha, F);
            jerk_process_noise_exact(dt, alpha, Q);
        }

        static inline double drift_alpha(double alpha)
        {
            return alpha;
        }
    };

    struct FirstOrder
    {
    };

    struct SecondOrder
    {
    };

    struct Unscented
    {
        static inline double alpha_s() { return 1.0; }
        static inline double beta_s() { return 2.0; }
        static inline double kappa_s() { return 0.0; }
    };

    struct NoDiagnostics
    {
    };

    struct WithDiagnostics
    {
    };

    template <class D, int M>
    class DiagnosticsStore;

    template <int M>
    class DiagnosticsStore<NoDiagnostics, M>
    {
    protected:
        inline void store_diagnostics(const double (&y)[M], const double (&S)[M][M], const double (&S_inv)[M][M], double det_S)
        {
            (void)y;
            (void)S;
            (void)S_inv;
            (void)det_S;
        }
    };

    template <int M>
    class DiagnosticsStore<WithDiagnostics, M>
    {
    private:
        double innovation[M];
        double innovation_covariance[M][M];
        double innovation_covariance_inverse[M][M];
        double innovation_covariance_determinant;

    protected:
        inline void store_diagnostics(const double (&y)[M], const double (&S)[M][M], const double (&S_inv)[M][M], double det_S)
        {
            for (int i = 0; i < M; i++)
            {
                innovation[i] = y[i];
                for (int k = 0; k < M; k++)
                {
                    innovation_covariance[i][k] = S[i][k];
                    innovation_covariance_inverse[i][k] = S_inv[i][k];
                }
            }
            innovation_covariance_determinant = det_S;
        }

    public:
        DiagnosticsStore() : innovation_covariance_determinant(0.0)
        {
            for (int i = 0; i < M; i++)
            {
                innovation[i] = 0.0;
                for (int k = 0; k < M; k++)
                {
                    innovation_covariance[i][k] = 0.0;
                    innovation_covariance_inverse[i][k] = 0.0;
                }
            }
        }

        /**
         * @brief Innovation y of the last update, measured minus predicted measurement.
         */
        inline double (&get_innovation())[M] { return innovation; }

        /**
         * @brief Innovation covariance S of the last update.
         */
        inline double (&get_innovation_covariance())[M][M] { return innovation_covariance; }

        /**
         * @brief Inverse of the innovation covariance of the last update.
         */
        inline double (&get_innovation_covariance_inverse())[M][M] { return innovation_covariance_inverse; }

        /**
         * @brief Determinant of the innovation covariance of the last update.
         */
        inline double get_innovation_covariance_determinant() const { return innovation_covariance_determinant; }
    };
}

#endif
