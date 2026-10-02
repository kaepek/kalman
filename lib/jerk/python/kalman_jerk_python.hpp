/*
 * Runtime selection of the compile time policies for the Python bindings.
 * Each class holds one instantiation, with diagnostics, chosen by form and order codes:
 *  form: 0 JerkSmallAlphaT, 1 JerkExact
 *  order: 0 FirstOrder, 1 SecondOrder, 2 Unscented
 *
 * Developed for the Kaepek Project
 */

#ifndef KAEPEK_KALMAN_JERK_PYTHON_H
#define KAEPEK_KALMAN_JERK_PYTHON_H

#include "../kalman_jerk_cartesian.hpp"
#include "../kalman_jerk_2d_polar.hpp"
#include "../kalman_jerk_3d_spherical.hpp"
#include "../kalman_jerk_2d_azel.hpp"
#include "../kalman_jerk_1d_bearing_moving_sensor.hpp"
#include "../kalman_jerk_2d_azel_moving_sensor.hpp"

namespace kaepek
{
    namespace python
    {
        /**
         * @brief Access common to every filter: state, covariance and innovation quantities as flat arrays.
         */
        struct FilterInterface
        {
            virtual ~FilterInterface() {}
            virtual const double *kalman_vector() = 0;
            virtual const double *covariance_matrix() = 0;
            virtual const double *innovation() = 0;
            virtual const double *innovation_covariance() = 0;
            virtual const double *innovation_covariance_inverse() = 0;
            virtual double innovation_covariance_determinant() = 0;
        };

        template <int N>
        struct CartesianInterface : FilterInterface
        {
            virtual void step(double t, const double *x) = 0;
            virtual void step_covariance(double t, const double *x, const double *R) = 0;
            virtual const double *eular_vector() = 0;
        };

        template <int N, class Form>
        struct CartesianImpl : CartesianInterface<N>
        {
            KalmanJerkCartesian<N, Form, WithDiagnostics> f;
            CartesianImpl(double alpha, double res, double jerk, bool rel, double acc) : f(alpha, res, jerk, rel, acc) {}
            void step(double t, const double *x)
            {
                double v[N];
                for (int i = 0; i < N; i++)
                    v[i] = x[i];
                f.step(t, v);
            }
            void step_covariance(double t, const double *x, const double *R)
            {
                double v[N], C[N][N];
                for (int i = 0; i < N; i++)
                {
                    v[i] = x[i];
                    for (int k = 0; k < N; k++)
                        C[i][k] = R[N * i + k];
                }
                f.step(t, v, C);
            }
            const double *kalman_vector() { return f.get_kalman_vector(); }
            const double *covariance_matrix() { return &f.get_covariance_matrix()[0][0]; }
            const double *eular_vector() { return f.get_eular_vector(); }
            const double *innovation() { return f.get_innovation(); }
            const double *innovation_covariance() { return &f.get_innovation_covariance()[0][0]; }
            const double *innovation_covariance_inverse() { return &f.get_innovation_covariance_inverse()[0][0]; }
            double innovation_covariance_determinant() { return f.get_innovation_covariance_determinant(); }
        };

        template <int N>
        class CartesianPython
        {
        private:
            CartesianInterface<N> *impl;

        public:
            CartesianPython(double alpha, double x_resolution_error, double x_jerk_error, bool time_is_relative, double x_acceleration_error, int form)
            {
                if (form == 1)
                    impl = new CartesianImpl<N, JerkExact>(alpha, x_resolution_error, x_jerk_error, time_is_relative, x_acceleration_error);
                else
                    impl = new CartesianImpl<N, JerkSmallAlphaT>(alpha, x_resolution_error, x_jerk_error, time_is_relative, x_acceleration_error);
            }
            ~CartesianPython() { delete impl; }
            void step(double t, const double *x) { impl->step(t, x); }
            void step_covariance(double t, const double *x, const double *R) { impl->step_covariance(t, x, R); }
            const double *kalman_vector() { return impl->kalman_vector(); }
            const double *covariance_matrix() { return impl->covariance_matrix(); }
            const double *eular_vector() { return impl->eular_vector(); }
            const double *innovation() { return impl->innovation(); }
            const double *innovation_covariance() { return impl->innovation_covariance(); }
            const double *innovation_covariance_inverse() { return impl->innovation_covariance_inverse(); }
            double innovation_covariance_determinant() { return impl->innovation_covariance_determinant(); }
        };

        typedef CartesianPython<2> KalmanJerk2DPython;
        typedef CartesianPython<3> KalmanJerk3DPython;

        template <class Filter, int N>
        struct DelegatedDiagnostics : FilterInterface
        {
            Filter f;
            template <typename... A>
            DelegatedDiagnostics(A... args) : f(args...) {}
            const double *kalman_vector() { return f.get_kalman_vector(); }
            const double *covariance_matrix() { return &f.get_covariance_matrix()[0][0]; }
            const double *innovation() { return f.get_cartesian().get_innovation(); }
            const double *innovation_covariance() { return &f.get_cartesian().get_innovation_covariance()[0][0]; }
            const double *innovation_covariance_inverse() { return &f.get_cartesian().get_innovation_covariance_inverse()[0][0]; }
            double innovation_covariance_determinant() { return f.get_cartesian().get_innovation_covariance_determinant(); }
        };

        struct PolarInterface : FilterInterface
        {
            virtual void step(double t, double range, double angle) = 0;
        };

        template <class Form>
        struct PolarImpl : PolarInterface
        {
            DelegatedDiagnostics<KalmanJerk2DPolar<Form, WithDiagnostics>, 2> d;
            PolarImpl(double alpha, double re, double ae, double je, bool rel, double acc) : d(alpha, re, ae, je, rel, acc) {}
            void step(double t, double range, double angle) { d.f.step(t, range, angle); }
            const double *kalman_vector() { return d.kalman_vector(); }
            const double *covariance_matrix() { return d.covariance_matrix(); }
            const double *innovation() { return d.innovation(); }
            const double *innovation_covariance() { return d.innovation_covariance(); }
            const double *innovation_covariance_inverse() { return d.innovation_covariance_inverse(); }
            double innovation_covariance_determinant() { return d.innovation_covariance_determinant(); }
        };

        class KalmanJerk2DPolarPython
        {
        private:
            PolarInterface *impl;

        public:
            KalmanJerk2DPolarPython(double alpha, double range_error, double angle_error, double jerk_error, bool time_is_relative, double acceleration_error, int form)
            {
                if (form == 1)
                    impl = new PolarImpl<JerkExact>(alpha, range_error, angle_error, jerk_error, time_is_relative, acceleration_error);
                else
                    impl = new PolarImpl<JerkSmallAlphaT>(alpha, range_error, angle_error, jerk_error, time_is_relative, acceleration_error);
            }
            ~KalmanJerk2DPolarPython() { delete impl; }
            void step(double t, double range, double angle) { impl->step(t, range, angle); }
            const double *kalman_vector() { return impl->kalman_vector(); }
            const double *covariance_matrix() { return impl->covariance_matrix(); }
            const double *innovation() { return impl->innovation(); }
            const double *innovation_covariance() { return impl->innovation_covariance(); }
            const double *innovation_covariance_inverse() { return impl->innovation_covariance_inverse(); }
            double innovation_covariance_determinant() { return impl->innovation_covariance_determinant(); }
        };

        struct SphericalInterface : FilterInterface
        {
            virtual void step(double t, double range, double azimuth, double elevation) = 0;
        };

        template <class Form>
        struct SphericalImpl : SphericalInterface
        {
            DelegatedDiagnostics<KalmanJerk3DSpherical<Form, WithDiagnostics>, 3> d;
            SphericalImpl(double alpha, double re, double ae, double ee, double je, bool rel, double acc) : d(alpha, re, ae, ee, je, rel, acc) {}
            void step(double t, double range, double azimuth, double elevation) { d.f.step(t, range, azimuth, elevation); }
            const double *kalman_vector() { return d.kalman_vector(); }
            const double *covariance_matrix() { return d.covariance_matrix(); }
            const double *innovation() { return d.innovation(); }
            const double *innovation_covariance() { return d.innovation_covariance(); }
            const double *innovation_covariance_inverse() { return d.innovation_covariance_inverse(); }
            double innovation_covariance_determinant() { return d.innovation_covariance_determinant(); }
        };

        class KalmanJerk3DSphericalPython
        {
        private:
            SphericalInterface *impl;

        public:
            KalmanJerk3DSphericalPython(double alpha, double range_error, double azimuth_error, double elevation_error, double jerk_error, bool time_is_relative, double acceleration_error, int form)
            {
                if (form == 1)
                    impl = new SphericalImpl<JerkExact>(alpha, range_error, azimuth_error, elevation_error, jerk_error, time_is_relative, acceleration_error);
                else
                    impl = new SphericalImpl<JerkSmallAlphaT>(alpha, range_error, azimuth_error, elevation_error, jerk_error, time_is_relative, acceleration_error);
            }
            ~KalmanJerk3DSphericalPython() { delete impl; }
            void step(double t, double range, double azimuth, double elevation) { impl->step(t, range, azimuth, elevation); }
            const double *kalman_vector() { return impl->kalman_vector(); }
            const double *covariance_matrix() { return impl->covariance_matrix(); }
            const double *innovation() { return impl->innovation(); }
            const double *innovation_covariance() { return impl->innovation_covariance(); }
            const double *innovation_covariance_inverse() { return impl->innovation_covariance_inverse(); }
            double innovation_covariance_determinant() { return impl->innovation_covariance_determinant(); }
        };

        /**
         * @brief Diagnostics access for filters that hold their own diagnostics.
         */
        template <class Filter>
        struct OwnDiagnostics : FilterInterface
        {
            Filter f;
            template <typename... A>
            OwnDiagnostics(A... args) : f(args...) {}
            const double *kalman_vector() { return f.get_kalman_vector(); }
            const double *covariance_matrix() { return &f.get_covariance_matrix()[0][0]; }
            const double *innovation() { return f.get_innovation(); }
            const double *innovation_covariance() { return &f.get_innovation_covariance()[0][0]; }
            const double *innovation_covariance_inverse() { return &f.get_innovation_covariance_inverse()[0][0]; }
            double innovation_covariance_determinant() { return f.get_innovation_covariance_determinant(); }
        };

        struct AzElInterface : FilterInterface
        {
            virtual void step(double t, double azimuth, double elevation) = 0;
            virtual void step_covariance(double t, double azimuth, double elevation, const double *R) = 0;
            virtual const double *state_vector() = 0;
            virtual const double *basis() = 0;
            virtual const double *basis_covariance_matrix() = 0;
            virtual bool azimuth_defined() = 0;
        };

        template <class Form, class Order>
        struct AzElImpl : AzElInterface
        {
            OwnDiagnostics<KalmanJerk2DAzEl<Form, Order, WithDiagnostics> > d;
            AzElImpl(double alpha, double de, double je, bool rel, double acc, int substeps) : d(alpha, de, je, rel, acc, substeps) {}
            void step(double t, double azimuth, double elevation) { d.f.step(t, azimuth, elevation); }
            void step_covariance(double t, double azimuth, double elevation, const double *R)
            {
                double C[2][2] = {{R[0], R[1]}, {R[2], R[3]}};
                d.f.step(t, azimuth, elevation, C);
            }
            const double *state_vector() { return d.f.get_state_vector(); }
            const double *basis() { return d.f.get_basis(); }
            const double *basis_covariance_matrix() { return &d.f.get_basis_covariance_matrix()[0][0]; }
            bool azimuth_defined() { return d.f.azimuth_defined(); }
            const double *kalman_vector() { return d.kalman_vector(); }
            const double *covariance_matrix() { return d.covariance_matrix(); }
            const double *innovation() { return d.innovation(); }
            const double *innovation_covariance() { return d.innovation_covariance(); }
            const double *innovation_covariance_inverse() { return d.innovation_covariance_inverse(); }
            double innovation_covariance_determinant() { return d.innovation_covariance_determinant(); }
        };

        template <template <class, class> class Impl, class Interface, typename... A>
        Interface *select_policies(int form, int order, A... args)
        {
            if (form == 1)
            {
                if (order == 1)
                    return new Impl<JerkExact, SecondOrder>(args...);
                if (order == 2)
                    return new Impl<JerkExact, Unscented>(args...);
                return new Impl<JerkExact, FirstOrder>(args...);
            }
            if (order == 1)
                return new Impl<JerkSmallAlphaT, SecondOrder>(args...);
            if (order == 2)
                return new Impl<JerkSmallAlphaT, Unscented>(args...);
            return new Impl<JerkSmallAlphaT, FirstOrder>(args...);
        }

        class KalmanJerk2DAzElPython
        {
        private:
            AzElInterface *impl;

        public:
            KalmanJerk2DAzElPython(double alpha, double direction_error, double jerk_error, bool time_is_relative, double acceleration_error, int substeps, int form, int order)
            {
                impl = select_policies<AzElImpl, AzElInterface>(form, order, alpha, direction_error, jerk_error, time_is_relative, acceleration_error, substeps);
            }
            ~KalmanJerk2DAzElPython() { delete impl; }
            void step(double t, double azimuth, double elevation) { impl->step(t, azimuth, elevation); }
            void step_covariance(double t, double azimuth, double elevation, const double *R) { impl->step_covariance(t, azimuth, elevation, R); }
            const double *kalman_vector() { return impl->kalman_vector(); }
            const double *covariance_matrix() { return impl->covariance_matrix(); }
            const double *state_vector() { return impl->state_vector(); }
            const double *basis() { return impl->basis(); }
            const double *basis_covariance_matrix() { return impl->basis_covariance_matrix(); }
            bool azimuth_defined() { return impl->azimuth_defined(); }
            const double *innovation() { return impl->innovation(); }
            const double *innovation_covariance() { return impl->innovation_covariance(); }
            const double *innovation_covariance_inverse() { return impl->innovation_covariance_inverse(); }
            double innovation_covariance_determinant() { return impl->innovation_covariance_determinant(); }
        };

        struct BearingMovingSensorInterface : FilterInterface
        {
            virtual void step(double t, double bearing, const double *sensor) = 0;
            virtual double range() = 0;
        };

        template <class Form, class Order>
        struct BearingMovingSensorImpl : BearingMovingSensorInterface
        {
            OwnDiagnostics<KalmanJerk1DBearingMovingSensor<Form, Order, WithDiagnostics> > d;
            BearingMovingSensorImpl(double alpha, double be, double je, bool rel, double rmin, double rmax, double l1, double l2, double l3, double acc)
                : d(alpha, be, je, rel, rmin, rmax, l1, l2, l3, acc) {}
            void step(double t, double bearing, const double *sensor)
            {
                double s[8];
                for (int i = 0; i < 8; i++)
                    s[i] = sensor[i];
                d.f.step(t, bearing, s);
            }
            double range() { return d.f.get_range(); }
            const double *kalman_vector() { return d.kalman_vector(); }
            const double *covariance_matrix() { return d.covariance_matrix(); }
            const double *innovation() { return d.innovation(); }
            const double *innovation_covariance() { return d.innovation_covariance(); }
            const double *innovation_covariance_inverse() { return d.innovation_covariance_inverse(); }
            double innovation_covariance_determinant() { return d.innovation_covariance_determinant(); }
        };

        class KalmanJerk1DBearingMovingSensorPython
        {
        private:
            BearingMovingSensorInterface *impl;

        public:
            KalmanJerk1DBearingMovingSensorPython(double alpha, double bearing_error, double jerk_error, bool time_is_relative, double range_min, double range_max,
                                                  double log_range_rate_error, double log_range_acceleration_error, double log_range_jerk_error,
                                                  double acceleration_error, int form, int order)
            {
                impl = select_policies<BearingMovingSensorImpl, BearingMovingSensorInterface>(form, order, alpha, bearing_error, jerk_error, time_is_relative, range_min, range_max,
                                                                                                log_range_rate_error, log_range_acceleration_error, log_range_jerk_error, acceleration_error);
            }
            ~KalmanJerk1DBearingMovingSensorPython() { delete impl; }
            void step(double t, double bearing, const double *sensor) { impl->step(t, bearing, sensor); }
            double range() { return impl->range(); }
            const double *kalman_vector() { return impl->kalman_vector(); }
            const double *covariance_matrix() { return impl->covariance_matrix(); }
            const double *innovation() { return impl->innovation(); }
            const double *innovation_covariance() { return impl->innovation_covariance(); }
            const double *innovation_covariance_inverse() { return impl->innovation_covariance_inverse(); }
            double innovation_covariance_determinant() { return impl->innovation_covariance_determinant(); }
        };

        struct AzElMovingSensorInterface : FilterInterface
        {
            virtual void step(double t, double azimuth, double elevation, const double *sensor) = 0;
            virtual void step_covariance(double t, double azimuth, double elevation, const double *sensor, const double *R) = 0;
            virtual const double *state_vector() = 0;
            virtual const double *basis() = 0;
            virtual double range() = 0;
        };

        template <class Form, class Order>
        struct AzElMovingSensorImpl : AzElMovingSensorInterface
        {
            OwnDiagnostics<KalmanJerk2DAzElMovingSensor<Form, Order, WithDiagnostics> > d;
            AzElMovingSensorImpl(double alpha, double de, double je, bool rel, double rmin, double rmax, double l1, double l2, double l3, double acc)
                : d(alpha, de, je, rel, rmin, rmax, l1, l2, l3, acc) {}
            void step(double t, double azimuth, double elevation, const double *sensor)
            {
                double s[12];
                for (int i = 0; i < 12; i++)
                    s[i] = sensor[i];
                d.f.step(t, azimuth, elevation, s);
            }
            void step_covariance(double t, double azimuth, double elevation, const double *sensor, const double *R)
            {
                double s[12];
                for (int i = 0; i < 12; i++)
                    s[i] = sensor[i];
                double C[2][2] = {{R[0], R[1]}, {R[2], R[3]}};
                d.f.step(t, azimuth, elevation, s, C);
            }
            const double *state_vector() { return d.f.get_state_vector(); }
            const double *basis() { return d.f.get_basis(); }
            double range() { return d.f.get_range(); }
            const double *kalman_vector() { return d.kalman_vector(); }
            const double *covariance_matrix() { return d.covariance_matrix(); }
            const double *innovation() { return d.innovation(); }
            const double *innovation_covariance() { return d.innovation_covariance(); }
            const double *innovation_covariance_inverse() { return d.innovation_covariance_inverse(); }
            double innovation_covariance_determinant() { return d.innovation_covariance_determinant(); }
        };

        class KalmanJerk2DAzElMovingSensorPython
        {
        private:
            AzElMovingSensorInterface *impl;

        public:
            KalmanJerk2DAzElMovingSensorPython(double alpha, double direction_error, double jerk_error, bool time_is_relative, double range_min, double range_max,
                                               double log_range_rate_error, double log_range_acceleration_error, double log_range_jerk_error,
                                               double acceleration_error, int form, int order)
            {
                impl = select_policies<AzElMovingSensorImpl, AzElMovingSensorInterface>(form, order, alpha, direction_error, jerk_error, time_is_relative, range_min, range_max,
                                                                                         log_range_rate_error, log_range_acceleration_error, log_range_jerk_error, acceleration_error);
            }
            ~KalmanJerk2DAzElMovingSensorPython() { delete impl; }
            void step(double t, double azimuth, double elevation, const double *sensor) { impl->step(t, azimuth, elevation, sensor); }
            void step_covariance(double t, double azimuth, double elevation, const double *sensor, const double *R) { impl->step_covariance(t, azimuth, elevation, sensor, R); }
            const double *state_vector() { return impl->state_vector(); }
            const double *basis() { return impl->basis(); }
            double range() { return impl->range(); }
            const double *kalman_vector() { return impl->kalman_vector(); }
            const double *covariance_matrix() { return impl->covariance_matrix(); }
            const double *innovation() { return impl->innovation(); }
            const double *innovation_covariance() { return impl->innovation_covariance(); }
            const double *innovation_covariance_inverse() { return impl->innovation_covariance_inverse(); }
            double innovation_covariance_determinant() { return impl->innovation_covariance_determinant(); }
        };
    }
}

#endif
