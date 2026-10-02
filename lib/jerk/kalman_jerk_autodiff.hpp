/*
 * Forward mode automatic differentiation for the jerk filters.
 *
 * Dual<N>: value and gradient with respect to N inputs, giving a Jacobian in one evaluation.
 * HyperDual: value with two first order parts e1, e2 and the mixed part e12, giving one second derivative
 * d2f/(dx_k dx_l) per evaluation.
 *
 * Developed for the Kaepek Project
 */

#ifndef KAEPEK_KALMAN_JERK_AUTODIFF_H
#define KAEPEK_KALMAN_JERK_AUTODIFF_H

#include <math.h>

namespace kaepek
{
    using ::atan2;
    using ::cos;
    using ::sin;
    using ::sqrt;

    inline double value_of(double x) { return x; }

    template <int N>
    struct Dual
    {
        double v;
        double d[N];

        Dual() : v(0.0)
        {
            for (int i = 0; i < N; i++)
                d[i] = 0.0;
        }

        Dual(double value) : v(value)
        {
            for (int i = 0; i < N; i++)
                d[i] = 0.0;
        }

        static Dual variable(double value, int index)
        {
            Dual r(value);
            r.d[index] = 1.0;
            return r;
        }

        friend Dual operator+(const Dual &a, const Dual &b)
        {
            Dual r(a.v + b.v);
            for (int i = 0; i < N; i++)
                r.d[i] = a.d[i] + b.d[i];
            return r;
        }

        friend Dual operator-(const Dual &a, const Dual &b)
        {
            Dual r(a.v - b.v);
            for (int i = 0; i < N; i++)
                r.d[i] = a.d[i] - b.d[i];
            return r;
        }

        friend Dual operator-(const Dual &a)
        {
            Dual r(-a.v);
            for (int i = 0; i < N; i++)
                r.d[i] = -a.d[i];
            return r;
        }

        friend Dual operator*(const Dual &a, const Dual &b)
        {
            Dual r(a.v * b.v);
            for (int i = 0; i < N; i++)
                r.d[i] = a.d[i] * b.v + a.v * b.d[i];
            return r;
        }

        friend Dual operator/(const Dual &a, const Dual &b)
        {
            double inv = 1.0 / b.v;
            Dual r(a.v * inv);
            for (int i = 0; i < N; i++)
                r.d[i] = (a.d[i] - r.v * b.d[i]) * inv;
            return r;
        }

        Dual &operator+=(const Dual &b) { return *this = *this + b; }
        Dual &operator-=(const Dual &b) { return *this = *this - b; }
        Dual &operator*=(const Dual &b) { return *this = *this * b; }

        friend Dual sqrt(const Dual &a)
        {
            double s = ::sqrt(a.v);
            Dual r(s);
            double g = 0.5 / s;
            for (int i = 0; i < N; i++)
                r.d[i] = g * a.d[i];
            return r;
        }

        friend Dual sin(const Dual &a)
        {
            Dual r(::sin(a.v));
            double g = ::cos(a.v);
            for (int i = 0; i < N; i++)
                r.d[i] = g * a.d[i];
            return r;
        }

        friend Dual cos(const Dual &a)
        {
            Dual r(::cos(a.v));
            double g = -::sin(a.v);
            for (int i = 0; i < N; i++)
                r.d[i] = g * a.d[i];
            return r;
        }

        friend Dual atan2(const Dual &y, const Dual &x)
        {
            Dual r(::atan2(y.v, x.v));
            double inv = 1.0 / (x.v * x.v + y.v * y.v);
            for (int i = 0; i < N; i++)
                r.d[i] = (x.v * y.d[i] - y.v * x.d[i]) * inv;
            return r;
        }

        friend double value_of(const Dual &a) { return a.v; }
    };

    struct HyperDual
    {
        double v;
        double e1;
        double e2;
        double e12;

        HyperDual() : v(0.0), e1(0.0), e2(0.0), e12(0.0) {}
        HyperDual(double value) : v(value), e1(0.0), e2(0.0), e12(0.0) {}
        HyperDual(double value, double d1, double d2, double d12) : v(value), e1(d1), e2(d2), e12(d12) {}

        /**
         * @brief Applies f with f(v), f'(v) and f''(v) given.
         */
        static HyperDual chain(const HyperDual &a, double f, double df, double ddf)
        {
            return HyperDual(f, df * a.e1, df * a.e2, df * a.e12 + ddf * a.e1 * a.e2);
        }

        friend HyperDual operator+(const HyperDual &a, const HyperDual &b) { return HyperDual(a.v + b.v, a.e1 + b.e1, a.e2 + b.e2, a.e12 + b.e12); }
        friend HyperDual operator-(const HyperDual &a, const HyperDual &b) { return HyperDual(a.v - b.v, a.e1 - b.e1, a.e2 - b.e2, a.e12 - b.e12); }
        friend HyperDual operator-(const HyperDual &a) { return HyperDual(-a.v, -a.e1, -a.e2, -a.e12); }

        friend HyperDual operator*(const HyperDual &a, const HyperDual &b)
        {
            return HyperDual(a.v * b.v, a.e1 * b.v + a.v * b.e1, a.e2 * b.v + a.v * b.e2, a.e12 * b.v + a.e1 * b.e2 + a.e2 * b.e1 + a.v * b.e12);
        }

        friend HyperDual operator/(const HyperDual &a, const HyperDual &b)
        {
            double inv = 1.0 / b.v;
            return a * chain(b, inv, -inv * inv, 2.0 * inv * inv * inv);
        }

        HyperDual &operator+=(const HyperDual &b) { return *this = *this + b; }
        HyperDual &operator-=(const HyperDual &b) { return *this = *this - b; }
        HyperDual &operator*=(const HyperDual &b) { return *this = *this * b; }

        friend HyperDual sqrt(const HyperDual &a)
        {
            double s = ::sqrt(a.v);
            return chain(a, s, 0.5 / s, -0.25 / (s * a.v));
        }

        friend HyperDual sin(const HyperDual &a)
        {
            double s = ::sin(a.v);
            return chain(a, s, ::cos(a.v), -s);
        }

        friend HyperDual cos(const HyperDual &a)
        {
            double c = ::cos(a.v);
            return chain(a, c, -::sin(a.v), -c);
        }

        friend HyperDual atan2(const HyperDual &y, const HyperDual &x)
        {
            double r2 = x.v * x.v + y.v * y.v;
            double inv = 1.0 / r2;
            double fy = x.v * inv;
            double fx = -y.v * inv;
            double inv2 = inv * inv;
            double fyy = -2.0 * x.v * y.v * inv2;
            double fxx = 2.0 * x.v * y.v * inv2;
            double fxy = (y.v * y.v - x.v * x.v) * inv2;
            return HyperDual(::atan2(y.v, x.v),
                             fy * y.e1 + fx * x.e1,
                             fy * y.e2 + fx * x.e2,
                             fy * y.e12 + fx * x.e12 + fyy * y.e1 * y.e2 + fxx * x.e1 * x.e2 + fxy * (x.e1 * y.e2 + y.e1 * x.e2));
        }

        friend double value_of(const HyperDual &a) { return a.v; }
    };
}

#endif
