/*
 * Unit sphere geometry for the direction filters.
 *
 * A direction state is the unit vector u with tangent vectors w, a, j stored as x[12] = [u, w, a, j],
 * and an orthonormal tangent basis b[6] = [b1, b2]. Errors are ordered by axis
 * [v_1, dw_1, da_1, dj_1, v_2, dw_2, da_2, dj_2] with components taken in the basis.
 *
 * Developed for the Kaepek Project
 */

#ifndef KAEPEK_KALMAN_JERK_SPHERE_H
#define KAEPEK_KALMAN_JERK_SPHERE_H

#include <math.h>
#include "kalman_jerk_autodiff.hpp"

namespace kaepek
{
    template <typename S>
    inline S dot3(const S *a, const S *b)
    {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    template <typename S>
    inline void cross3(const S *a, const S *b, S *c)
    {
        c[0] = a[1] * b[2] - a[2] * b[1];
        c[1] = a[2] * b[0] - a[0] * b[2];
        c[2] = a[0] * b[1] - a[1] * b[0];
    }

    /**
     * @brief Rotates x by the rotation vector r (angle |r| about r), smooth at r = 0.
     */
    template <typename S>
    inline void rotate3(const S *r, const S *x, S *out)
    {
        S t2 = dot3(r, r);
        S s, c;
        if (value_of(t2) < 1e-8)
        {
            s = S(1.0) - t2 / 6.0 + t2 * t2 / 120.0;
            c = S(0.5) - t2 / 24.0 + t2 * t2 / 720.0;
        }
        else
        {
            S t = sqrt(t2);
            s = sin(t) / t;
            c = (S(1.0) - cos(t)) / t2;
        }
        S rx[3], rrx[3];
        cross3(r, x, rx);
        cross3(r, rx, rrx);
        for (int i = 0; i < 3; i++)
            out[i] = x[i] + s * rx[i] + c * rrx[i];
    }

    /**
     * @brief Tangent vector at u pointing along the great circle to m with length the angle between them.
     */
    template <typename S>
    inline void log_map(const S *u, const S *m, S *v)
    {
        S cd = dot3(u, m);
        S tau[3];
        for (int i = 0; i < 3; i++)
            tau[i] = m[i] - cd * u[i];
        S s2 = dot3(tau, tau);
        S factor;
        if (value_of(s2) < 1e-12 && value_of(cd) > 0.0)
            factor = S(1.0) + s2 / 6.0 + 3.0 * s2 * s2 / 40.0;
        else
        {
            S s = sqrt(s2);
            factor = atan2(s, cd) / s;
        }
        for (int i = 0; i < 3; i++)
            v[i] = factor * tau[i];
    }

    /**
     * @brief Rotation vector of the parallel transport along the great circle from u in direction v.
     */
    template <typename S>
    inline void transport_vector(const S *u, const S *v, S *r)
    {
        cross3(u, v, r);
    }

    /**
     * @brief Orthonormal tangent basis at c built from the coordinate axis least aligned with c.
     */
    template <typename S>
    inline void tangent_basis(const S *c, S *b)
    {
        int k = 0;
        double m = fabs(value_of(c[0]));
        for (int i = 1; i < 3; i++)
            if (fabs(value_of(c[i])) < m)
            {
                m = fabs(value_of(c[i]));
                k = i;
            }
        S e[3] = {S(0.0), S(0.0), S(0.0)};
        e[k] = S(1.0);
        S ce = c[k];
        S b1[3];
        for (int i = 0; i < 3; i++)
            b1[i] = e[i] - ce * c[i];
        S n = sqrt(dot3(b1, b1));
        for (int i = 0; i < 3; i++)
            b1[i] = b1[i] / n;
        S b2[3];
        cross3(c, b1, b2);
        for (int i = 0; i < 3; i++)
        {
            b[i] = b1[i];
            b[3 + i] = b2[i];
        }
    }

    template <typename S>
    inline void unit_from_azel(const S &az, const S &el, S *u)
    {
        u[0] = cos(el) * cos(az);
        u[1] = cos(el) * sin(az);
        u[2] = sin(el);
    }

    inline void azel_from_unit(const double *u, double &az, double &el)
    {
        az = atan2(u[1], u[0]);
        el = atan2(u[2], sqrt(u[0] * u[0] + u[1] * u[1]));
    }

    /**
     * @brief Tangent vectors along increasing azimuth and elevation at u. Returns false at the zenith and nadir.
     */
    inline bool azel_frame(const double *u, double *e_a, double *e_e)
    {
        double rho = sqrt(u[0] * u[0] + u[1] * u[1]);
        if (rho < 1e-12)
            return false;
        e_a[0] = -u[1] / rho;
        e_a[1] = u[0] / rho;
        e_a[2] = 0.0;
        e_e[0] = -u[2] * u[0] / rho;
        e_e[1] = -u[2] * u[1] / rho;
        e_e[2] = rho;
        return true;
    }

    /**
     * @brief Tangent vector with components c_1, c_2 in the basis b.
     */
    template <typename S, typename T>
    inline void basis_vector(const T *b, const S &c1, const S &c2, S *out)
    {
        for (int i = 0; i < 3; i++)
            out[i] = c1 * b[i] + c2 * b[3 + i];
    }

    /**
     * @brief Direction state obtained from the estimate (x, b) and the error d, with the basis carried along.
     */
    template <typename S>
    inline void direction_retract(const double *x, const double *b, const S *d, S *x_out, S *b_out)
    {
        S u[3], v[3], r[3], t[3];
        for (int i = 0; i < 3; i++)
            u[i] = S(x[i]);
        basis_vector(b, d[0], d[4], v);
        transport_vector(u, v, r);
        rotate3(r, u, x_out);
        for (int k = 1; k < 4; k++)
        {
            S dk[3];
            basis_vector(b, d[k], d[4 + k], dk);
            for (int i = 0; i < 3; i++)
                t[i] = S(x[3 * k + i]) + dk[i];
            rotate3(r, t, x_out + 3 * k);
        }
        for (int c = 0; c < 2; c++)
        {
            S bc[3];
            for (int i = 0; i < 3; i++)
                bc[i] = S(b[3 * c + i]);
            rotate3(r, bc, b_out + 3 * c);
        }
    }

    /**
     * @brief Error d of the direction state x_s relative to the estimate (x, b).
     */
    template <typename S>
    inline void direction_inverse_retract(const double *x, const double *b, const S *x_s, S *d)
    {
        S u[3], v[3], r[3], neg_r[3];
        for (int i = 0; i < 3; i++)
            u[i] = S(x[i]);
        log_map(u, x_s, v);
        transport_vector(u, v, r);
        for (int i = 0; i < 3; i++)
            neg_r[i] = -r[i];
        S comp[4][3];
        for (int i = 0; i < 3; i++)
            comp[0][i] = v[i];
        for (int k = 1; k < 4; k++)
        {
            S back[3];
            rotate3(neg_r, x_s + 3 * k, back);
            for (int i = 0; i < 3; i++)
                comp[k][i] = back[i] - S(x[3 * k + i]);
        }
        for (int k = 0; k < 4; k++)
        {
            d[k] = comp[k][0] * b[0] + comp[k][1] * b[1] + comp[k][2] * b[2];
            d[4 + k] = comp[k][0] * b[3] + comp[k][1] * b[4] + comp[k][2] * b[5];
        }
    }

    /**
     * @brief Basis carried from u0 to u1 by the parallel transport along the great circle between them.
     */
    inline void transport_basis(const double *u0, const double *u1, const double *b, double *b_out)
    {
        double v[3], r[3];
        log_map(u0, u1, v);
        transport_vector(u0, v, r);
        rotate3(r, b, b_out);
        rotate3(r, b + 3, b_out + 3);
    }

    /**
     * @brief Normalises u and removes the normal parts of w, a, j and of the basis.
     */
    inline void direction_normalise(double *x, double *b)
    {
        double n = sqrt(dot3(x, x));
        for (int i = 0; i < 3; i++)
            x[i] /= n;
        for (int k = 1; k < 4; k++)
        {
            double p = dot3(x, x + 3 * k);
            for (int i = 0; i < 3; i++)
                x[3 * k + i] -= p * x[i];
        }
        double p = dot3(x, b);
        for (int i = 0; i < 3; i++)
            b[i] -= p * x[i];
        double nb = sqrt(dot3(b, b));
        for (int i = 0; i < 3; i++)
            b[i] /= nb;
        cross3(x, b, b + 3);
    }
}

#endif
