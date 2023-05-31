/*
 *
 * Developed for the Kaepek Project
 *
 */

/**
 * @brief Jerk model
 *  Implemention following this paper:
 *  [Ref1]: https://www.researchgate.net/publication/3002819_A_jerk_model_to_tracking_highly_maneuvering_targets/link/00b7d51f03ca864666000000/download
 *
 *  Definitions
 *  -------------------------------------------
 *  t: time
 *  k: a step index of the system
 *  j(t): jerk as a function of time
 *  w(t): white noise applying only to jerk as a function of time
 *  w_{k}: white noise at step k
 *  tau: a small time
 *  r_j(tau): jerk correlation as a function of a small time
 *  sigma_j: standard deviation of target jerk
 *  alpha: varience of w(t)
 *  delta(tau): the unit impulse function or "deltafunction"
 *  r_w(tau): auto correlation of the white noise input
 *  absolute value of variable g: ||g||
 *  q: the transform of the white noise w(t) that drives j(t)
 *  x: 1D displacement variable
 *  X_{k}: state of the system at step k. e.g. [position(x),velocity(dx/dt), acceleration(d2x/dt2), jerk(d3x/dt3)]
 *  X_{k+1}: state of the system at step t+1. e.g. [position,velocity, acceleration, jerk]
 *  dt: the real world time measured difference between consecutive steps k and k+1
 *  u(k): process noise as a function of steps
 *  B: jerk extractor matrix
 *  F(dt): Creates the state transition matrix as defined in [Ref1] low alpha tau (T in the reference) regiume, from step k to k+1
 *  F_{k+1,k}: State transition matrix from k to k+1
 *  Q(dt): Creates the covariance matrix of the process noise u(k) as a function of dt
 *  P(dt): Creates the covariance matrix initalisation as a function of dt
 *  P_{k,k+1}: Covariance matrix at step k to k+1
 *  P_{k-1,k}: The last covariance matrix at step k-1 to k
 *  Q_{k,k+1}: Variance matrix at step k to k+1
 *  sigma_x: standard deviation of variable x (uncertainty in measurement x)
 *  R: [[sigma_x^2]] [1x1]
 *  H: Measurement matrix [1x4]
 *
 *  jerk defined as:
 *  j(t) = - alpha * j(t) + w(t)
 *
 *  jerk correlation defined as:
 *  r_j(tau) = sigma_j^2 * exp(-alpha*||tau||)
 *
 *  white noise correlation defined as:
 *  r_w(tau) = 2 * alpha * sigma_j^2 * delta(tau) = q * delta(tau)
 *
 *  q defined as:
 *  q = 2 * alpha * sigma_j^2
 *
 *  B defined as:
 *  B = transpose([0, 0, 0, 1])
 *
 *  F
 *
 *  Method:
 *  -----------------------------------------------
 *  create Q_{k,k+1} and F_{k,k+1}:
 *  Q_{k,k+1} = Q(dt) [4x4]
 *  F_{k,k+1} = F(dt) [4x4]
 *
 *  take last known state X_{k} and project it ahead in time, one step forward using the transition matrix:
 *  X_{k+1} = F_{k,k+1} * X_{k} + B * w_{k}
 *  [4x1] = [4x4] * [4x1] + [4x4] * [4x1]
 *  note B adds w_{k} to jerk component of (F_{k,k+1} * X_{k}) we can omit this as it is environmental noise
 *
 *  Project the error covariance ahead:
 *  P_{k,k+1} = F_{k,k+1} * P_{-k,k} * F_{k,k+1}.T + Q_{k,k+1}
 *  [4x4] = [4x4] * [4x4] * [4x4] + [4x4]
 *
 *  Calculate uncertainty in estimate + uncertainty in measurement:
 *  S = H*P_{k,k+1}*H.T + R
 *  [1x1] = [1x4] * [4x4] * [4x1] + [1x1]
 *  [1x1] = [1x4] * [4x1] + [1x1] = [1x1] + [1x1] = [1x1]
 *
 *  Calculate uncertainty in estimate:
 *  C = (P_{k,k+1}*H.T)
 *  [4x1] = [4x4] * [4x1]
 *
 *  Calculate Kalman Gain:  (S^-1 is the inverted S matrix, in the 1d case it is just a recropical of a scalar)
 *  K = C * S^-1
 *  [4x1] = [4x1] * [1x1]
 *
 *  Update the estimate via z (get the last measurement)
 *  Z = x.reshape(H.shape[0],1) assume x is 1D [1x1]
 *  [1x1] = [1x1].reshape(1,1) 1d
 *  other cases:
 *  Z = x.reshape(H.shape[0],1) assume x is 3D [1x3] H would be (3x12)
 *  [3x1] = [1x3].reshape(3,1)
 *
 *  Calculate Innovation or Residual (assuming 1d case)
 *  Y = Z - (H*X_{k+1})
 *  [1x1] = [1x1] - ([1x4]*[4x1])
 *
 *  Calculate filtered new state
 *  X_FINAL_{k+1} = X_{k+1} + (K*Y)
 *
 *  Assign old to new state before looping again for another measurement
 *  X_{k} = X_{k+1} + (K*Y)
 *
 *  Before next iteration update the error covariance
 *  P_{-k,k} = (I - (K*H)) * P_{k,k+1}
 *  [4x4] = ([4x4] - ([4x1] * [1x4])) * [4x4]
 *  [4x4] = ( [4x4] - [4x4]) * [4x4] = [4x4] * [4x4]
 *
 *  k++ (loop for next measurement)
 */

/*

Code is simplified using dependancy chain analysis of the linear algebra algorithm as written above see (kalman.model2.py) for
details.


needed variables:
P1
P2
X [state]
Y
current_idx

needed methods:
private:
get_initial_P()
get_Q_low_alpha_T(dt)
get_F_low_alpha_T(dt)
calculate_diff_x()
calculate_diff_t()
public:
step(s)


*/

#ifndef KALMAN1D_H
#define KALMAN1D_H

namespace kaepek
{
    typedef double Dbl4x4[4][4];
    typedef double Dbl4x1[4];
    typedef double Dbl5x1[5];

    typedef double (&Dbl4x1Pointer)[4];
    typedef double (&Dbl5x1Pointer)[5];
    typedef double (&Dbl4x4Pointer)[4][4];

    typedef double *ndArr;
    typedef double P[4][4];
    typedef double Q[4][4];
    typedef double F[4][4];
    typedef double K[4];
    typedef double Y;
    typedef double X[4];
    typedef double Kalman_error;

    class KalmanJerk1D
    {
    private:
        double alpha;
        double x_variance;
        double jerk_variance;
        bool x_is_modular;
        double x_mod_limit;
        double x_mod_limit_over_2;
        int current_idx;
        bool time_is_relative;

        Dbl4x4 p;
        Dbl4x4 f;
        Dbl4x1 X_kp1;
        Dbl4x4 P_kp1;
        Dbl4x1 K;
        Dbl4x1 X;
        Dbl5x1 eular_state;
        Dbl4x4 q;
        double q_scale;

        /**
         * @brief Initalise the covariance matrix P as a function of dt
         *
         */
        void get_initial_P(double dt);

        /**
         * @brief Creates Q the process noise covariance matrix u(k) as a function of dt
         *
         */
        void get_Q_low_alpha_T(double dt);

        /**
         * @brief Creates the state transition matrix as defined in [Ref1] low alpha tau (T in the reference) regime, from step k to k+1
         *
         */
        void get_F_low_alpha_T(double dt);

        void kalman_step(double dx, double dt);

    protected:
    public:
        /**
         * @brief Get change in x between two displacement values
         *
         */
        double calculate_diff_x(double last_x, double current_x);

        /**
         * @brief Get change in t between two time values
         *
         */
        double calculate_diff_t(double last_t, double current_t);

        /**
         * @brief Get Kalman state X estimate of the system e.g. [position, velocity, acceleration, jerk], time t found in get_eular_vector()[0]
         *
         */
        Dbl4x1Pointer get_kalman_vector();

        /**
         * @brief Get the covariance matrix P
         *
         */
        Dbl4x4Pointer get_covariance_matrix();

        /**
         * @brief Get Eular state X estimate of the system e.g. [time, position, velocity, acceleration, jerk]
         *
         */
        Dbl5x1Pointer get_eular_vector();

        /**
         * Kalman1D constructor.
         *
         * @param x_resolution_error The standard deviation of a measurement variable x
         * @param x_jerk_error The standard deviation constraint of the 4th temporal derivative of a measurement variable x
         * @param alpha alpha used to calculate q factor with q = 2 * alpha * x_jerk_error^2
         * @return instance of Kalman1D class
         */
        KalmanJerk1D(double alpha, double x_resolution_error, double x_jerk_error, bool time_is_relative);

        /**
         * Kalman1D constructor.
         *
         * @param x_resolution_error The standard deviation of a measurement variable x
         * @param x_jerk_error The standard deviation constraint of the 4th temporal derivative of a measurement variable x
         * @param alpha alpha used to calculate q factor with q = 2 * alpha * x_jerk_error^2
         * @param x_mod_limit If x obeys modular arithmatic what is the maximum value before restarting x at 0
         * @return instance of Kalman1D class
         */
        KalmanJerk1D(double alpha, double x_resolution_error, double x_jerk_error, bool time_is_relative, double x_mod_limit);

        /**
         * Kalman1D step function
         * @param time (absolute time [s])
         * @param x (absolute position/rotation [units])
         */
        void step(double time, double x);
    };
}

#endif