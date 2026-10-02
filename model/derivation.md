# Jerk Model Filter Family: Derivation

This document derives every filter in the library from the jerk model of K. Mehrotra and P. R. Mahapatra, "A Jerk Model for Tracking Highly Maneuvering Targets", IEEE Transactions on Aerospace and Electronic Systems, vol. 33, no. 4, pp. 1094-1105, 1997. Equation numbers in parentheses refer to that paper. The sensor is stationary unless a class name ends in `MovingSensor`.

# Coverage

| Class | Measurement | Coverage by the paper | Paper equations | Additional derivation |
|-|-|-|-|-|
| `KalmanJerk1D` | $`x`$, or an angle with wrap limit | Complete (Sections II to IV) | (1) to (29) | None |
| `KalmanJerk2D` | $`(x, y)`$ | Three axis form only (Section V) | (32), (33), (39), (47), (49) | None |
| `KalmanJerk3D` | $`(x, y, z)`$ | Complete (Section V) | (32), (33), (39), (47), (49) | None |
| `KalmanJerk2DPolar` | $`(r, \theta)`$ | Planar case of Section V | (44), (45) with $`\varphi = 0`$ | None |
| `KalmanJerk3DSpherical` | $`(r, \theta, \varphi)`$ | Complete (Section V) | (44), (45) | None |
| `KalmanJerk2DAzEl` | $`(\theta, \varphi)`$ | Not covered | (5) to (8), (11), (13) applied on the sphere | Intrinsic jerk model on the unit sphere |
| `KalmanJerk1DBearingMovingSensor` | $`\beta`$ and sensor state | Not covered | (5) to (8), (11), (13) applied to relative motion | Modified polar coordinates of jerk order |
| `KalmanJerk2DAzElMovingSensor` | $`(\theta, \varphi)`$ and sensor state | Not covered | (5) to (8), (11), (13) applied to relative motion | Modified spherical coordinates of jerk order |

Angles follow the convention of (44): $`\theta`$ is azimuth and $`\varphi`$ is elevation above the horizontal plane.

# Notation

The following quantities are those of the paper and are used without rederivation.

| Symbol | Definition | Paper |
|-|-|-|
| $`\alpha`$, $`\sigma_j`$ | reciprocal jerk time constant and jerk standard deviation | (1) |
| $`Q_c`$ | white noise intensity $`2 \alpha \sigma_j^2`$ | (6) |
| $`A`$, $`B`$ | continuous state and input matrices of one axis | (7) |
| $`F(T)`$ | transition matrix of one axis | (14), (15); (16) when $`\alpha T`$ is small |
| $`Q(T)`$ | process noise covariance of one axis | (20); (21) when $`\alpha T`$ is small |
| $`\sigma_m`$ | standard deviation of the target acceleration | (28) |
| $`P_{proc}`$ | terms of the initial covariance (28) that involve $`\sigma_m`$, $`\sigma_j`$ and the $`q_{ij}`$, or of (29) when $`\alpha T`$ is small | (28), (29) |

The compile time policy `JerkExact` evaluates (14), (15) and (20), and `JerkSmallAlphaT` evaluates (16) and (21). The filters are initialised from the first three measurements by (22). For an angular coordinate with wrap limit $`M`$, differences of measurements are formed with

$$
d(a, b) = \mathrm{mod}(a - b, M) - M \cdot [\mathrm{mod}(a - b, M) > M/2]
$$

which maps the raw difference into the interval $`(-M/2, M/2]`$.

# KalmanJerk1D

`KalmanJerk1D` tracks a single coordinate with a four state jerk model: position, velocity, acceleration and jerk. The coordinate is either a linear position or an angle that wraps at a fixed limit.

It is the base of the family, and its per axis matrices are used by every other class.

## Derivation

Sections II to IV of the paper, (1) to (29). An angular coordinate lies on a circle, which has zero intrinsic curvature, so the paper's model applies to it unchanged once measurement differences are formed with $`d`$.

# KalmanJerk2D

`KalmanJerk2D` tracks a point moving in a plane from direct measurements of its Cartesian coordinates $`(x, y)`$. Each axis carries the four state jerk model of `KalmanJerk1D`, giving eight states.

It provides the two axis Cartesian core used by `KalmanJerk2DPolar`. The measurement covariance is a full 2 by 2 matrix, so measurement errors that are correlated between the axes are represented, and the axes are estimated jointly.

## Derivation

Section V of the paper, (32), (33), (39), (47) and (49), with two axes in place of three, and initialisation by (22) to (28) applied to each axis with the full measurement covariance.

# KalmanJerk3D

`KalmanJerk3D` tracks a point moving in three dimensional space from direct measurements of its Cartesian coordinates $`(x, y, z)`$. It has twelve states, four per axis.

It is the Cartesian core of the paper's three dimensional filter and is used by `KalmanJerk3DSpherical`. A full 3 by 3 measurement covariance allows correlated measurement errors, which arise whenever the measurements are produced by a coordinate conversion.

## Derivation

Section V of the paper, (32), (33), (39), (47) and (49), with initialisation by (22) to (28) applied to each axis with the full measurement covariance.

# KalmanJerk2DAzEl

`KalmanJerk2DAzEl` tracks a direction in three dimensional space from measurements of azimuth $`\theta`$ and elevation $`\varphi`$, with the range unknown. The direction moves on the unit sphere, and the filter carries a jerk model of that motion with eight degrees of freedom: two for the direction and two each for the angular rate, angular acceleration and angular jerk.

It serves sensors that measure direction only, such as acoustic direction finding. The range is not observable from a stationary sensor, so the motion model is posed on the sphere itself. The sphere has nonzero curvature, so no choice of two angular coordinates represents this motion as two independent copies of the one dimensional model; the filter is derived intrinsically on the sphere and its two tangent axes are coupled.

The class is parameterised at compile time:

| Parameter | Options | Default |
|-|-|-|
| `Form` | `JerkSmallAlphaT`, `JerkExact` | `JerkSmallAlphaT` |
| `Order` | `FirstOrder`, `SecondOrder`, `Unscented` | `FirstOrder` |
| `Diag` | `NoDiagnostics`, `WithDiagnostics` | `NoDiagnostics` |

`Form` selects the evaluation of the per axis chain, `Order` the propagation of the mean and covariance through the nonlinear model, and `Diag` whether the innovation quantities are stored.

## Derivation

### Tangent frame

The direction is the unit vector

$$
u = [\cos \varphi \cos \theta, \; \cos \varphi \sin \theta, \; \sin \varphi]^T
$$

with orthonormal tangent vectors

$$
e_a = [-\sin \theta, \; \cos \theta, \; 0]^T, \qquad
e_e = [-\sin \varphi \cos \theta, \; -\sin \varphi \sin \theta, \; \cos \varphi]^T
$$

along increasing azimuth and increasing elevation. Then $`\dot{u} = \omega_a e_a + \omega_e e_e`$ with physical angular rates

$$
\omega_a = \dot{\theta} \cos \varphi, \qquad \omega_e = \dot{\varphi}
$$

For a tangent vector field $`v`$ along the motion, the covariant derivative is the tangential part of the time derivative:

$$
\frac{D v}{dt} = \frac{d v}{dt} + (\dot{u} \cdot v) \, u
$$

### Continuous model

The model of (5) to (8) is posed on the sphere by replacing each time derivative of the kinematic chain with the covariant derivative. With angular velocity $`\omega = \dot{u}`$, angular acceleration $`a`$ and angular jerk $`j`$ as tangent vectors:

$$
\frac{D \omega}{dt} = a, \qquad \frac{D a}{dt} = j, \qquad \frac{D j}{dt} = -\alpha j + w
$$

where $`w`$ is white noise in the tangent plane with covariance $`Q_c I_2`$ in any orthonormal tangent basis, $`Q_c = 2 \alpha \sigma_j^2`$, and $`\sigma_j`$ is in radians per second cubed. The noise is isotropic in the tangent plane, which is the condition that the same physical angular jerk is equally probable in every direction of motion. The model contains no coordinates.

### State and dynamics

The state is the unit vector $`u`$ together with the tangent vectors $`\omega`$, $`a`$ and $`j`$, each held as a vector in three dimensions. From $`u \cdot u = 1`$ and the tangency conditions $`u \cdot \omega = u \cdot a = u \cdot j = 0`$, the covariant derivatives give

$$
\begin{aligned}
\dot{u} &= \omega \\
\dot{\omega} &= a - \lvert \omega \rvert^2 u \\
\dot{a} &= j - (\omega \cdot a) \, u \\
\dot{j} &= -\alpha j - (\omega \cdot j) \, u + w
\end{aligned}
$$

These equations preserve the unit norm and the tangency conditions exactly, and contain no singular point. In the coordinates $`(\theta, \varphi)`$, with $`\omega_a = \dot{\theta} \cos \varphi`$ and $`\omega_e = \dot{\varphi}`$, unforced motion ($`a = j = w = 0`$) satisfies

$$
\ddot{\theta} = 2 \tan \varphi \, \dot{\theta} \dot{\varphi}, \qquad \ddot{\varphi} = -\sin \varphi \cos \varphi \, \dot{\theta}^2
$$

which are the geodesic equations of the unit sphere, so unforced motion follows great circles. These coordinates are singular at $`\varphi = \pm \pi / 2`$, where $`\cos \varphi = 0`$ and the azimuth is undefined, and the equations of motion written in them contain the coefficients $`\sec \varphi`$ and $`\tan \varphi`$, which are unbounded there. The state above is defined at every direction.

### Error representation

The covariance is held in an orthonormal basis $`B = [b_1, b_2]`$ of the tangent plane at the estimate. The basis is transported in parallel with the state, $`\dot{b}_i = -(\omega \cdot b_i) \, u`$, and is rotated with the state at each update, so that it is defined at every direction. The error $`\delta \in \mathbb{R}^8`$ is ordered by axis as in (35), $`\delta = [v_1, \delta\omega_1, \delta a_1, \delta j_1, v_2, \delta\omega_2, \delta a_2, \delta j_2]^T`$, and the tangent vectors it represents are $`v = B [v_1, v_2]^T`$ and likewise for the other components.

A state is obtained from the estimate $`(\hat{u}, \hat{\omega}, \hat{a}, \hat{j})`$ and an error $`\delta`$ by the retraction

$$
\mathcal{R}(\delta) = \left( R \hat{u}, \; R (\hat{\omega} + \delta\omega), \; R (\hat{a} + \delta a), \; R (\hat{j} + \delta j) \right), \qquad R = \mathrm{Rot}(\hat{u} \times v)
$$

where $`\mathrm{Rot}(r)`$ is the rotation by angle $`\lvert r \rvert`$ about $`r`$. The rotation $`R`$ moves $`\hat{u}`$ along the great circle in direction $`v`$ through angle $`\lvert v \rvert`$ and is the parallel transport along that great circle, so it maps the tangent plane at $`\hat{u}`$ isometrically onto the tangent plane at $`R \hat{u}`$. The inverse is

$$
v = \mathrm{Log}_{\hat{u}}(u), \qquad \delta\omega = R^T \omega - \hat{\omega}, \quad \delta a = R^T a - \hat{a}, \quad \delta j = R^T j - \hat{j}
$$

with components taken in $`B`$ and

$$
\mathrm{Log}_{\hat{u}}(u) = \mathrm{atan2}(\lvert \tau \rvert, \hat{u} \cdot u) \, \frac{\tau}{\lvert \tau \rvert}, \qquad \tau = u - (\hat{u} \cdot u) \, \hat{u}
$$

### Error dynamics

Linearising the dynamics in the retraction and differentiating in the transported basis gives the error dynamics $`\dot{\delta} = A_\delta \delta + G w`$ with, for each pair of tangent components,

$$
\begin{aligned}
\dot{v} &= \delta\omega \\
\dot{\delta\omega} &= \delta a - K_\omega v \\
\dot{\delta a} &= \delta j - K_a v \\
\dot{\delta j} &= -\alpha \, \delta j - K_j v + w
\end{aligned}
$$

where, with $`\omega_t = B^T \omega`$, $`a_t = B^T a`$ and $`j_t = B^T j`$,

$$
K_\omega = \lvert \omega \rvert^2 I_2 - \omega_t \omega_t^T, \qquad
K_a = (\omega \cdot a) I_2 - \omega_t a_t^T, \qquad
K_j = (\omega \cdot j) I_2 - \omega_t j_t^T
$$

and $`G`$ selects the two jerk components. Without the terms in $`K_\omega`$, $`K_a`$ and $`K_j`$, each tangent axis carries exactly the chain (7). The term in $`K_\omega`$ is the Jacobi equation of the unit sphere, which describes the convergence of neighbouring great circles; $`K_a`$ and $`K_j`$ are its counterparts for the higher derivatives. These terms couple the position error into the rate, acceleration and jerk of both tangent axes.

### Discretisation

The transition matrix (11) and the process noise covariance (13) of the paper are the solutions over one sampling interval of

$$
\dot{\Phi} = A \Phi, \quad \Phi(0) = I, \qquad \dot{Q} = A Q + Q A^T + G Q_c G^T, \quad Q(0) = 0
$$

with $`\Phi(T) = e^{A T}`$ and $`Q(T) = Q_c \int_0^T e^{A u} G G^T e^{A^T u} du`$. For the model on the sphere the same equations are solved with $`A_\delta`$ evaluated along the predicted trajectory, jointly with the state and the basis:

$$
\begin{aligned}
&\dot{u}, \dot{\omega}, \dot{a}, \dot{j} \text{ as above with } w = 0, & &\text{from the estimate} \\
&\dot{b}_i = -(\omega \cdot b_i) \, u, & &b_i(0) = b_i \\
&\dot{\Phi} = A_\delta(t) \, \Phi, & &\Phi(0) = I \\
&\dot{Q} = A_\delta(t) \, Q + Q \, A_\delta(t)^T + G Q_c G^T, & &Q(0) = 0
\end{aligned}
$$

giving the propagated state, the propagated basis, $`F = \Phi(T)`$ and $`Q_{dir} = Q(T)`$. The last expression is the time varying form of (13). The system is integrated by the classical fourth order Runge Kutta method with $`n_s`$ equal substeps over $`T`$.

`JerkExact` uses $`\alpha`$ as given in the dynamics and in $`A_\delta`$. `JerkSmallAlphaT` sets $`\alpha = 0`$ in both while retaining $`Q_c = 2 \alpha \sigma_j^2`$, which is the limit taken in (16) and (21). When the angular rate, acceleration and jerk vanish, $`A_\delta = I_2 \otimes A`$ and the solutions are $`I_2 \otimes F(T)`$ and $`I_2 \otimes Q(T)`$ with $`F(T)`$ and $`Q(T)`$ given by (14) and (20), or by (16) and (21).

### Measurement

The measured azimuth and elevation define the measured direction $`m = u(\theta_m, \varphi_m)`$. The innovation is the tangent vector from the predicted direction to the measured direction, in the basis $`B`$:

$$
y = B^T \mathrm{Log}_{u^-}(m)
$$

The measurement noise is modelled as additive in the tangent plane at the predicted direction, $`y = H \delta + n`$ with $`n \sim N(0, R_t)`$ and $`H`$ selecting $`v_1`$ and $`v_2`$. Since $`\mathrm{Log}_{u^-}(\mathrm{Exp}_{u^-}(v)) = v`$, the innovation is linear in the error. The covariance $`R_t`$ is a 2 by 2 matrix in the tangent plane. For a sensor with isotropic angular error $`\sigma_d`$, $`R_t = \sigma_d^2 I_2`$. For a sensor characterised by azimuth and elevation variances,

$$
R_t = B^T R_m J \, \mathrm{diag}(\sigma_\theta^2, \sigma_\varphi^2) \, J^T R_m^T B, \qquad J = [\cos \varphi_m \, e_a, \; e_e]
$$

with $`J`$ evaluated at the measured direction and $`R_m`$ the parallel transport from $`m`$ to $`u^-`$. This covariance has rank one at $`\varphi_m = \pm \pi/2`$, where the azimuth variance describes no displacement on the sphere.

The update is

$$
S = H P^- H^T + R_t, \quad K = P^- H^T S^{-1}, \quad \hat{\delta} = K y, \quad P = (I - K H) P^-
$$

and the estimate is the retraction of the predicted state by $`\hat{\delta}`$, with the basis rotated by the same rotation $`R`$.

### Output

The azimuth and elevation are $`\theta = \mathrm{atan2}(u_y, u_x)`$ and $`\varphi = \arcsin u_z`$, and the tangent components of $`\omega`$, $`a`$ and $`j`$ along $`e_a`$ and $`e_e`$ are their inner products with these vectors. At $`u = \pm [0, 0, 1]^T`$ the azimuth and the frame $`(e_a, e_e)`$ are undefined while the state itself remains defined.

### Initialisation

The estimate (22) is formed in Riemannian normal coordinates of the sphere centred on the third measured direction $`c = m(3)`$. The basis $`B_0`$ at $`c`$ is formed from the coordinate axis $`e_k`$ least aligned with $`c`$: $`b_1 = (e_k - (e_k \cdot c) c) / \lvert e_k - (e_k \cdot c) c \rvert`$, $`b_2 = c \times b_1`$. A direction $`m`$ has normal coordinates $`p = B_0^T \mathrm{Log}_c(m)`$, whose magnitude is the great circle distance from $`c`$. Great circles through $`c`$ are straight lines in these coordinates traversed at constant speed, and the Christoffel symbols vanish at the origin, so the coordinate velocity and acceleration of a trajectory through $`c`$ equal its angular velocity and covariant acceleration.

With $`p_n`$ the normal coordinates of $`m(n)`$, and $`p_3 = 0`$, (22) gives

$$
\hat{u} = c, \quad \hat{\omega} = B_0 \frac{p_3 - p_2}{T}, \quad \hat{a} = B_0 \frac{p_3 - 2 p_2 + p_1}{T^2}, \quad \hat{j} = 0
$$

The initial covariance is the error analysis (23) to (28) carried through these coordinates:

$$
P = \sum_{n=1}^{3} G_n R_t(n) G_n^T + P_{proc}, \qquad G_n = \frac{\partial \delta}{\partial n(n)}
$$

where $`n(n)`$ is the tangent measurement error at sample $`n`$, $`R_t(n)`$ its covariance, and $`\delta`$ the error of the resulting estimate in the basis $`B_0`$. The Jacobians include the dependence of $`c`$ and $`B_0`$ on $`m(3)`$. $`P_{proc}`$ is that of (28), or of (29) when $`\alpha T`$ is small, for each tangent axis, with $`\sigma_m`$ and $`\sigma_j`$ in angular units. In the limit of small angular separations the estimate and covariance reduce to (22) and (28) on each tangent axis.

### Order

The measurement is linear in the error, so the update is identical for every order. The orders differ in the propagation of the mean and covariance over one sampling interval. Let $`\psi(\delta)`$ denote the error at the end of the interval of the trajectory that starts at error $`\delta`$, relative to the propagated estimate:

$$
\psi(\delta) = \mathcal{R}_{T}^{-1} \left( \phi_T ( \mathcal{R}_0(\delta) ) \right)
$$

where $`\phi_T`$ is the flow of the noise free dynamics over $`T`$, $`\mathcal{R}_0`$ is the retraction at the estimate and $`\mathcal{R}_T^{-1}`$ the inverse retraction at the propagated estimate. The Jacobian of $`\psi`$ at zero is $`F`$.

#### FirstOrder

The mean is the propagated estimate and the covariance is

$$
P^- = F P F^T + Q_{dir}
$$

which is the propagation of the extended Kalman filter.

#### SecondOrder

The Gaussian second order filter retains the second derivatives $`\Psi_i = \partial^2 \psi_i / \partial \delta^2`$ at zero. The mean error and the covariance after propagation are

$$
\bar{\delta}_i = \frac{1}{2} \mathrm{tr}(\Psi_i P), \qquad
P^- = F P F^T + \frac{1}{2} \left[ \mathrm{tr}(\Psi_i P \Psi_k P) \right]_{ik} + Q_{dir}
$$

and the predicted estimate is the retraction of the propagated estimate by $`\bar{\delta}`$. The second derivatives are obtained from the second order variational equations of the flow,

$$
\dot{Y}_{kl} = A(t) \, Y_{kl} + f_{xx}(t)[\Phi e_k, \Phi e_l], \qquad Y_{kl}(0) = 0
$$

integrated with the first order equations, where $`f_{xx}`$ is the second derivative of the dynamics, combined by the chain rule with the second derivatives of $`\mathcal{R}_0`$ and $`\mathcal{R}_T^{-1}`$. The covariance term in $`\Psi_i P \Psi_k P`$ is the contribution of the fourth moments of a Gaussian error.

#### Unscented

The scaled unscented transform with parameters $`\alpha_s`$, $`\beta_s`$, $`\kappa_s`$ and $`\lambda = \alpha_s^2 (8 + \kappa_s) - 8`$ forms the seventeen errors

$$
\delta_0 = 0, \qquad \delta_{\pm i} = \pm \left( \sqrt{(8 + \lambda) P} \right)_i, \quad i = 1, \ldots, 8
$$

with weights $`W^m_0 = \lambda / (8 + \lambda)`$, $`W^c_0 = W^m_0 + 1 - \alpha_s^2 + \beta_s`$ and $`W^m_{\pm i} = W^c_{\pm i} = 1 / (2 (8 + \lambda))`$, where $`(\cdot)_i`$ is the $`i`$th column of a matrix square root. Each point is retracted, propagated by the full nonlinear flow and expressed as an error at the propagated estimate, $`\xi_i = \psi(\delta_i)`$. The predicted estimate is the retraction of the propagated estimate by $`\bar{\xi} = \sum_i W^m_i \xi_i`$, the points are re expressed at the predicted estimate as $`\xi_i'`$ with weighted mean $`\bar{\xi}'`$, and

$$
P^- = \sum_i W^c_i (\xi_i' - \bar{\xi}')(\xi_i' - \bar{\xi}')^T + Q_{dir}
$$

### Numerical verification

The derivation was verified against direct simulation of the continuous model on the sphere, with $`\alpha = 0.2`$, $`\sigma_j = 0.05`$, $`T = 0.05`$ and measurement standard deviation $`0.003`$ radians. The error dynamics $`A_\delta`$ agree with finite differences of the exact map $`\psi`$ to $`10^{-9}`$, and in the limit of vanishing rates the discretisation reproduces (14), (16), (20) and (21). The table lists the mean normalised innovation squared (expected value 2) and the mean normalised estimation error squared (expected value 8) over Monte Carlo runs. The overhead scenario passes within $`3 \times 10^{-3}`$ radians of the zenith.

| Order | Scenario | Runs | NIS | NEES |
|-|-|-|-|-|
| `FirstOrder` | general | 40 | 1.99 | 8.29 |
| `FirstOrder` | overhead | 40 | 2.01 | 8.38 |
| `SecondOrder` | general | 12 | 1.96 | 7.66 |
| `SecondOrder` | overhead | 12 | 1.96 | 7.92 |
| `Unscented` | general | 40 | 1.99 | 8.29 |
| `Unscented` | overhead | 40 | 2.01 | 8.38 |

The isotropic measurement covariance $`R_t = \sigma_d^2 I_2`$ is used in these runs. Within $`0.05`$ radians of the zenith the mean normalised estimation error squared is 8.28. In passes with closest distances to the zenith from 0 to 0.3 radians, at angular speeds of 0.1, 0.5 and 2 radians per second, no track was lost and the root mean square position error near the closest approach was between 2.0 and 2.8 milliradians.

# KalmanJerk1DBearingMovingSensor

`KalmanJerk1DBearingMovingSensor` tracks a target moving in a plane from bearing measurements $`\beta`$ taken by a sensor whose own kinematic state is known at every step. It estimates the bearing, its derivatives to jerk order, the derivatives of the logarithm of range, and the inverse range.

When the sensor moves, the bearings depend on the range through parallax, which makes the range observable. The target follows the paper's Cartesian jerk model, and the filter is expressed in modified polar coordinates so that the observable angular quantities are separated from the inverse range.

## Derivation

### Relative motion

Let $`X_t`$ be the target state of `KalmanJerk2D` and $`X_s`$ the known sensor state of the same structure. The relative state $`X_\rho = X_t - X_s`$ evolves as

$$
X_\rho(k+1) = F_2 X_\rho(k) + F_2 X_s(k) - X_s(k+1) + v(k), \qquad v(k) \sim N(0, Q_2)
$$

where $`F_2`$ and $`Q_2`$ are those of `KalmanJerk2D`. This is (32) for the target, expressed relative to the sensor.

### Modified polar coordinates

Writing the relative position as the complex number $`\rho = r e^{i \beta} = e^{L}`$ with $`L = \lambda + i \beta`$ and $`\lambda = \ln r`$, successive derivatives satisfy

$$
\frac{\dot{\rho}}{\rho} = \dot{L}, \qquad \frac{\ddot{\rho}}{\rho} = \dot{L}^2 + \ddot{L}, \qquad \frac{\dddot{\rho}}{\rho} = \dot{L}^3 + 3 \dot{L} \ddot{L} + \dddot{L}
$$

The filter state is

$$
Y = [\beta, \dot{\beta}, \ddot{\beta}, \dddot{\beta}, \dot{\lambda}, \ddot{\lambda}, \dddot{\lambda}, 1/r]^T
$$

All components except $`1/r`$ are invariant under scaling of the relative motion. For $`\lambda`$ derivatives of first order this is the modified polar state of bearings only tracking, in which $`\dot{\lambda} = \dot{r}/r`$; the higher derivatives extend it to the jerk order of the paper.

The relative state normalised by range, $`\xi = X_\rho / r`$, is a function of the first seven components of $`Y`$. With $`z_n = \lambda^{(n)} + i \beta^{(n)}`$ and $`c_n = \rho^{(n)} / r`$:

$$
c_0 = e^{i \beta}, \quad c_1 = c_0 z_1, \quad c_2 = c_0 (z_1^2 + z_2), \quad c_3 = c_0 (z_1^3 + 3 z_1 z_2 + z_3)
$$

and the real and imaginary parts of $`c_n`$ are the $`x`$ and $`y`$ components of $`\xi`$.

### Prediction

The normalised relative state at step $`k+1`$, scaled by the range at step $`k`$, is

$$
\xi' = F_2 \xi(k) + \frac{1}{r(k)} \left( F_2 X_s(k) - X_s(k+1) \right) + \frac{v(k)}{r(k)}
$$

From $`\xi'`$ the state at step $`k+1`$ is recovered exactly:

$$
\frac{r(k+1)}{r(k)} = \lvert c_0' \rvert, \quad \beta(k+1) = \arg c_0', \quad w_n = \frac{c_n'}{c_0'}, \quad
z_1 = w_1, \quad z_2 = w_2 - z_1^2, \quad z_3 = w_3 - 3 z_1 z_2 - z_1^3
$$

$$
\frac{1}{r(k+1)} = \frac{1}{r(k)} \cdot \frac{1}{\lvert c_0' \rvert}
$$

The composition of these maps, denoted $`Y(k+1) = \Psi(Y(k), v(k))`$, is the mean prediction with $`v = 0`$. The covariance is propagated with the Jacobians of $`\Psi`$:

$$
P^- = J_Y P J_Y^T + G \, \frac{Q_2}{r(k)^2} \, G^T, \qquad J_Y = \frac{\partial \Psi}{\partial Y}, \qquad G = \frac{\partial \Psi}{\partial \xi'}
$$

evaluated at the current estimate, with $`1/r(k)`$ taken from the state. $`F_2`$ and $`Q_2`$ follow the policy `JerkSmallAlphaT` or `JerkExact`, so the target dynamics are exactly those of Section V restricted to two axes.

When the sensor is stationary, $`X_s = 0`$ and the first seven components of $`Y`$ propagate independently of $`1/r`$, which enters only through the scaling of $`Q_2`$. The sensor term couples $`1/r`$ into the angular components, which is the mechanism by which range becomes observable.

### Measurement and update

The measurement is $`z = \beta_m`$ with $`H = [1, 0, 0, 0, 0, 0, 0, 0]`$ and $`R = \sigma_\beta^2`$. The innovation is formed with $`d`$ with $`M = 2 \pi`$, and the update follows `KalmanJerk1D`.

### Initialisation

The bearing and its derivatives are initialised from the first three measurements by (22), with covariance of the form (29). The derivatives of $`\lambda`$ are initialised at zero with prior variances $`\sigma_{\lambda 1}^2`$, $`\sigma_{\lambda 2}^2`$, $`\sigma_{\lambda 3}^2`$. The inverse range is initialised from a prior interval $`[r_{min}, r_{max}]`$ as a uniform distribution in $`1/r`$:

$$
E\{1/r\} = \frac{1}{2} \left( \frac{1}{r_{min}} + \frac{1}{r_{max}} \right), \qquad
\mathrm{var}\{1/r\} = \frac{1}{12} \left( \frac{1}{r_{min}} - \frac{1}{r_{max}} \right)^2
$$

# KalmanJerk2DAzElMovingSensor

`KalmanJerk2DAzElMovingSensor` tracks a target moving in three dimensional space from azimuth and elevation measurements taken by a sensor whose own kinematic state is known at every step. It estimates the direction, the angular rate, acceleration and jerk in the tangent plane, the derivatives of the logarithm of range, and the inverse range.

It is the three dimensional counterpart of `KalmanJerk1DBearingMovingSensor`. The target follows the paper's three dimensional Cartesian jerk model, and the filter is expressed in modified spherical coordinates so that the observable angular quantities are separated from the inverse range.

## Derivation

### Relative motion

With $`F_3`$ and $`Q_3`$ those of `KalmanJerk3D`, the relative state $`X_\rho = X_t - X_s`$ evolves as

$$
X_\rho(k+1) = F_3 X_\rho(k) + F_3 X_s(k) - X_s(k+1) + v(k), \qquad v(k) \sim N(0, Q_3)
$$

which is (32) for the target expressed relative to the sensor.

### Modified spherical coordinates

The relative position is $`\rho = r u`$ with $`u`$ the unit direction of `KalmanJerk2DAzEl`. With $`\omega`$, $`a`$ and $`j`$ the angular velocity, acceleration and jerk of `KalmanJerk2DAzEl` as tangent vectors, the derivatives of $`u`$ are

$$
\dot{u} = \omega, \qquad \ddot{u} = a - \lvert \omega \rvert^2 u, \qquad \dddot{u} = j - \lvert \omega \rvert^2 \omega - 3 (\omega \cdot a) u
$$

which follow from $`u \cdot u = 1`$ and the definitions $`D\omega/dt = a`$, $`Da/dt = j`$. With $`\lambda = \ln r`$, the normalised derivatives $`\eta_n = \rho^{(n)} / r`$ are

$$
\begin{aligned}
\eta_0 &= u \\
\eta_1 &= \dot{\lambda} u + \omega \\
\eta_2 &= (\ddot{\lambda} + \dot{\lambda}^2 - \lvert \omega \rvert^2) u + 2 \dot{\lambda} \omega + a \\
\eta_3 &= (\dddot{\lambda} + 3 \dot{\lambda} \ddot{\lambda} + \dot{\lambda}^3 - 3 \dot{\lambda} \lvert \omega \rvert^2 - 3 \omega \cdot a) u + 3 (\ddot{\lambda} + \dot{\lambda}^2) \omega + 3 \dot{\lambda} a + j - \lvert \omega \rvert^2 \omega
\end{aligned}
$$

The filter state is

$$
Y = \left( u, \; \omega, \; a, \; j, \; \dot{\lambda}, \; \ddot{\lambda}, \; \dddot{\lambda}, \; 1/r \right)
$$

with $`u`$, $`\omega`$, $`a`$ and $`j`$ held as in `KalmanJerk2DAzEl`, so that the state is defined at every direction. All components except $`1/r`$ are invariant under scaling of the relative motion. The first order part of this state is the modified spherical state of bearings only tracking; the higher derivatives extend it to the jerk order of the paper. The error has twelve components: the eight of the `KalmanJerk2DAzEl` error, taken through its retraction and transported basis, followed by the errors of $`\dot{\lambda}`$, $`\ddot{\lambda}`$, $`\dddot{\lambda}`$ and $`1/r`$, which enter additively. The vectors $`\eta_n`$ are the components of the normalised relative state $`\xi = X_\rho / r`$.

### Prediction

As in the planar case,

$$
\xi' = F_3 \xi(k) + \frac{1}{r(k)} \left( F_3 X_s(k) - X_s(k+1) \right) + \frac{v(k)}{r(k)}
$$

The state at step $`k+1`$ is recovered from $`\xi'`$ by normalising with $`\lvert \eta_0' \rvert = r(k+1)/r(k)`$ and inverting the expressions above, with $`T(v) = v - (u \cdot v) u`$ the tangential projection:

$$
\begin{aligned}
u &= \eta_0 \\
\dot{\lambda} &= u \cdot \eta_1, \qquad \omega = \eta_1 - \dot{\lambda} u \\
\ddot{\lambda} &= u \cdot \eta_2 - \dot{\lambda}^2 + \lvert \omega \rvert^2, \qquad a = T(\eta_2) - 2 \dot{\lambda} \omega \\
\dddot{\lambda} &= u \cdot \eta_3 - 3 \dot{\lambda} \ddot{\lambda} - \dot{\lambda}^3 + 3 \dot{\lambda} \lvert \omega \rvert^2 + 3 \omega \cdot a, \qquad j = T(\eta_3) - 3 (\ddot{\lambda} + \dot{\lambda}^2) \omega - 3 \dot{\lambda} a + \lvert \omega \rvert^2 \omega
\end{aligned}
$$

and

$$
\frac{1}{r(k+1)} = \frac{1}{r(k)} \cdot \frac{1}{\lvert \eta_0' \rvert}
$$

The composition is the mean prediction. The covariance is propagated with the Jacobians of the composite map in the error coordinates, the retraction at the estimate on the input side and the inverse retraction at the prediction on the output side, as in `KalmanJerk1DBearingMovingSensor`, with $`Q_3 / r(k)^2`$ in place of $`Q_2 / r(k)^2`$. The basis of the tangent plane is carried to the prediction by the parallel transport along the great circle from the estimated direction to the predicted direction. When the sensor is stationary the first eleven components propagate independently of $`1/r`$ except through the scaling of $`Q_3`$.

### Measurement and update

The measurement, innovation, measurement covariance and update are those of `KalmanJerk2DAzEl`, with $`H`$ selecting the two direction components of the twelve component error. The output angles and tangent components are formed as for `KalmanJerk2DAzEl`.

### Initialisation

The direction and its derivatives are initialised as in `KalmanJerk2DAzEl`. The derivatives of $`\lambda`$ and the inverse range are initialised as in `KalmanJerk1DBearingMovingSensor`.
