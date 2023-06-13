# KAEPEK-KALMAN V1.1.1

## Description

Velocity, acceleration and jerk can be computed using the Eular method from temporal and angular/spacial measurements, some effort is required to smooth these measurements and deal with error in the higher derivatives. Kalman or more specifically EKF will be used with a jerk as white noise model.

[Model derivation](model/TrackingKalman.pdf)

## Python

### Installation

```
pip3 install git+https://github.com/kaepek/kalman.git
```

### Usage

```
import CyKalman
filter1D = CyKalman.KalmanJerk1D(0.1, 1.0. 1.0, False, 2**14) # <alpha>, <x_error>, <jerk_constraint>, <time_is_relative>, <x_mod_limit>

t = 0
x = 0
filter1D.step(t, x)

kalman_state = filter1D.get_kalman_vector()
eular_state = filter1D.get_eular_vector()
p = filter1D.get_covariance_matrix()
```

## Cpp

Include `kalman.cpp` and `kalman.hpp` within your code. Use a symbolic link or a git-submodule to include this in your project.

```
#include "/somedir/lib/jerk/kalman_jerk.hpp"
#include "/somedir/lib/jerk/kalman_jerk.cpp"

double alpha = 0.1;
double x_resolution_error = 1.0;
double x_jerk_error = 0.1;
bool time_is_relative = false;
double x_mod_limit = 16384.0;

KalmanJerk1D k1 = kaepek::KalmanJerk1D(alpha, x_resolution_error, x_jerk_error, time_is_relative);
KalmanJerk1D k2 = kaepek::KalmanJerk1D(alpha, x_resolution_error, x_jerk_error, time_is_relative, x_mod_limit);

void loop() {
  // get <value> somehow... not specified here
  k1.step(seconds_since_last, value);
  double *kalman_vec = k1.get_kalman_vector();
  double *eular_vec = k1.get_eular_vector();
  double *covar = k1.get_covariance_matrix();
}
```

# Testing

## Local installation

Use Bash on Linux/Mac or GitBash on Windows.

### Create a virtual environment

```
npm run create:venv
```

### Activate the virtual environment

Bash: 
```
source env/bin/activate
```

Windows: 
```
. env/Scripts/Activate
```

### Install dependancies:

```
npm run install:venv
```

### Then... Self installation:

This must be done before running any local(to the repository) tests:
```
npm run create:wheel && npm run install:wheel
```

## State estimation graphing software for validation:

### Fixed speed, absolute time, 1,000,000 steps benchmark

```
time npm run simulate:fixed-speed:1e6:abs --py_or_cy=cy
```
Set ```--py_or_cy``` to ```cy``` for cython and ```py``` for native python

### Fixed speed, relative time, 1,000,000 steps benchmark

```
time npm run simulate:fixed-speed:1e6:rel --py_or_cy=cy
```
Set ```--py_or_cy``` to ```cy``` for cython and ```py``` for native python

### IBM dataset:

REQUIRES: Downloading the [IBM double pendulum dataset](./datasets/README.md) first!
```
npm run download:ibm-double-pendulum
```

In the root project directory run this command to simulate the double pendulum...
```
npm run simulate:double-pendulum --py_or_cy=cy --dataset=6
```

Set ```--py_or_cy``` to ```cy``` for cython and ```py``` for native python
Set ```--dataset=6 ``` to change which dataset to graph post kalman filtering.

Note simulate:double-pendulum will automatically invoke ```npm run install:ibm-double-pendulum``` for the given dataset.

## Perform BLDC duty simulation
In the root project directory run...
```
npm run simulate:bldc
```

## Results


## References / Useful links

- [A_jerk_model_to_tracking_highly_maneuvering_targets](https://www.researchgate.net/publication/3002819_A_jerk_model_to_tracking_highly_maneuvering_targets)
- [Jerk stackoverflow](https://dsp.stackexchange.com/questions/24847/wrong-estimation-of-derivatives-with-an-extended-kalman-filter)

- [Kalman-Filter-CA.py constant acceleration example](https://github.com/balzer82/Kalman/blob/master/Kalman-Filter-CA.py)

## Code Dependancies

- [setuptools](https://github.com/pypa/setuptools/blob/main/LICENSE)
- [wheel](https://github.com/pypa/wheel/blob/main/LICENSE.txt)
- [Cython](https://github.com/cython/cython/blob/master/LICENSE.txt)
- [numpy](https://numpy.org/doc/stable/license.html)
- [sympy](https://github.com/sympy/sympy/blob/master/LICENSE)