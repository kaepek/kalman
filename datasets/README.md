# Test datasets

## IBM's "Double Pendulum Chaotic" dataset

### About

Videos of the chaotic motion of a double pendulum, in addition to the positions of the datums of the pendulum.

- [Source website](https://developer.ibm.com/exchanges/data/all/double-pendulum-chaotic/)

- [License (must download first)](./data//double-pendulum/LICENSE.pdf)

### Usage

#### Download

In the project root directory run Bash (or GitBash) command...
  - ```npm run download:ibm-double-pendulum```

#### Installation

Pick a dataset 0->20 and install it:

```npm run install:ibm-double-pendulum --dataset=<dataset>```

Example:

```npm run install:ibm-double-pendulum --dataset=0```

#### Test kalman with double pendulum

```
npm run simulate:double-pendulum --dataset=6 --py_or_cy=cy
```

Set ```--py_or_cy``` to ```cy``` for cython and ```py``` for native python
Set ```--dataset=6 ``` to change which dataset to graph post kalman filtering.

Note simulate:double-pendulum will automatically invoke ```npm run install:ibm-double-pendulum``` for the given dataset.