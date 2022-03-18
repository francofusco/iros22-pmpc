# Investigating the Performances of Control Parameterizations for Nonlinear Model Predictive Control

Work in Progress. The plan is to include precompiled binaries for the PMPC library and the source code for the demos. This should make it much easier to compile and test our examples!



## Dependencies

- Eigen3
- OpenMP
- Python3 headers (package `python3-dev`), matplotlib (for python3)
- qpOASES


## Bulding the examples

First of all, you need to tell CMake how to locate qpOASES. Open the file `CMakeLists.txt` and look for the line `set(qpOASES_BASE_DIR "$ENV{HOME}/programs/qpOASES")`. Change the path to point to where you installed qpOASES. In my case, I cloned the qpOASES repository in `~/programs/qpOASES`, which explains the default value that I gave to `qpOASES_BASE_DIR`.

Now, in the root of this project, create a build folder and invoke CMake "as usual":

```
mkdir build && cd build
cmake ..
cmake --build .
```


## Running the examples

Three examples are provided:

- `triple_integrator` solves a constrained parameterized MPC optimization for a linear triple integrator.
- `crane_pendulum` solves a constrained parameterized MPC optimization for a nonlinear inverted pendulum (cart-pole). The goal is to move the base while keeping the pole balanced.
- `crane_pendulum_swingup` as above, but the task is to swingup the pole rather than move the base around.

In all cases, you can run the examples with different parameterizations as:

```
./example_name parameterization
```

In which the `parameterization` argument can be used to select different parameterizations:
- `full`: the number of parameters corresponds to the prediction horizon.
- `simple`: uses the "simple" parameterization.
- `zoh`: uses the "zero-order-holder" parameterization, with free parameters spread along the prediction horizon.
- `lerp`: uses the "linear interpolation" parameterization, with free parameters spread along the prediction horizon.
- `poly`: uses a function basis parameterization with exponentially damped polynomials.


# Credits

The `matplotlibcpp.h` file was copied from the open source project [matplotlibcpp](https://github.com/lava/matplotlib-cpp).
