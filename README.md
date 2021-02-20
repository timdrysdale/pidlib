# pidlib
Simple discrete PID implementation 

## Purpose

This implementation of a discrete controller has been developed for use in an Arduino microcontroler, although no hardware-specific features are required by the library. It can therefore be used on other devices.

## Source

The implementation is based on the discrete digital PID implementation presented [here](https://www.scilab.org/discrete-time-pid-controller-implementation).

It has the following features:

 - derivative term is a low-pass filter to make it less noisy 
 - backward Euler representations used for the integral and derivative terms 
 - final version of equation is in difference equation form
 - plant limits enforced in the controller, preventing history from accumulating non-physical out-of-limit terms.
 - closer match to continuous time PID controller than version with non-filtered derivative term

## Improvements

In its originally-presented form, the controller suffers from [integral windup](https://en.wikipedia.org/wiki/Integral_windup). This is fixed by zero-ing the integral terms `u1` and `u2` if the error becomes zero, or changes sign (which occurs when zero is crossed). This avoids the unsatisfactory situation of having the controller spend an equal time on the other side of the setpoint as it spend on the previous side. The zero crossing is detected by checking whether the multiplication of the current and last step's error terms is less than or equal to zero.

## Testing

The [googletest](https://github.com/google/googletest) C++ test framework is used to run some simple tests on the individual P, I, and D components of the controller. 

The tests are contained in `pid_unittests.cpp`, and are run as follows (on a linux platform with `g++`, `cmake`, `gtest` installed). 

```
make
./runTests
```

The tests should work on other platforms, as no platform-specific libraries are used.

## Real world performance

This library has not yet been tested on real hardware.

## Python

The tests were developed by copying the library into python format in `./modelling/pidlib.py`. Another python script (`./modelling/firstorder.py`)showing some first order system modelling in python is also in the same directory, although this has not been altered from its original presentation [here](https://apmonitor.com/pdc/index.php/Main/ProportionalIntegralControl). It is retained as an example of a  potential method of simulating the library using FOSS software.

