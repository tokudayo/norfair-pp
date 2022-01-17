# Norfair C++
This is a C++ implementation of [Techainer's Norfair](https://github.com/Techainer/norfair), which originates from [Norfair](https://github.com/tryolabs/norfair), a library for real-time 2D object tracking.


Its function is identical to Techainer's version:

> It assigns a track id to each object instead of returning a list of new objects after tracking.
>
> This is optimized for the use case when there is 1 representative point per detection.

This contains some more optimizations in tracker update functions and the use of Kalman filter. Overall, the Python binding for this C++ implementation offers a ~10x speedup compared to Techainer's fork, which was already much faster than the original Norfair (for the above use case).

## Python binding

Clone this repository:
```bash
git clone https://github.com/20toduc01/norfair-cpp.git
cd norfair-cpp
```

Install pybind11:
```bash
pip3 install pybind11
```

Install Eigen3:
```bash
curl https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
unzip Eigen
```

Compile Python binding:
```bash
c++ -O3 -Wall -shared -std=c++11 -fPIC $(python3 -m pybind11 --includes) -I eigen-3.4.0/Eigen pybinding.cpp -o norfair_cpp$(python3-config --extension-suffix)
```