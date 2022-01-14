#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "tracker/tracker.h"

namespace py = pybind11;

PYBIND11_MODULE(tracker_cpp, m) {
    py::class_<Tracker>(m, "Tracker")
        .def(py::init<FLOAT_T, int, int, int, int, int>())
        .def("update", &Tracker::Update);
}

// g++ -O3 -Wall -shared -std=c++11 -fPIC $(python3 -m pybind11 --includes) binding.cpp -o tracker_cpp$(python3-config --extension-suffix)
