#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "norfair_pp/tracker.h"

namespace py = pybind11;

PYBIND11_MODULE(norfair_pp, m) {
    py::class_<Tracker>(m, "Tracker")
        .def(py::init<FLOAT_T, int, int, int, int>())
        .def("update", &Tracker::Update);
}
