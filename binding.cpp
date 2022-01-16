#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "tracker/tracker.h"

namespace py = pybind11;

PYBIND11_MODULE(tracker_cpp, m) {
    py::class_<Tracker>(m, "Tracker")
        .def(py::init<FLOAT_T, int, int, int, int, int>())
        .def("update", &Tracker::Update);
    // py::class_<KalmanFilter>(m, "KalmanFilter")
    //     .def(py::init<double, double>())
    //     .def("predict", &KalmanFilter::Predict)
    //     .def("update", &KalmanFilter::Update_)
    //     .def("estimate", &KalmanFilter::Estimate);
}
