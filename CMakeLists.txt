cmake_minimum_required(VERSION 2.8.12)
project(norfair_pp)
include_directories(${PYTHON_INCLUDE_DIRS}) 

find_package(pybind11 REQUIRED)
find_package (Eigen3 3.3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIRS} )

pybind11_add_module(norfair_pp MODULE pybinding.cpp)