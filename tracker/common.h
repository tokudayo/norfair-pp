#pragma once

#include <vector>
#include "Eigen/Dense"

#define FLOAT_T double

using namespace Eigen;

typedef Matrix<FLOAT_T, 1, 2> Point;
typedef std::vector<Point> PointArray;
typedef Matrix<FLOAT_T, Dynamic, 2> PointMatrix;

inline PointMatrix PointArrayToMatrix(PointArray& src) {
    PointMatrix dst(src.size());
    for (size_t i = 0; i < src.size(); i++) {
        dst(i, 0) = src[i](0);
        dst(i, 1) = src[i](1);
    }
    return dst;
}


