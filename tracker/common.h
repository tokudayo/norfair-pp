#pragma once

#include <vector>
#include "Eigen/Dense"

#define FLOAT_T double

using namespace Eigen;

typedef Matrix<FLOAT_T, 1, 2> Point;
typedef std::vector<Point> PointArray;
typedef Matrix<FLOAT_T, Dynamic, 2> PointMatrix;

struct Detection
{
    Point point;
    int ID;
    
    Detection(Point point, int ID)
    {
        this->point = point;
        this->ID = ID;
    }
};


