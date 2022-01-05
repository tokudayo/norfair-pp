#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "Eigen/Dense"

#define FLOAT_T float

using namespace Eigen;
namespace py = pybind11;

typedef std::vector<Point> PointArray;
typedef Matrix<FLOAT_T, Dynamic, 2> PointMatrix;

struct Point {
    FLOAT_T x, y;
    Point(FLOAT_T x, FLOAT_T y) : x(x), y(y) {};
};

inline PointMatrix PointArrayToMatrix(PointArray& src) {
    PointMatrix dst(src.size());
    for (size_t i = 0; i < src.size(); i++) {
        dst(i, 0) = src[i].x;
        dst(i, 1) = src[i].y;
    }
    return dst;
}

class Tracker 
{
public:

    Tracker(FLOAT_T dist_threshold,
            int hit_inertia_min = 10,
            int hit_inertia_max = 25,
            int init_delay = -1,
            int initial_hit_count = -1,
            int point_transience = -1) 
    {
        std::cout << "Tracker()" << std::endl;
        this->tracked_objects = new std::vector<TrackedObject>();
        this->hit_inertia_max = hit_inertia_max;
        this->hit_inertia_min = hit_inertia_min;
        this->nextID = 0;
        if (init_delay >= 0 && init_delay < hit_inertia_max - hit_inertia_min) 
        {
            this->init_delay = init_delay;
        } else 
        {
            this->init_delay = (hit_inertia_max - hit_inertia_min) / 2;
        }
        this->dist_threshold = dist_threshold;
        this->point_transience = point_transience;
    }

    ~Tracker() 
    {
        std::cout << "~Tracker()" << std::endl;
    }

    void Update(PointArray* detections = nullptr, int period = 1)
    {
        this->period = period;

        if (detections != nullptr) 
        {    
        }
        std::vector<TrackedObject> a;
        // Update tracked object list
        for (int i = 0; i < tracked_objects.size(); i++) 
        {
            if (tracked_objects[i].has_inertia()) 
            {
                tracked_objects.erase(tracked_objects.begin() + i);
            }
        }

        // Update tracker
        for (auto obj : tracked_objects) 
        {
            obj.tracker_step();
        }
        std::vector<TrackedObject> initializing_objs;
        std::vector<TrackedObject> initialized_objs;

        for (auto obj : tracked_objects) 
        {
            if (obj.is_initializing()) 
            {
                initialized_objs.push_back(obj);
            } else 
            {
                initializing_objs.push_back(obj);
            }
        }

        unmatched_dets, det_obj_pair_r1 = 
            UpdateObjectInPlace(initializing_objs, detections);
                

    }



private:
    std::vector<TrackedObject> tracked_objects;
    FLOAT_T dist_threshold;
    int hit_inertia_min;
    int hit_inertia_max;
    int nextID;
    int init_delay;
    int initial_hit_count;
    int period;
    int point_transience;
    void UpdateObjectInPlace(std::vector<TrackedObject> objs, PointArray)
};


PYBIND11_MODULE(testmodule, m) {
    py::class_<Tracker>(m, "Tracker")
        .def(py::init<>());
}