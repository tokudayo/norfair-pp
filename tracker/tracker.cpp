#include <vector>
#include <unordered_set>
#include <iostream>
#include "common.h"
#include <tracked_object.h>

typedef std::vector<TrackedObject> TrackedObjArray;

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
        // this->tracked_objects = new std::vector<TrackedObject>();
        this->tracked_objects = TrackedObjArray();
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
        std::vector<TrackedObject*> initializing_objs;
        std::vector<TrackedObject*> initialized_objs;

        for (auto obj : tracked_objects) 
        {
            if (obj.is_initializing()) 
            {
                initialized_objs.push_back(&obj);
            } else 
            {
                initializing_objs.push_back(&obj);
            }
        }

        // unmatched_dets, det_obj_pair_r1 = 
        //     UpdateObjectInPlace(initializing_objs, detections);
                

    }

    std::tuple<std::vector<std::pair<int, int>>, std::vector<int>> 
    UpdateObjectInPlace(
        const std::vector<TrackedObject*> &objects,
        const std::vector<Detection> &detections) 
    {
        int num_dets = detections.size();
        int num_objs = objects.size();
        if (num_dets == 0) 
        {
            return std::make_tuple(std::vector<std::pair<int, int>>(), std::vector<int>());
        }
        if (num_objs == 0)
        {
            std::vector<int> unmatched_dets = std::vector<int>();
            for (int i = 0; i < num_dets; i++) 
            {
                unmatched_dets.push_back(i);
            }
            return std::make_tuple(std::vector<std::pair<int, int>>(), unmatched_dets);
        }

        // Calculate distance between detections and objects
        MatrixXf dist_matrix = MatrixXf::Zero(detections.size(), objects.size());
        for (int i = 0; i < detections.size(); i++) {
            for (int j = 0; j < objects.size(); j++) {
                dist_matrix(i, j) = (detections[i].point - objects[j]->estimate()).norm();
            }
        };        

        // Need to merge it with the above part later now im too lazy to do it
        
        // To keep track of matched pairs
        std::vector<std::pair<int,int>> det_obj_pairs;
        std::vector<int> unmatched_dets;
        std::unordered_set<int> matched_dets, matched_objs = {};
        std::vector< std::pair<int, float> > dist_flattened;
        

        // Flatten the distance matrix and sort by distance in ascending order
        // In the future, maybe pass in a long vector instead of a matrix
        for (int i = 0; i < num_dets; i++) {
            for (int j = 0; j < num_objs; j++) {
                dist_flattened.push_back(std::make_pair(i*num_objs + j, dist_matrix(i, j)));
            }
        }
        std::sort(dist_flattened.begin(), dist_flattened.end(), [](
            const std::pair<int, float> &a,
            const std::pair<int, float> &b) {
                return a.second < b.second;
            });

        // Matching
        for (int i = 0; i < dist_flattened.size(); i++) {
            if (dist_flattened[i].second > dist_threshold)
                break;
            int det_idx = dist_flattened[i].first / dist_matrix.cols();
            int obj_idx = dist_flattened[i].first % dist_matrix.cols();
            if (matched_dets.find(det_idx) == matched_dets.end()
                && matched_objs.find(obj_idx) == matched_objs.end()) 
            {
                matched_dets.insert(det_idx);
                matched_objs.insert(obj_idx);
            }
            det_obj_pairs.push_back(std::make_pair(det_idx, obj_idx));
        }

        // Find unmatched detections
        for (int i = 0; i < num_dets; i++) {
            if (matched_dets.find(i) == matched_dets.end()) {
                unmatched_dets.push_back(i);
            }
        }
        return {det_obj_pairs, unmatched_dets};
    } 



private:
    TrackedObjArray tracked_objects;
    FLOAT_T dist_threshold;
    int hit_inertia_min;
    int hit_inertia_max;
    int nextID;
    int init_delay;
    int initial_hit_count;
    int period;
    int point_transience;
    void UpdateObjectInPlace(std::vector<TrackedObject> objs, PointArray &detections);
};