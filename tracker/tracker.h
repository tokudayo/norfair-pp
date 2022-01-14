#pragma once
#include <vector>
#include <unordered_map>
#include <iostream>
#include "common.h"
#include "tracked_object.h"

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
        std::cout << "Tracker initialization successful\n";
    }

    ~Tracker() 
    {
        std::cout << "~Tracker()" << std::endl;
    }

    std::vector<int> Update(
        std::vector<std::vector<FLOAT_T>> detections, 
        int period = 1)
    {
        this->period = period;
        // Create instances of detections from the point array
        std::vector<Detection> dets = std::vector<Detection>();
        if (detections.size()) {
            for (int id = 0; id < detections.size(); id++) {
                dets.emplace_back((Point({detections[id][0], detections[id][1]})), id);
            }
        }
        // if (detections != nullptr) 
        // {
        //     for (size_t i = 0; i < detections->size(); i++) 
        //     {
        //         dets.emplace_back((*detections)[i], i);
        //     }
        // }
        std::cout << "Creaated instances of detections\n";

        // Update self tracked object list by removing those without inertia
        for (int i = 0; i < tracked_objects.size(); i++) 
        {
            if (!tracked_objects[i].has_inertia()) 
            {
                tracked_objects.erase(tracked_objects.begin() + i);
            }
        }
        std::cout << "Removed objects without inertia\n";

        // Update state of tracked objects
        for (auto& obj : tracked_objects) 
        {
            std::cout << "Updating object " << obj.ID << std::endl;
            obj.tracker_step();
            std::cout << "OK\n";
        }
        std::cout << "Updated state of tracked objects\n";

        // Divide tracked objects into 2 groups for matching.
        // I use pointers to avoid copying the objects, or maybe I don't need to.
        std::vector<TrackedObject*> initializing_objs;
        std::vector<TrackedObject*> initialized_objs;

        for (auto& obj : tracked_objects) 
        {
            if (obj.is_initializing()) 
            {
                initialized_objs.push_back(&obj);
            } else 
            {
                initializing_objs.push_back(&obj);
            }
        }

        // Match detections to initializing objects
        auto match_result_r1 = UpdateObjectInPlace(initializing_objs, dets);
        auto match_result_r2 = UpdateObjectInPlace(initialized_objs, dets);

        // Map results
        std::vector<int> map_result = std::vector<int>();
        for (int i = 0; i < dets.size(); i++) 
        {
            if (match_result_r1.find(i) != match_result_r1.end()) 
            {
                map_result.push_back(match_result_r1[i]);
            } else 
            {
                if (match_result_r2.find(i) != match_result_r2.end()) 
                {
                    map_result.push_back(match_result_r2[i]);
                } else 
                {
                    map_result.push_back(-1);
                }
            }
        }
        std::cout << "Matched detections to objects\n";

        // Create new tracked objects from yet unmatched detections
        int index = 0;
        std::cout << dets.size() << std::endl;
        for (auto& d : dets) 
        {
            std::cout << index++ << "\n";
            tracked_objects.emplace_back(
                d.point,
                hit_inertia_min, hit_inertia_max,
                init_delay, initial_hit_count,
                point_transience, period
            );
        }

        std::cout << "Created new tracked objects\n";

        // Finish initialization of new tracked objects
        for (auto& obj : tracked_objects) {
            if (!obj.is_initializing() && obj.ID == -1) {
                obj.ID = this->nextID++;
            }
        }

        std::cout << "Finished initialization of new tracked objects\n\n\n";

        return map_result;

    }

    /**
     * @brief Calculate and return index of matched det-obj pairs and remove
     * matched detections from the list.
     * 
     * @param objects 
     * @param detections 
     * @return tuple<vector<pair<int, int>>, vector<int>>
     *         A tuple of matched det-obj pairs and unmatched dets
     */
    std::unordered_map<int, int> UpdateObjectInPlace(
        const std::vector<TrackedObject*> &objects,
        std::vector<Detection> &detections) 
    {
        int num_dets = detections.size();
        int num_objs = objects.size();
        if (num_dets == 0) 
        {
            return std::unordered_map<int, int>();
        }
        if (num_objs == 0)
        {
            std::vector<int> unmatched_dets = std::vector<int>();
            for (int i = 0; i < num_dets; i++) 
            {
                unmatched_dets.push_back(detections[i].ID);
            }
            return std::unordered_map<int, int>();
        }

        // Calculate distance between detections and objects
        MatrixXf dist_matrix = MatrixXf::Zero(detections.size(), objects.size());
        for (int i = 0; i < detections.size(); i++) 
        {
            for (int j = 0; j < objects.size(); j++) 
            {
                dist_matrix(i, j) = (detections[i].point - objects[j]->estimate()).norm();
            }
        };

        // Need to merge it with the above part later now im too lazy to do it
        
        // To keep track of matched pairs
        std::unordered_map<int, int> det_obj_pairs;
        std::vector<int> unmatched_dets;
        std::vector<int> matched_dets, matched_objs = {};
        std::vector< std::pair<int, float> > dist_flattened;
        

        // Flatten the distance matrix and sort by distance in ascending order
        // In the future, maybe pass in a long vector instead of a matrix
        for (int i = 0; i < num_dets; i++) 
        {
            for (int j = 0; j < num_objs; j++) 
            {
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
            {
                break;
            }
            int det_idx = dist_flattened[i].first / dist_matrix.cols();
            int obj_idx = dist_flattened[i].first % dist_matrix.cols();
            if (std::find(matched_dets.begin(), matched_dets.end(), det_idx) == matched_dets.end()
                && std::find(matched_objs.begin(), matched_objs.end(), obj_idx) == matched_objs.end()) 
            {
                matched_dets.push_back(det_idx);
                matched_objs.push_back(obj_idx);
            }
        }

        // Find unmatched detections
        int idx = 0;
        for (int i = 0; i < num_dets; i++) 
        {
            if (std::find(matched_dets.begin(), matched_dets.end(), i) == matched_dets.end()) 
            {
                detections.erase(detections.begin() + idx);
            } else
            {
                idx++;
            }
        }

        // Map local indices to global indices
        for (int i = 0; i < matched_dets.size(); i++) 
        {
            det_obj_pairs[detections[matched_dets[i]].ID] = objects[matched_objs[i]]->ID;
        }

        return det_obj_pairs;
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