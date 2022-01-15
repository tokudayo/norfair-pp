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
        if (init_delay >= 0) 
        { 
            this->init_delay = init_delay;
        } else 
        {
            this->init_delay = (hit_inertia_max - hit_inertia_min) / 2;
        }
        if (initial_hit_count == -1) 
        {
            this->initial_hit_count = this->hit_inertia_max/2;
        } else 
        {
            this->initial_hit_count = initial_hit_count;
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
        std::cout << "Tracked objects size: " << tracked_objects.size() << " detections size: " << detections.size() << std::endl;
        this->period = period;
        // Create instances of detections from the point array
        std::vector<Detection> dets = std::vector<Detection>();
        if (detections.size()) {
            for (int id = 0; id < detections.size(); id++) {
                dets.emplace_back((Point({detections[id][0], detections[id][1]})), id);
            }
        }

        // Update self tracked object list by removing those without inertia
        int i = 0;
        while (i < tracked_objects.size()) {
            if (!tracked_objects[i].has_inertia()) {
                std::cout << "Erasing an object lol\n";
                tracked_objects.erase(tracked_objects.begin() + i);
            } else {
                i++;
            }
        }

        // Update state of tracked objects
        for (auto& obj : tracked_objects) 
        {
            std::cout << "Updating object " << obj.ID << std::endl;
            obj.tracker_step();
        }

        // Divide tracked objects into 2 groups for matching.
        // I use pointers to avoid copying the objects, or maybe I don't need to.
        std::vector<TrackedObject*> initializing_objs;
        std::vector<TrackedObject*> initialized_objs;

        for (auto& obj : tracked_objects) 
        {
            std::cout << "Checking object state\n";
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


        // Finish initialization of new tracked objects
        for (auto& obj : tracked_objects) {
            if (!obj.is_initializing() && obj.ID == -1) {
                std::cout << "NEW OBJECT\n";
                obj.ID = this->nextID++;
            }
        }


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
        std::cout << "Updateing objects with " << num_objs << " objects and " << num_dets << " detections\n";
        
        // Handle special cases
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

        // Trivial case
        std::vector< std::pair<int, double> > dist_flattened;
        for (int i = 0; i < num_dets; i++) 
        {
            for (int j = 0; j < num_objs; j++) 
            {
                std::cout << "Point: " << detections[i].point << " Object: " << objects[j]->estimate() << std::endl;
                dist_flattened.emplace_back(
                    std::make_pair(
                        i*num_objs + j,
                        (detections[i].point - objects[j]->estimate()).norm()
                    )
                );
            }
        }

        std::cout << "Flattened distances " << dist_flattened.size() << std::endl;
        
        // To keep track of matched pairs
        std::unordered_map<int, int> det_obj_pairs;
        std::vector<int> unmatched_dets;
        std::vector<int> matched_dets, matched_objs = {};
        

        // Sort by distance in ascending order
        std::sort(dist_flattened.begin(), dist_flattened.end(), [](
            const std::pair<int, double> &a,
            const std::pair<int, double> &b) {
                return a.second < b.second;
            });

        std::cout << "Sorted\n";
        for (auto &p : dist_flattened) 
        {
            std::cout << p.first << " " << p.second << "   ";
        }
        std::cout << std::endl;

        // Matching
        for (int i = 0; i < dist_flattened.size(); i++) {
            if (dist_flattened[i].second > dist_threshold) 
            {
                break;
            }
            int det_idx = dist_flattened[i].first / num_objs;
            int obj_idx = dist_flattened[i].first % num_objs;
            if (std::find(matched_dets.begin(), matched_dets.end(), det_idx) == matched_dets.end()
                && std::find(matched_objs.begin(), matched_objs.end(), obj_idx) == matched_objs.end()) 
            {
                matched_dets.push_back(det_idx);
                matched_objs.push_back(obj_idx);
                std::cout << "Updating object " << objects[obj_idx]->ID << " with detection " << detections[det_idx].ID << "\n";
                objects[obj_idx]->Hit(detections[det_idx].point);
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