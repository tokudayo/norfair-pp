#pragma once
#include <vector>
#include <unordered_map>
#include <iostream>
#include "common.h"
#include "tracked_object.h"


struct Detection
{
    Point point;
    int ID;
    
    Detection(Point point, int ID):
        point(point), ID(ID)
    {
    }
};

class Tracker 
{
public:
    std::vector<TrackedObject> tracked_objects;
    FLOAT_T dist_threshold;
    int hit_inertia_min;
    int hit_inertia_max;
    int nextID;
    int init_delay;
    int initial_hit_count;
    int period;
    int point_transience;

    Tracker(FLOAT_T dist_threshold,
            int hit_inertia_min = 10,
            int hit_inertia_max = 25,
            int init_delay = -1,
            int initial_hit_count = -1,
            int point_transience = -1):
        tracked_objects(std::vector<TrackedObject>()), dist_threshold(dist_threshold),
        hit_inertia_min(hit_inertia_min), hit_inertia_max(hit_inertia_max),
        nextID(0), period(1), point_transience(point_transience)
    {
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
    }

    ~Tracker()
    {
    }

    /**
     * @brief Update the state of tracker with new detections and return a list
     * of object ID corresponding to each box
     * 
     * @param detection
     * @param period
     * @return A list of numbers corresponding to the ID of each detection.
     * A number is non-negative if a match is found or -1 otherwise.
     */
    std::vector<int> Update(
        std::vector<std::array<FLOAT_T, 2>> detections, 
        int period = 1)
    {
        this->period = period;
        // Create instances of detections from the point array
        std::vector<Detection> dets = std::vector<Detection>();
        if (detections.size())
        {
            for (size_t id = 0; id < detections.size(); id++)
            {
                dets.emplace_back(
                    Point({detections[id][0], detections[id][1]}), id
                );
            }
        }

        // Update self tracked object list by removing those without inertia
        size_t idx = 0;
        while (idx < tracked_objects.size())
        {
            if (!tracked_objects[idx].has_inertia())
            {
                tracked_objects.erase(tracked_objects.begin() + idx);
            }
            else idx++;
        }

        // Update state of tracked objects
        for (auto& obj : tracked_objects) 
        {
            obj.tracker_step();
        }

        // Divide tracked objects into 2 groups for matching.
        std::vector<TrackedObject*> initializing_objs;
        std::vector<TrackedObject*> initialized_objs;

        for (auto& obj : tracked_objects)
        {
            if (obj.is_initializing())
            {
                initializing_objs.push_back(&obj);
            } else
            {
                initialized_objs.push_back(&obj);
            }
        }

        // Match detections to initializing objects
        auto match_result_r1 = UpdateObjectInPlace(initialized_objs, dets);
        auto match_result_r2 = UpdateObjectInPlace(initializing_objs, dets);

        // Map results
        std::vector<int> map_result = std::vector<int>();
        for (size_t i = 0; i < detections.size(); i++)
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
        for (auto& d : dets)
        {
            tracked_objects.emplace_back(
                d.point,
                hit_inertia_min, hit_inertia_max,
                init_delay, initial_hit_count,
                point_transience, period
            );
        }

        // Finish initialization of new tracked objects
        for (auto& obj : tracked_objects)
        {
            if (!obj.is_initializing() && obj.ID == -1)
            {
                obj.ID = this->nextID++;
            }
        }

        return map_result;

    }

    /**
     * @brief Calculate and return index of matched det-obj pairs. Also remove
     * matched detections from the detection list.
     * 
     * @param objects
     * @param detections
     * @return A detection ID <--> object ID map.
     */
    std::unordered_map<int, int> UpdateObjectInPlace(
        const std::vector<TrackedObject*> &objects,
        std::vector<Detection> &detections) 
    {
        size_t num_dets = detections.size();
        size_t num_objs = objects.size();
        
        // Handle special cases
        if (num_dets == 0) 
        {
            return std::unordered_map<int, int>();
        }

        if (num_objs == 0)
        {
            std::vector<int> unmatched_dets = std::vector<int>();
            for (size_t i = 0; i < num_dets; i++) 
            {
                unmatched_dets.push_back(detections[i].ID);
            }
            return std::unordered_map<int, int>();
        }

        // Trivial case
        std::vector< std::pair<int, FLOAT_T> > dist_flattened;
        for (size_t i = 0; i < num_dets; i++) 
        {
            for (size_t j = 0; j < num_objs; j++) 
            {
                dist_flattened.emplace_back(
                    std::make_pair(
                        i*num_objs + j,
                        (detections[i].point - objects[j]->estimate()).norm()
                    )
                );
            }
        }
        
        // To keep track of matched pairs
        std::unordered_map<int, int> det_obj_pairs;
        std::vector<int> unmatched_dets;
        std::vector<int> matched_dets, matched_objs = {};

        // Sort by distance in ascending order
        std::sort(dist_flattened.begin(), dist_flattened.end(), [](
            const std::pair<int, FLOAT_T> &a,
            const std::pair<int, FLOAT_T> &b) {
                return a.second < b.second;
            });

        // Matching
        for (size_t i = 0; i < dist_flattened.size(); i++)
        {
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
                objects[obj_idx]->Hit(detections[det_idx].point);
            }
        }

        // Map local indices to global indices
        for (size_t i = 0; i < matched_dets.size(); i++) 
        {
            det_obj_pairs[detections[matched_dets[i]].ID] = objects[matched_objs[i]]->ID;
        }

        // Remove matched detections
        int idx = 0;
        for (size_t i = 0; i < num_dets; i++) 
        {
            if (std::find(matched_dets.begin(), matched_dets.end(), i) != matched_dets.end()) 
            {
                detections.erase(detections.begin() + idx);
            } else
            {
                idx++;
            }
        }

        return det_obj_pairs;
    }
};