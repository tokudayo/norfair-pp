#pragma once
#include "common.h"
#include "kalman.h"


class TrackedObject
{
public:
    int hit_inertia_min, hit_inertia_max, init_delay, 
    initial_hit_count, hit_counter, ID;
    KalmanFilter filter;

    TrackedObject(const Point& initial_detection, int hit_intertia_min,
                  int hit_intertia_max, int init_delay,
                  int initial_hit_count, int period):
        hit_inertia_min(hit_intertia_min), hit_inertia_max(hit_intertia_max),
        init_delay(init_delay), initial_hit_count(initial_hit_count), 
        hit_counter(hit_inertia_min + period), ID(-1),
        filter(KalmanFilter(initial_detection)),
        m_is_initializing_flag(true), m_detected_at_least_once(false)
    {
    };

    ~TrackedObject()
    {
    }

    void tracker_step()
    {
        hit_counter -= 1;
        filter.Predict();
    }

    bool is_initializing()
    {
        if (m_is_initializing_flag 
            && hit_counter > hit_inertia_min + init_delay)
        {
            m_is_initializing_flag = false;
            hit_counter = initial_hit_count;
        }
        return m_is_initializing_flag;
    }

    bool has_inertia()
    {
        return hit_counter >= hit_inertia_min;
    }

    Point estimate()
    {
        return filter.x.transpose()(seq(fix<0>, fix<0>), seq(fix<0>, fix<1>));
    }

    void Hit(const Point &detection, int period = 1)
    {
        if (hit_counter < hit_inertia_max)
        {
            hit_counter += 2*period;
        }
        filter.Update(detection);
        if (!m_detected_at_least_once)
        {
            m_detected_at_least_once = true;
            filter.x(2, 0) = 0;
            filter.x(3, 0) = 0;
        }
    }

private:
    bool m_is_initializing_flag;
    bool m_detected_at_least_once;
};