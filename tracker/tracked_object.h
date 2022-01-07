#include "common.h"
#include "kalman.h"

class TrackedObject {
public:
    int hit_inertia_min, hit_inertia_max, init_delay, initial_hit_count, 
        point_hit_inertia_min, point_hit_inertia_max, intial_period, hit_counter,
        point_hit_counter, dim_z, ID;

    KalmanFilter* filter;

    TrackedObject(Point initial_detection, int hit_intertia_min,
                  int hit_intertia_max, int init_delay,
                  int initial_hit_count, int point_transience,
                  int period)
    {
        this->hit_inertia_min = hit_intertia_min;
        this->hit_inertia_max = hit_intertia_max;
        this->init_delay = init_delay;
        this->initial_hit_count = initial_hit_count;
        this->point_hit_inertia_min = floor(hit_inertia_min / point_transience);
        this->point_hit_inertia_max = floor(hit_inertia_max / point_transience);
        if (this->point_hit_inertia_max - this->point_hit_inertia_min)
        {
            this->point_hit_inertia_max = this->point_hit_inertia_min + period;
        }
        this->intial_period = period;
        this->hit_counter = hit_inertia_min + period;
        this->point_hit_counter = this->point_hit_inertia_min;
        FLOAT_T last_distance = -1.;
        // this->age = 0;
        this->ID = -1;
        this->filter = new KalmanFilter(initial_detection);
        this->dim_z = 2;
        m_is_initializing_flag = true;
        m_detected_at_least_once = false;
    };

    ~TrackedObject()
    {
        delete filter;
    }

    void tracker_step()
    {
        hit_counter -= 1;
        point_hit_counter -= 1;
        filter->Predict();
    }

    bool is_initializing()
    {
        if (m_is_initializing_flag && hit_counter > hit_inertia_min + init_delay)
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
        return filter->x.transpose()(seq(fix<0>, fix<0>), seq(fix<0>, fix<1>));
    }

    void Hit(Point detection, int det_id, int period = 1)
    {
        if (hit_counter < hit_inertia_max)
        {
            hit_counter += 2*period;
        }
        point_hit_counter += 2*period;
        point_hit_counter = std::max(0, point_hit_counter);
        point_hit_counter = std::min(point_hit_counter, point_hit_inertia_max);
        filter->Update(detection);
        if (!m_detected_at_least_once)
        {
            m_detected_at_least_once = true;
            filter->x(2, 0) = 0;
            filter->x(2, 1) = 0;
        }

    }

private:
    bool m_is_initializing_flag;
    bool m_detected_at_least_once;
};