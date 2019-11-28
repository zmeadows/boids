#pragma once

#include <optional>
#include <vector>

#include "ThreadPool.hpp"
#include "distribution.hpp"
#include "quad_tree.hpp"
#include "v2.hpp"

class QuadTree;

struct RuleParameters {
    struct {
        float com = 2.0;
        float density = 8.5;
        float confine = 50.f;
        float avg_vel = 3.f;
        float gravity;
    } value;

    struct {
        bool com = true;
        bool density = true;
        bool confine = true;
        bool avg_vel = true;
        bool gravity = false;
    } enabled;

    float max_force = 100.f;
    float max_vel = 60.f;
};

class BoidCollection {
    std::vector<V2> m_pos;
    std::vector<V2> m_vel;
    std::vector<V2> m_delta_avg_vel;
    std::vector<V2> m_delta_confine;
    std::vector<V2> m_delta_density;
    std::vector<V2> m_delta_center_of_mass;

    size_t m_count = 0;

    ThreadPool m_pool;

    void update_thread(float dt, const RuleParameters& params, const QuadTree& grid, size_t low_index,
                       size_t high_index);

public:
    BoidCollection(void);
    BoidCollection(size_t new_boid_count, Distribution& init_pos, Distribution& init_vel);

    void reset(size_t new_boid_count, Distribution& init_pos, Distribution& init_vel);
    void update(float dt, const RuleParameters& params, QuadTree& grid);

    inline size_t population(void) const { return m_count; }
    inline const std::vector<V2>& positions(void) const { return m_pos; }
    inline const std::vector<V2>& velocities(void) const { return m_vel; }
};
