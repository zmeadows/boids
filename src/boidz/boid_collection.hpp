#pragma once

#include <optional>
#include <vector>

#include "distribution.hpp"
#include "quad_tree.hpp"
#include "v2.hpp"

class QuadTree;

struct RuleParameters {
    struct {
        float com;
        float density;
        float confine;
        float avg_vel;
        float gravity;
    } value;

    struct {
        bool com;
        bool density;
        bool confine;
        bool avg_vel;
        bool gravity;
    } enabled;

    float max_force = 500.f;
    float max_vel = 200.f;
};

class BoidCollection {
    std::vector<V2> m_pos;
    std::vector<V2> m_vel;
    std::vector<V2> m_delta_avg_vel;
    std::vector<V2> m_delta_confine;
    std::vector<V2> m_delta_density;
    std::vector<V2> m_delta_center_of_mass;

    size_t m_count = 0;

public:
    BoidCollection(void);
    BoidCollection(size_t new_boid_count, Distribution& init_pos, Distribution& init_vel);

    void reset(size_t new_boid_count, Distribution& init_pos, Distribution& init_vel);
    void update(float dt, const RuleParameters& params, QuadTree& grid);

    inline size_t population(void) const { return m_count; }
    inline const std::vector<V2>& positions(void) const { return m_pos; }
    inline const std::vector<V2>& velocities(void) const { return m_vel; }
};
