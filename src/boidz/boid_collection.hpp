#pragma once

#include <vector>

#include "distribution.hpp"
#include "quad_tree.hpp"
#include "v2.hpp"

class QuadTree;

struct RuleParameters {
    float center_of_mass;
    float densitiy;
    float confine;
    float avg_vel;
    float gravity;

    RuleParameters(void);
};

class BoidCollection {
    std::vector<V2> m_pos;
    std::vector<V2> m_vel;
    std::vector<V2> m_delta_avg_vel;
    std::vector<V2> m_delta_confine;
    std::vector<V2> m_delta_density;
    std::vector<V2> m_delta_center_of_mass;

    size_t m_count = 0;

    static const float s_max_vel;
    static const float s_max_force;

public:
    BoidCollection(void);
    BoidCollection(size_t new_boid_count, Distribution& init_pos, Distribution& init_vel);

    void reset(size_t new_boid_count, Distribution& init_pos, Distribution& init_vel);
    void update(float dt, const RuleParameters& params, QuadTree& grid);

    inline size_t population(void) const { return m_count; }
    inline const std::vector<V2>& positions(void) const { return m_pos; }
    inline const std::vector<V2>& velocities(void) const { return m_vel; }
};
