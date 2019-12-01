#pragma once

#include <optional>
#include <vector>

#include "ThreadPool.hpp"
#include "distribution.hpp"
#include "quad_tree.hpp"
#include "v2.hpp"

class QuadTree;

enum RuleType {
    RT_CENTER_OF_MASS,
    RT_DENSITY,
    RT_CONFINE,
    RT_AVERAGE_VELOCITY,
    RT_GRAVITY,
    RT_RANDOM_NOISE,
    RT_MAX_FORCE,
    RT_MAX_VELOCITY,
    RT_COUNT
};

static constexpr const char* RULE_NAMES_NOSPACE[RT_COUNT] = {
    "Center_Of_Mass", "Density",      "Confine",   "Average_Velocity",
    "Gravity",        "Random_Noise", "Max_Force", "Max_Speed"};

static constexpr const char* RULE_NAMES[RT_COUNT] = {"Center Of Mass",   "Density",      "Confine",
                                                     "Average Velocity", "Gravity",      "Random Noise",
                                                     "Maximum Force",    "Maximum Speed"};

struct Rules {
    float values[RT_COUNT] = {
        10.f,  // Center Of Mass
        10.f,  // Density
        10.f,  // Confine
        10.f,  // Average Velocity
        10.f,  // Gravity
        10.f,  // Random Noise
        10.f,  // Maximum Force
        10.f,  // Maximum Velocity
    };

    bool toggles[RT_COUNT] = {
        true,  // Center Of Mass
        true,  // Density
        true,  // Confine
        true,  // Average Velocity
        true,  // Gravity
        true,  // Random Noise
        true,  // Maximum Force
        true,  // Maximum Velocity
    };
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

    void update_thread(const Rules& params, const QuadTree& grid, size_t low_index, size_t high_index);

public:
    BoidCollection(void);
    BoidCollection(size_t new_boid_count, Distribution& init_pos, Distribution& init_vel);

    void reset(size_t new_boid_count, Distribution& init_pos, Distribution& init_vel);
    void update(float dt, const Rules& params, QuadTree& grid);

    inline size_t population(void) const { return m_count; }
    inline const std::vector<V2>& positions(void) const { return m_pos; }
    inline const std::vector<V2>& velocities(void) const { return m_vel; }
};
