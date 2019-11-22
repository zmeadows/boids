#pragma once

#include <vector>

#include "boid_collection.hpp"
#include "props.hpp"
#include "v2.hpp"

class BoidCollection;

struct PseudoBoid {
    V2 pos = V2::null();
    V2 vel = V2::null();
    float weight = 0.f;

    PseudoBoid(void) = default;

    PseudoBoid(V2 _pos, V2 _vel, float _weight) : pos(_pos), vel(_vel), weight(_weight)
    {
        // TODO: deal with this error without failing
        assert(boid_coordinates::is_valid(pos));
    }
};

class QuadTree {
    class Node {
        std::vector<V2> m_positions;   // positions of boids in this node
        std::vector<V2> m_velocities;  // velocities of boids in this node

        // pseudo boid computed via the average position/velocity of this nodes members
        // it is only ever updated with a call to recompute_pseudoboid
        // we cache this pseudo boid to avoid computing it
        // multiple times (for each neighbor request)
        PseudoBoid m_pseudo_boid;

    public:
        inline size_t population(void) const
        {
            assert(m_positions.size() == m_velocities.size());
            return m_positions.size();
        }

        inline void recompute_pseudoboid(void)
        {
            const float count = static_cast<float>(this->population());
            assert(static_cast<size_t>(count) == this->population());

            const V2 avg_pos = average_of(m_positions);
            const V2 avg_vel = average_of(m_velocities);

            m_pseudo_boid = PseudoBoid(avg_pos, avg_vel, count);
        };

        inline void insert(V2 pos, V2 vel)
        {
            m_positions.push_back(pos);
            m_velocities.push_back(vel);
        }

        inline void clear(void)
        {
            m_positions.clear();
            m_velocities.clear();
        }

        inline const PseudoBoid& pseudoboid(void) const { return m_pseudo_boid; }

        inline const std::vector<V2>& positions(void) const { return m_positions; }
        inline const std::vector<V2>& velocities(void) const { return m_velocities; }
    };

    std::vector<Node> m_nodes;
    int m_nodes_per_axis;
    int m_node_count;  // m_nodes_per_axis ^ 2

    int position_to_node_index(V2 pos) const;
    inline Node& position_to_node(V2 pos) { return m_nodes[position_to_node_index(pos)]; }

    // in 'fine grain' cells we treat each boid as a separate PseudoBoid neighbor
    static constexpr int s_fine_grain_node_limit = 1;
    // in 'coarse grain' cells we group together all boids into a single PseudoBoids
    static constexpr int s_coarse_grain_node_limit = 3;

public:
    QuadTree(void) : QuadTree(16) {}

    QuadTree(int nodes_per_axis)
        : m_nodes(nodes_per_axis * nodes_per_axis),
          m_nodes_per_axis(nodes_per_axis),
          m_node_count(nodes_per_axis * nodes_per_axis)
    {
        assert(nodes_per_axis > 1);
    }

    // TODO: just pass vector<V2>'s
    void insert(const BoidCollection& boids);
    void get_pseudoboid_neighbors(V2 pos, std::vector<PseudoBoid>& neighbors) const;
};
