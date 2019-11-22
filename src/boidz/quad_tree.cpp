#include "quad_tree.hpp"

// @OPTIMIZE: there is a bit hack for doing this in ~1 cpu cycle
int QuadTree::position_to_node_index(V2 pos) const
{
    assert(boid_coordinates::is_valid(pos));
    const float node_span = boid_coordinates::span / static_cast<float>(m_nodes_per_axis);
    const int node_x = static_cast<int>(std::floor(pos.x / node_span));
    const int node_y = static_cast<int>(std::floor(pos.y / node_span));
    const int node_index = m_nodes_per_axis * node_y + node_x;

    assert(node_index < m_node_count);

    return node_index;
}

void QuadTree::get_pseudoboid_neighbors(V2 pos, std::vector<PseudoBoid>& neighbors) const
{
    neighbors.clear();

    // node index of the boid we're interested in
    const int focus_node_index = position_to_node_index(pos);

    // check if node index is actually part of the grid
    // matters when looking for neighboring nodes at a corner node
    auto is_valid_node_index = [&](int idx) { return idx >= 0 && idx < m_node_count; };

    // loop over the fine grain nodes directly adjacent to the node of interest,
    // creating a single PseudoBoid for each nearby boid
    for (int i = -s_fine_grain_node_limit; i <= s_fine_grain_node_limit; i++) {
        for (int j = -s_fine_grain_node_limit; j <= s_fine_grain_node_limit; j++) {
            const int node_index = focus_node_index + m_nodes_per_axis * j + i;
            if (is_valid_node_index(node_index)) {
                const Node& node = m_nodes[node_index];
                const size_t pop = node.population();
                const std::vector<V2>& positions = node.positions();
                const std::vector<V2>& velocities = node.velocities();
                for (size_t bid = 0; bid < pop; bid++) {
                    neighbors.emplace_back(positions[bid], velocities[bid], 1.f);
                }
            }
        }
    }

    // helper lambda for looping over the proceeding coarse grain cells,
    // while skipping over the fine grain cells we just looped over above.
    auto advance_coarse_cell_index = [](int idx) -> int {
        if (idx == -s_fine_grain_node_limit - 1) {
            return s_fine_grain_node_limit + 1;
        }
        else {
            return idx + 1;
        }
    };

    // loop over the fine grain nodes directly adjacent to the node of interest,
    // creating a single PseudoBoid for each nearby boid
    for (int i = -s_coarse_grain_node_limit; i <= s_coarse_grain_node_limit;
         i = advance_coarse_cell_index(i)) {
        for (int j = -s_coarse_grain_node_limit; j <= s_coarse_grain_node_limit;
             j = advance_coarse_cell_index(j)) {
            const int node_index = focus_node_index + m_nodes_per_axis * j + i;
            if (is_valid_node_index(node_index)) {
                const Node& node = m_nodes[node_index];
                // don't append zero-weight pseudoboids for empty nodes
                if (node.population() > 0) {
                    neighbors.emplace_back(node.pseudoboid());
                }
            }
        }
    }
}

void QuadTree::insert(const BoidCollection& boids)
{
    for (Node& node : m_nodes) {
        node.clear();
    }

    const std::vector<V2>& positions = boids.positions();
    const std::vector<V2>& velocities = boids.velocities();
    const size_t boid_count = boids.population();

    for (size_t i = 0; i < boid_count; i++) {
        const V2 pos = positions[i];
        const V2 vel = velocities[i];
        this->position_to_node(pos).insert(pos, vel);
    }

    for (Node& node : m_nodes) {
        node.recompute_pseudoboid();
    }
}
