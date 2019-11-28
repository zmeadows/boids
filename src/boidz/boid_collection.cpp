#include "boid_collection.hpp"

static constexpr float wrap_real(float x, float m) { return x - m * std::floor(x / m); }

BoidCollection::BoidCollection(size_t new_boid_count, Distribution& init_pos, Distribution& init_vel)
{
    reset(new_boid_count, init_pos, init_vel);
}

BoidCollection::BoidCollection(void)
{
    UniformDistribution d_pos(0.25f * WinProps::boid_span, 0.75f * WinProps::boid_span,
                              0.25f * WinProps::boid_span, 0.75f * WinProps::boid_span);
    UniformDistribution d_vel(-50.f, 50.f, -50.f, 50.f);
    reset(20000, d_pos, d_vel);
}

void BoidCollection::reset(size_t new_boid_count, Distribution& init_pos, Distribution& init_vel)
{
    for (std::vector<V2>* vec : {&m_pos, &m_vel, &m_delta_avg_vel, &m_delta_confine, &m_delta_density,
                                 &m_delta_center_of_mass}) {
        assert(vec->size() == m_count);

        if (new_boid_count < m_count) {
            vec->resize(new_boid_count);
        }
        else if (new_boid_count > m_count) {
            vec->reserve(new_boid_count);
        }

        vec->clear();
    }

    for (size_t i = 0; i < new_boid_count; i++) {
        m_pos.push_back(init_pos.sample());
        m_vel.push_back(init_vel.sample());
    }

    m_count = new_boid_count;
}

static std::vector<std::pair<size_t, size_t>> split_range(size_t thread_count, size_t object_count)
{
    const size_t N = thread_count;
    const size_t base = object_count / N;
    const size_t rem = object_count % N;

    std::vector<std::pair<size_t, size_t>> result;
    result.reserve(base);

    size_t marker = 0;
    for (size_t i = 0; i < rem; i++) {
        const size_t low = marker;
        const size_t high = marker + base + 1;
        marker = high;

        result.push_back({low, high});
    }

    for (size_t i = rem; i < N; i++) {
        const size_t low = marker;
        const size_t high = marker + base;
        marker = high;
        result.push_back({low, high});
    }

    return result;
}

void BoidCollection::update_thread(float dt, const RuleParameters& params, const QuadTree& grid,
                                   size_t low_index, size_t high_index)
{
    static thread_local std::vector<PseudoBoid> neighbors;

    for (size_t id = low_index; id < high_index; id++) {
        grid.get_pseudoboid_neighbors(m_pos[id], neighbors);

        const V2 pos = m_pos[id];
        const V2 vel = m_vel[id];

        // remove self from total
        V2 pos_sum = -1.f * pos;
        V2 vel_sum = -1.f * vel;
        float weight_sum = -1.f;
        V2 dens_accum = V2::null();

        for (const PseudoBoid& pb : neighbors) {
            pos_sum += pb.weight * pb.pos;
            vel_sum += pb.weight * pb.vel;
            weight_sum += pb.weight;

            const float separation = distance_sq(pos, pb.pos);
            if (separation > 1e-7) {
                dens_accum += pb.weight / separation * (pos - pb.pos);
            }
        }

        if (weight_sum > 0.f) {
            const float inverted_weight_sum = 1.f / weight_sum;
            const V2 avg_vel = vel_sum * inverted_weight_sum;
            const V2 avg_pos = pos_sum * inverted_weight_sum;

            if (params.enabled.avg_vel) {
                m_delta_avg_vel[id] = params.value.avg_vel * avg_vel;
            }

            if (params.enabled.density) {
                m_delta_density[id] = params.value.density * dens_accum;
            }

            if (params.enabled.com) {
                m_delta_center_of_mass[id] = params.value.com * (avg_pos - pos);
            }
        }
        else {
            m_delta_avg_vel[id] = V2::null();
            m_delta_density[id] = V2::null();
            m_delta_center_of_mass[id] = V2::null();
        }

        if (params.enabled.confine) {
            // @FIXME: Use smaller delta time increments when close to edge

            const float x = std::min(WinProps::boid_span - 1e-3F, std::max(1e-3F, pos.x));
            const float y = std::min(WinProps::boid_span - 1e-3F, std::max(1e-3F, pos.y));
            const float cp = params.value.confine;
            const float s = WinProps::boid_span;

            const float confine_x = 1e3 * cp * (1.f / std::pow(x, 4.f) - 1.f / std::pow(x - s, 4.f));
            const float confine_y = 1e3 * cp * (1.f / std::pow(y, 4.f) - 1.f / std::pow(y - s, 4.f));

            m_delta_confine[id] = {confine_x, confine_y};
        }
    }
}

void BoidCollection::update(float dt, const RuleParameters& params, QuadTree& grid)
{
    grid.insert(*this);

    const auto boid_ranges = split_range(m_pool.nthreads(), m_count);

    std::vector<std::future<void>> results;
    results.reserve(boid_ranges.size());

    for (const auto& r : boid_ranges) {
        results.emplace_back(m_pool.enqueue(
            [&](void) -> void { this->update_thread(dt, params, grid, r.first, r.second); }));
    }

    for (auto&& r : results) {
        r.get();
    }

    for (size_t id = 0; id < m_count; id++) {
        V2 dv = V2::null();

        // apply only the rules that have been turned on
        if (params.enabled.avg_vel) dv += m_delta_avg_vel[id];
        if (params.enabled.confine) dv += m_delta_confine[id];
        if (params.enabled.density) dv += m_delta_density[id];
        if (params.enabled.com) dv += m_delta_center_of_mass[id];
        if (params.enabled.gravity) dv.y -= params.value.gravity * dt;

        // clamp the force vector magnitude to the user-specified maximum force.
        const float dv_mag = dv.magnitude();
        if (dv_mag > params.max_force) {
            dv *= params.max_force / dv_mag;
        }

        V2& vel = m_vel[id];
        vel += dv;
        vel = clamp(vel, params.max_vel);

        V2& pos = m_pos[id];
        pos = pos + dt * vel;

        if (!WinProps::is_boid_onscreen(pos)) {
            pos = {10.f, 10.f};
            vel = {10.f, 10.f};
        }
    }
}
