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
    reset(30000, d_pos, d_vel);
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

void BoidCollection::update_thread(const Rules& params, const QuadTree& grid, size_t low_index,
                                   size_t high_index)
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
            const float separation = distance_sq(pos, pb.pos);
            if (separation < grid.effect_radius_squared()) {
                pos_sum += pb.weight * pb.pos;
                vel_sum += pb.weight * pb.vel;
                weight_sum += pb.weight;

                if (separation > 1e-7) {
                    dens_accum += pb.weight / separation * (pos - pb.pos);
                }
            }
        }

        const auto toggles = params.toggles;
        const auto values = params.values;

        if (weight_sum > 0.f) {
            const float inverted_weight_sum = 1.f / weight_sum;
            const V2 avg_vel = vel_sum * inverted_weight_sum;
            const V2 avg_pos = pos_sum * inverted_weight_sum;

            if (toggles[RT_AVERAGE_VELOCITY]) {
                m_delta_avg_vel[id] = values[RT_AVERAGE_VELOCITY] * avg_vel;
            }

            if (toggles[RT_DENSITY]) {
                m_delta_density[id] = values[RT_DENSITY] * dens_accum;
            }

            if (toggles[RT_CENTER_OF_MASS]) {
                m_delta_center_of_mass[id] = values[RT_CENTER_OF_MASS] * (avg_pos - pos);
            }
        }
        else {
            m_delta_avg_vel[id] = V2::null();
            m_delta_density[id] = V2::null();
            m_delta_center_of_mass[id] = V2::null();
        }

        if (toggles[RT_CONFINE]) {
            // @FIXME: Use smaller delta time increments when close to edge

            const float x = std::min(WinProps::boid_span - 1e-3F, std::max(1e-3F, pos.x));
            const float y = std::min(WinProps::boid_span - 1e-3F, std::max(1e-3F, pos.y));
            const float cp = values[RT_CONFINE];
            const float s = WinProps::boid_span;

            const float confine_x = 1e3 * cp * (1.f / std::pow(x, 4.f) - 1.f / std::pow(x - s, 4.f));
            const float confine_y = 1e3 * cp * (1.f / std::pow(y, 4.f) - 1.f / std::pow(y - s, 4.f));

            m_delta_confine[id] = {confine_x, confine_y};
        }
    }
}

void BoidCollection::update(float dt, const Rules& params, QuadTree& grid)
{
    grid.insert(*this);

    const auto boid_ranges = split_range(m_pool.nthreads(), m_count);

    std::vector<std::future<void>> results;
    results.reserve(boid_ranges.size());

    for (const auto& r : boid_ranges) {
        results.emplace_back(
            m_pool.enqueue([&](void) -> void { this->update_thread(params, grid, r.first, r.second); }));
    }

    for (auto&& r : results) {
        r.get();
    }

    const auto toggles = params.toggles;
    const auto values = params.values;

    for (size_t id = 0; id < m_count; id++) {
        V2 dv = V2::null();

        // apply only the rules that have been turned on
        if (toggles[RT_AVERAGE_VELOCITY]) dv += m_delta_avg_vel[id];
        if (toggles[RT_CONFINE]) dv += m_delta_confine[id];
        if (toggles[RT_DENSITY]) dv += m_delta_density[id];
        if (toggles[RT_CENTER_OF_MASS]) dv += m_delta_center_of_mass[id];
        if (toggles[RT_GRAVITY]) dv.y += values[RT_GRAVITY] * dt;

        {
            float max_force = 100.f;

            if (toggles[RT_MAX_FORCE] && values[RT_MAX_FORCE] >= 0.f && values[RT_MAX_FORCE] < 300.f) {
                max_force = values[RT_MAX_FORCE];
            }

            const float force_magnitude = dv.magnitude();

            if (force_magnitude > max_force) {
                dv *= max_force / force_magnitude;
            }
        }

        V2& vel = m_vel[id];
        vel += dv;

        if (toggles[RT_MAX_VELOCITY] && values[RT_MAX_VELOCITY] >= 0.f &&
            values[RT_MAX_VELOCITY] < 500.f) {
        }
        vel = clamp(vel, values[RT_MAX_VELOCITY]);

        V2& pos = m_pos[id];
        pos = pos + dt * vel;

        if (!WinProps::is_boid_onscreen(pos)) {
            // @TODO: use random position?
            pos = {10.f, 10.f};
            vel = {10.f, 10.f};
        }
    }
}
