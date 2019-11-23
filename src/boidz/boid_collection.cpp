#include "boid_collection.hpp"

static constexpr float wrap_real(float x, float m) { return x - m * std::floor(x / m); }

BoidCollection::BoidCollection(size_t new_boid_count, Distribution& init_pos,
                               Distribution& init_vel)
{
    reset(new_boid_count, init_pos, init_vel);
}

BoidCollection::BoidCollection(void)
{
    UniformDistribution d_pos;
    UniformDistribution d_vel(0.5f, 100.f / 2.f);
    reset(1000, d_pos, d_vel);
}

void BoidCollection::reset(size_t new_boid_count, Distribution& init_pos,
                           Distribution& init_vel)
{
    for (std::vector<V2>* vec : {&m_pos, &m_vel, &m_delta_avg_vel, &m_delta_confine,
                                 &m_delta_density, &m_delta_center_of_mass}) {
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

void BoidCollection::update(float dt, const RuleParameters& params, QuadTree& grid)
{
    grid.insert(*this);

    static std::vector<PseudoBoid> neighbors;

    for (size_t id = 0; id < m_count; id++) {
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

            //@TODO: adaptive time step will make this issue more consistent
            const float separation = distance_sq(pos, pb.pos);
            if (separation > 1e-7) {
                dens_accum += pb.weight / separation * (pos - pb.pos);
            }
        }

        if (weight_sum > 0.f) {
            const float inverted_weight_sum = 1.f / weight_sum;
            const V2 avg_vel = pos_sum * inverted_weight_sum;
            const V2 avg_pos = vel_sum * inverted_weight_sum;

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
            const float dx1 = std::max((float)1e-6, std::abs(pos.x));
            const float dx2 = std::max((float)1e-6, std::abs(pos.x - 256.f));
            const float dy1 = std::max((float)1e-6, std::abs(pos.y));
            const float dy2 = std::max((float)1e-6, std::abs(pos.y - 256.f));

            const float cp = params.value.confine;

            const float confine_x =
                1.f / (cp * std::pow(dx1, 2.f)) - 1.f / (cp * std::pow(dx2, 2.f));
            const float confine_y =
                1.f / (cp * std::pow(dy1, 2.f)) - 1.f / (cp * std::pow(dy2, 2.f));

            m_delta_confine[id] = {confine_x, confine_y};
        }
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
        vel = clamp(vel, params.max_vel);

        V2& pos = m_pos[id];

        // safety wrapping, shouldn't occur often, ideally ever
        // @TODO: log this when it happens.
        pos.x = wrap_real(pos.x + dt * vel.x, 256.f);
        pos.y = wrap_real(pos.y + dt * vel.y, 256.f);
    }
}
