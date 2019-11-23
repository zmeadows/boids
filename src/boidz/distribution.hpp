#pragma once

#include <random>

#include "props.hpp"
#include "v2.hpp"

class Distribution {
protected:
    std::random_device m_device;
    std::mt19937 m_engine;

    Distribution(void) : m_device(), m_engine(m_device()) {}

public:
    virtual V2 sample(void) = 0;
};

class UniformDistribution : public Distribution {
    std::uniform_real_distribution<float> m_dist;

    static constexpr float epsilon = 1e-3F;

public:
    UniformDistribution(float low, float high) : Distribution(), m_dist(low, high) {}

    UniformDistribution(void)
        : Distribution(), m_dist(epsilon, WindowProps::coordinate_span - epsilon)
    {
    }

    V2 sample(void) final { return {m_dist(m_engine), m_dist(m_engine)}; }
};
