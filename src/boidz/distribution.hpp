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
    std::uniform_real_distribution<float> m_distX;
    std::uniform_real_distribution<float> m_distY;

public:
    UniformDistribution(void) = delete;

    UniformDistribution(float x_low, float x_high, float y_low, float y_high)
        : Distribution(), m_distX(x_low, x_high), m_distY(y_low, y_high)
    {
    }

    V2 sample(void) final { return {m_distX(m_engine), m_distY(m_engine)}; }
};
