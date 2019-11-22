#pragma once

#include <assert.h>

#include <cmath>
#include <iostream>
#include <vector>

struct V2 {
    float x = 0.f;
    float y = 0.f;

    V2& operator+=(const V2& rhs);
    V2& operator-=(const V2& rhs);
    V2& operator/=(float sf);
    V2& operator*=(float sf);

    float magnitude(void) const;
    static constexpr V2 null(void) { return {0.f, 0.f}; }
};

V2 operator*(const V2& v, float sf);
V2 operator*(float sf, const V2& v);
V2 operator-(const V2& lhs, const V2& rhs);
V2 operator+(const V2& lhs, const V2& rhs);
V2 operator/(const V2& v, float sf);

V2 clamp(V2 vec, float max_magnitude);
float distance_sq(const V2& a, const V2& b);
std::ostream& operator<<(std::ostream& os, const V2& v);
V2 average_of(const std::vector<V2>& vecs);
