#include "v2.hpp"

V2& V2::operator+=(const V2& rhs)
{
    x += rhs.x;
    y += rhs.y;
    return *this;
}

V2& V2::operator-=(const V2& rhs)
{
    x -= rhs.x;
    y -= rhs.y;
    return *this;
}

V2& V2::operator/=(float sf)
{
    x /= sf;
    y /= sf;
    return *this;
}

V2& V2::operator*=(float sf)
{
    x *= sf;
    y *= sf;
    return *this;
}

float V2::magnitude(void) const { return std::sqrt(x * x + y * y); }

V2 operator*(const V2& v, float sf) { return {v.x * sf, v.y * sf}; }

V2 operator*(float sf, const V2& v) { return {v.x * sf, v.y * sf}; }

V2 operator-(const V2& lhs, const V2& rhs) { return {lhs.x - rhs.x, lhs.y - rhs.y}; }

V2 operator+(const V2& lhs, const V2& rhs) { return {lhs.x + rhs.x, lhs.y + rhs.y}; }

V2 operator/(const V2& v, float sf) { return {v.x / sf, v.y / sf}; }

V2 clamp(V2 vec, float max_magnitude)
{
    const auto current_magnitude = vec.magnitude();
    if (current_magnitude > max_magnitude) {
        return vec * (max_magnitude / current_magnitude);
    }
    else {
        return vec;
    }
}

float distance_sq(const V2& a, const V2& b)
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    return dx * dx + dy * dy;
}

std::ostream& operator<<(std::ostream& os, const V2& v)
{
    os << "(" << v.x << " , " << v.y << ")";
    return os;
}

V2 average_of(const std::vector<V2>& vecs)
{
    if (vecs.size() == 0) return V2::null();

    V2 accum = V2::null();

    for (const V2& v : vecs) {
        accum += v;
    }

    return accum / static_cast<float>(vecs.size());
}

