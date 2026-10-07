#include "util/Vector.h"

float distance(const Vec2& a, const Vec2& b)
{
    return magnitude(a - b);
}

float magnitude(const Vec2& a)
{
    return std::sqrt(a.x*a.x + a.y*a.y);
}
