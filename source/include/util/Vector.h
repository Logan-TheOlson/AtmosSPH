#ifndef STJARNA_VECTOR_H
#define STJARNA_VECTOR_H

#include <cmath>

struct Vec2 {
    float x, y;

    Vec2() = default;
    constexpr Vec2(float x, float y) : x(x), y(y) {}

    Vec2& operator+=(const Vec2& rhs) { x += rhs.x; y += rhs.y; return *this; }
    Vec2& operator-=(const Vec2& rhs) { x -= rhs.x; y -= rhs.y; return *this; }
    Vec2& operator*=(float s)         { x *= s;     y *= s;     return *this; }
};

inline bool operator==(const Vec2& a, const Vec2& b) { return a.x == b.x && a.y == b.y; }
inline Vec2 operator+(const Vec2& a, const Vec2& b) { return Vec2(a.x + b.x, a.y + b.y); }
inline Vec2 operator-(const Vec2& a, const Vec2& b) { return Vec2(a.x - b.x, a.y - b.y); }
inline Vec2 operator*(const Vec2& a, float s)        { return Vec2(a.x * s, a.y * s); }
inline Vec2 operator*(float s, const Vec2& a)        { return a * s; }
inline Vec2 operator/(const Vec2& a, float s)        { return Vec2(a.x / s, a.y / s); }

float distance(const Vec2& a, const Vec2& b);

float magnitude(const Vec2& a);

inline float dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }

#endif //STJARNA_VECTOR_H
