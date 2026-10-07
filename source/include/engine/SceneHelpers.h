#pragma once
// Geometry, spawn-layout and color-blending helpers for the scene data structs in Scene.h.
#include "engine/Scene.h"

#include <algorithm>
#include <cmath>
#include <vector>

// Field acceleration at (px, py) for a Field-mode event; false if outside the area. Zero at the center.
inline bool EventFieldAccel(const SceneEvent& e, float px, float py, float& ax, float& ay) {
    const float dx = px - e.cx, dy = py - e.cy;
    float edge; // 0 at the center -> 1 at the boundary
    if (e.shape == EventShape::Circle) {
        const float d2 = dx * dx + dy * dy;
        if (d2 > e.radius * e.radius) return false;
        edge = e.radius > 0.f ? std::sqrt(d2) / e.radius : 0.f;
    } else {
        if (std::fabs(dx) > e.halfW || std::fabs(dy) > e.halfH) return false;
        edge = std::max(e.halfW > 0.f ? std::fabs(dx) / e.halfW : 0.f, e.halfH > 0.f ? std::fabs(dy) / e.halfH : 0.f);
    }
    ax = ay = 0.f;
    const float dist = std::sqrt(dx * dx + dy * dy);
    if (dist < 1e-4f) return true;
    const float mag = e.strength * (e.fade ? 1.f - edge : 1.f);
    const float ux = dx / dist, uy = dy / dist;
    switch (e.fieldKind) {
        case FieldKind::Curl:    ax = -uy * mag; ay =  ux * mag; break;
        case FieldKind::Attract: ax = -ux * mag; ay = -uy * mag; break;
        case FieldKind::Repel:   ax =  ux * mag; ay =  uy * mag; break;
    }
    return true;
}

// Signed distance from (x, y) to the obstacle's surface (negative inside) and the outward unit
// normal. Callers treat `sd < margin` as touching.
inline float ObstacleSurface(const SceneObstacle& o, float x, float y, float& nx, float& ny) {
    const float dx = x - o.cx, dy = y - o.cy;
    if (o.shape == EventShape::Circle) {
        const float d = std::sqrt(dx * dx + dy * dy);
        if (d > 1e-6f) { nx = dx / d; ny = dy / d; }
        else           { nx = 0.f;    ny = 1.f; }
        return d - o.radius;
    }
    const float qx = std::fabs(dx) - o.halfW, qy = std::fabs(dy) - o.halfH;
    const float sx = dx < 0.f ? -1.f : 1.f, sy = dy < 0.f ? -1.f : 1.f;
    if (qx > 0.f || qy > 0.f) {
        const float ox = std::max(qx, 0.f), oy = std::max(qy, 0.f);
        const float d  = std::sqrt(ox * ox + oy * oy);
        nx = sx * ox / d; ny = sy * oy / d;
        return d;
    }
    if (qx > qy) { nx = sx;  ny = 0.f; return qx; }
    nx = 0.f; ny = sy; return qy;
}

// True if a point (grown by `margin`, e.g. the particle radius) lies inside any obstacle.
inline bool InsideAnyObstacle(const std::vector<SceneObstacle>& obstacles, float x, float y, float margin = 0.f) {
    float nx, ny;
    for (const SceneObstacle& o : obstacles)
        if (ObstacleSurface(o, x, y, nx, ny) < margin) return true;
    return false;
}

inline constexpr int MaxSpawnParticles = 1'000'000; // per group; keeps typos from allocating gigabytes

inline float GroupSpacing(const SceneGroup& g, float circleRadius) {
    return circleRadius * 3.f * std::max(g.spacingScale, 0.1f);
}

// Lattice dimensions of a rect group (0 x 0 if the rect is empty or the spacing degenerate).
inline void GroupGrid(const SceneGroup& g, float spacing, int& nx, int& ny) {
    nx = ny = 0;
    if (!(spacing > 0.f) || !(g.halfW > 0.f) || !(g.halfH > 0.f)) return;
    // +1e-3 so a block sized as exactly N spacings isn't rounded down to N-1 by float error.
    const float fx = 2.f * g.halfW / spacing + 1e-3f, fy = 2.f * g.halfH / spacing + 1e-3f;
    if (!std::isfinite(fx) || !std::isfinite(fy)) return;
    nx = std::clamp(static_cast<int>(std::min(fx, static_cast<float>(MaxSpawnParticles))), 1, MaxSpawnParticles);
    ny = std::clamp(static_cast<int>(std::min(fy, static_cast<float>(MaxSpawnParticles))), 1, MaxSpawnParticles / nx);
}

inline int GroupCircleCount(const SceneGroup& g, float spacing) {
    if (!(spacing > 0.f) || !(g.radius > 0.f)) return 0;
    const float c = 3.14159265f * g.radius * g.radius / (spacing * spacing) + 0.5f;
    return std::isfinite(c) ? static_cast<int>(std::min(c, static_cast<float>(MaxSpawnParticles))) : 0;
}

inline int GroupParticleCount(const SceneGroup& g, float spacing) {
    if (g.shape == EventShape::Circle) return GroupCircleCount(g, spacing);
    int nx, ny;
    GroupGrid(g, spacing, nx, ny);
    return nx * ny;
}

// Rounds a rect group's box to a whole number of lattice cells (up to half a spacing per side), so
// the particles fill it edge to edge. Circles already fill their disk; they are left alone.
inline void SnapGroupToLattice(SceneGroup& g, float spacing) {
    if (g.shape == EventShape::Circle || !(spacing > 0.f) || !(g.halfW > 0.f) || !(g.halfH > 0.f)) return;
    const float fx = 2.f * g.halfW / spacing, fy = 2.f * g.halfH / spacing;
    if (!std::isfinite(fx) || !std::isfinite(fy)) return;
    const int nx = std::clamp(static_cast<int>(std::lround(std::min(fx, static_cast<float>(MaxSpawnParticles)))), 1, MaxSpawnParticles);
    const int ny = std::clamp(static_cast<int>(std::lround(std::min(fy, static_cast<float>(MaxSpawnParticles)))), 1, MaxSpawnParticles / nx);
    g.halfW = 0.5f * static_cast<float>(nx) * spacing;
    g.halfH = 0.5f * static_cast<float>(ny) * spacing;
}

// For a rect group with fixSize: sizes the box to exactly cols x rows cells at `spacing`.
inline void FitGroupToGrid(SceneGroup& g, float spacing) {
    if (!g.fixSize || g.shape == EventShape::Circle || !(spacing > 0.f)) return;
    g.cols = std::clamp(g.cols, 1, MaxSpawnParticles);
    g.rows = std::clamp(g.rows, 1, MaxSpawnParticles / g.cols);
    g.halfW = 0.5f * static_cast<float>(g.cols) * spacing;
    g.halfH = 0.5f * static_cast<float>(g.rows) * spacing;
}

// Resizes the group (keeping center, shape and a rect's aspect ratio) to hold `count` particles at
// `spacing`; a rect ends up a whole number of cells on each side.
inline void ResizeGroupToCount(SceneGroup& g, float spacing, int count) {
    if (!(spacing > 0.f)) return;
    count = std::clamp(count, 1, MaxSpawnParticles);
    if (g.shape == EventShape::Circle) {
        g.radius = spacing * std::sqrt(static_cast<float>(count) / 3.14159265f);
    } else {
        const float aspect = (g.halfW > 0.f && g.halfH > 0.f) ? g.halfW / g.halfH : 1.f;
        const int ny = std::max(1, static_cast<int>(std::lround(std::sqrt(static_cast<float>(count) / aspect))));
        const int nx = std::max(1, static_cast<int>(std::lround(static_cast<float>(count) / static_cast<float>(ny))));
        g.halfW = 0.5f * static_cast<float>(nx) * spacing;
        g.halfH = 0.5f * static_cast<float>(ny) * spacing;
    }
}

// Calls fn(x, y) for every particle position of the group at `spacing`.
template <typename Fn>
inline void ForEachGroupPoint(const SceneGroup& g, float spacing, Fn&& fn) {
    if (g.shape == EventShape::Circle) {
        // Fibonacci (Vogel) spiral: exactly `count` particles at areal density 1 / spacing^2.
        const int   count = GroupCircleCount(g, spacing);
        constexpr float GoldenAngle = 2.39996323f; // pi * (3 - sqrt(5))
        for (int i = 0; i < count; i++) {
            const float r     = g.radius * std::sqrt((static_cast<float>(i) + 0.5f) / static_cast<float>(count));
            const float theta = static_cast<float>(i) * GoldenAngle;
            fn(g.cx + r * std::cos(theta), g.cy + r * std::sin(theta));
        }
        return;
    }
    int nx, ny;
    GroupGrid(g, spacing, nx, ny);
    const float startX = g.cx - static_cast<float>(nx - 1) * spacing * 0.5f;
    const float startY = g.cy - static_cast<float>(ny - 1) * spacing * 0.5f;
    for (int x = 0; x < nx; x++)
        for (int y = 0; y < ny; y++)
            fn(startX + static_cast<float>(x) * spacing, startY + static_cast<float>(y) * spacing);
}

// Blends SceneColorRegions into `out` (which must hold the color the point would otherwise have).
//  * Color: a gradient between the areas' centers, each weighted by reach / distance-to-center.
//    Where areas overlap, only the topmost (last) takes part.
//  * Coverage: that color is solid inside the areas' footprint and fades linearly back to `out`
//    over a band beyond the edge.
// `strength` (0..1) scales how far the gradient replaces an area's flat color and how much of the
// edge fade applies (0 = hard-edged patches). `radius` (px) is the band width; <= 0 = 50% of the
// area's smaller half-size. Returns false (out untouched) for points no area reaches.
inline bool BlendColorRegions(const std::vector<SceneColorRegion>& regions, float x, float y, Color& out,
                              float strength = 1.f, float radius = 0.f) {
    auto edgeDist = [&](const SceneColorRegion& r, float& size) {
        const float dx = x - r.cx, dy = y - r.cy;
        if (r.shape == EventShape::Circle) {
            size = r.radius;
            return std::max(std::sqrt(dx * dx + dy * dy) - r.radius, 0.f);
        }
        size = std::min(r.halfW, r.halfH);
        const float ex = std::max(std::fabs(dx) - r.halfW, 0.f), ey = std::max(std::fabs(dy) - r.halfH, 0.f);
        return std::sqrt(ex * ex + ey * ey);
    };
    strength = std::clamp(strength, 0.f, 1.f);
    int top = -1;  // topmost area containing the point
    for (int i = 0; i < static_cast<int>(regions.size()); i++) {
        float size;
        if (edgeDist(regions[static_cast<size_t>(i)], size) <= 0.f) top = i;
    }
    float sr = 0.f, sg = 0.f, sb = 0.f, sa = 0.f, sw = 0.f, cover = 0.f;
    for (int i = 0; i < static_cast<int>(regions.size()); i++) {
        const SceneColorRegion& r = regions[static_cast<size_t>(i)];
        float size;
        const float d = edgeDist(r, size);
        if (d <= 0.f && i != top) continue;  // covered by a region above it
        // Each area's reach is its own (independent of the other areas): solid inside, linear fade over `band`.
        const float band = radius > 0.f ? radius : std::max(size * 0.5f, 1.f);
        const float c = d <= 0.f ? 1.f : std::clamp(1.f - d / band, 0.f, 1.f) * strength;
        if (c <= 0.f) continue;
        // Colors mix by reach / distance-to-center, so an area only tints points it actually reaches and,
        // where reaches overlap, the colors gradient between the centers.
        const float w = c / std::max(std::hypot(x - r.cx, y - r.cy), 1.f);
        sr += r.color.r * w; sg += r.color.g * w; sb += r.color.b * w; sa += r.color.a * w; sw += w;
        cover = std::max(cover, c);
    }
    if (sw <= 0.f || cover <= 0.f) return false;
    const float inv = 1.f / sw;
    Color grad = { sr * inv, sg * inv, sb * inv, sa * inv };
    if (top >= 0) {  // inside: strength picks between the area's flat color and the gradient
        const Color& flat = regions[static_cast<size_t>(top)].color;
        grad = { flat.r + (grad.r - flat.r) * strength, flat.g + (grad.g - flat.g) * strength,
                 flat.b + (grad.b - flat.b) * strength, flat.a + (grad.a - flat.a) * strength };
    }
    out = { out.r + (grad.r - out.r) * cover, out.g + (grad.g - out.g) * cover,
            out.b + (grad.b - out.b) * cover, out.a + (grad.a - out.a) * cover };
    return true;
}
