#pragma once
#include "Config.h"
#include "util/Vector.h"
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

// Simulation container half-extents, as a fraction of the window's half-extents.
struct SceneBoundary {
    float widthFrac  = 1.0f;
    float heightFrac = 1.0f;
    // Absolute half-extents in simulation pixels. When > 0, each overrides its *Frac above, making
    // the container independent of window size and recording resolution.
    float halfWidthPx  = 0.f;
    float halfHeightPx = 0.f;
    // If set, a particle crossing the left/right wall teleports to the other side (velocity
    // untouched) instead of bouncing. main.cpp also mirrors particles across the seam (see
    // ForEachPeriodicX) so density/pressure near it match an infinite domain.
    bool  periodicX  = false;
};

struct ScenePhysics {
    // Constant acceleration applied to every particle (px/s^2).
    Vec2  force          = Vec2(0.f, -600.f);
    float restitution   = 0.85f;
    // Fraction of tangential velocity removed on wall contact when noSlipWalls is enabled.
    float friction       = 0.3f;
    bool  noSlipWalls    = true;
    int   substeps       = 12;
    int   forceInterval  = 1;
};

// BarnesHut: the 3D Newtonian (1/r^2) law restricted to the simulation plane (see Quadtree.h).
// Genuine2D: the softened log-potential (1/r force) law of a true 2D universe (see Gravity.cpp's
// EvaluateForce2D); the default.
enum class GravityMethod { BarnesHut, Genuine2D };

// Self-gravity between particles, layered on top of ScenePhysics::force. Off by default.
struct SceneGravity {
    bool          enabled          = false;
    GravityMethod method           = GravityMethod::Genuine2D;
    float         g                = 20.f;
    float         particleMass     = 1.f;
    float         softening        = 7.9f; // Plummer softening length, avoids the 1/r^2 singularity as r -> 0
    float         macTheta         = 0.5f; // Barnes-Hut node-acceptance criterion
    int           maxLeafParticles = 8;
    bool          validate         = false; // periodically compares the active method against brute-force on a small sample
};

// Which pressure-from-density law main.cpp's CalculatePressure applies.
enum class EosModel {
    // Becker-Teschner weakly-compressible SPH: pressure = stiffness*targetDensity/exponent *
    // ((density/targetDensity)^exponent - 1), clamped to >=0.
    WCSPH,
    // Pure polytrope: pressure = stiffness * density^(1 + 1/polytropicIndex), no targetDensity offset.
    Polytropic,
};

// Which viscous force main.cpp's CalculatePressureForce applies between particle pairs.
enum class ViscosityModel {
    // Monaghan artificial viscosity: a pressure-like term along the pair axis for approaching
    // pairs, set by `viscosity` (alpha) and `viscosityQuadratic` (beta). The effective kinematic
    // viscosity (~alpha*h*c/8 in 2D) is not an input and scales with h.
    Monaghan,
    // Morris, Fox & Zhu (1997) physical viscosity: nu * laplacian(v) with nu = `kinematicViscosity`.
    Morris,
};

struct SceneParticles {
    float circleRadius = 3.0f;
    Color circleColor  = { 0.2f, 0.6f, 1.0f, 1.0f };

    // Calibrated to smoothingRadius=15 and a grid spacing of circleRadius*3=9px; recompute
    // targetDensity (lattice-sum the Poly6 kernel) if either changes.
    float smoothingRadius    = 15.0f;
    // 6.48e-4 is the lattice sum under the old 3D Poly6 normalization; the factor converts it to
    // the 2D normalization (4/(pi*h^8)) at h=15.
    float targetDensity      = 6.48e-4f * (256.f * 15.f / 315.f); // = 7.8994...e-3
    float stiffness          = 12000.f;

    EosModel eos             = EosModel::WCSPH;
    int      exponent        = 7;         // WCSPH only
    float    polytropicIndex = 1.f / 6.f; // Polytropic only; gamma = 1 + 1/polytropicIndex

    ViscosityModel viscosityModel = ViscosityModel::Monaghan;

    float viscosity          = 0.5f;  // Monaghan only (alpha)
    // Quadratic-term coefficient (beta); must stay well under 1 (see CalculatePressureForce). Monaghan only.
    float viscosityQuadratic = 0.05f;
    // Kinematic viscosity nu in px^2/s. Morris only.
    float kinematicViscosity = 800.f;
};

// How Init() lays out the starting particles for a scene without groups: a gridCountX x gridCountY
// block (or a circle inscribed in it), centered vertically and offset horizontally by
// offsetXFrac * the boundary's half-width.
struct SceneSpawn {
    int   gridCountX  = 100;
    int   gridCountY  = 100;
    float offsetXFrac = 0.0f;
    bool  circular     = false;
    // If set, Init() packs the lattice wall to wall and writes the resulting counts back into gridCountX/Y.
    bool  fillBoundary = false;
    // Lattice spacing as a multiple of the default (3 * circleRadius). < 1 starts the fluid
    // compressed; the particle count rises by 1 / spacingScale^2.
    float spacingScale = 1.f;
};

// An acceleration applied to every particle inside an area. Continuous applies `accel` (px/s^2)
// every substep while active; Once adds `accel` (px/s) to the velocity once, at startTime.
enum class EventShape { Circle, Rect };
// Field: a position-dependent acceleration (see EventFieldAccel) instead of a uniform push.
enum class EventMode  { Continuous, Once, Field };
// Curl swirls counter-clockwise around the area's center (negative strength = clockwise).
enum class FieldKind  { Curl, Attract, Repel };

struct SceneEvent {
    EventShape shape = EventShape::Circle;
    EventMode  mode  = EventMode::Once;
    // Area center in world pixels (origin = window center, +y up).
    float cx = 0.f, cy = 0.f;
    float radius = 100.f;                // Circle
    float halfW = 100.f, halfH = 100.f;  // Rect
    Vec2  accel = Vec2(0.f, 0.f);        // Continuous: px/s^2.  Once: px/s velocity delta.
    float startTime = 0.f;               // simulated seconds since the run started
    float duration  = 0.f;               // Continuous/Field only; 0 = until the run ends
    FieldKind fieldKind = FieldKind::Curl; // Field only
    float strength  = 400.f;             // Field only: acceleration magnitude, px/s^2
    bool  fade      = false;             // Field only: full strength at the center, falling to 0 at the edge
};

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

// A persistent-color patch: particles spawned inside it keep `color` for the run unless a live
// color-by mode overrides it. World pixels, as SceneEvent. Later regions win overlaps.
struct SceneColorRegion {
    EventShape shape = EventShape::Rect;
    float cx = 0.f, cy = 0.f;
    float radius = 100.f;                // Circle
    float halfW = 100.f, halfH = 100.f;  // Rect
    Color color = { 1.f, 0.3f, 0.2f, 1.f };
};

// A solid obstacle: nothing spawns inside it, particles bounce off it, and it mirrors particles
// for SPH density (main.cpp's ForEachObstacleGhost). World pixels, as SceneEvent.
struct SceneObstacle {
    EventShape shape = EventShape::Circle;
    float cx = 0.f, cy = 0.f;
    float radius = 100.f;                // Circle
    float halfW = 100.f, halfH = 100.f;  // Rect
    Color color = { 0.55f, 0.58f, 0.65f, 1.f };
};

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

// A block of starting particles: a square lattice filling a rect, or a Fibonacci-spiral disk for a
// circle, at spacing 3 * circleRadius * spacingScale. A Scene with `useGroups` spawns exactly its
// groups; otherwise Init() derives one from SceneSpawn (SpawnBlockAsGroup). World pixels.
struct SceneGroup {
    EventShape shape = EventShape::Rect;
    float cx = 0.f, cy = 0.f;
    float radius = 100.f;                // Circle
    float halfW = 100.f, halfH = 100.f;  // Rect
    float spacingScale = 1.f;            // < 1 packs tighter, like SceneSpawn::spacingScale
    bool  customColor = false;           // false: the scene's particle color (particles.circleColor)
    Color color = { 0.2f, 0.6f, 1.0f, 1.0f };
};

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

// Resizes the group (keeping center, shape and a rect's aspect ratio) to hold about `count`
// particles at `spacing`.
inline void ResizeGroupToCount(SceneGroup& g, float spacing, int count) {
    if (!(spacing > 0.f)) return;
    count = std::clamp(count, 1, MaxSpawnParticles);
    const float area = static_cast<float>(count) * spacing * spacing; // one particle per spacing^2
    if (g.shape == EventShape::Circle) {
        g.radius = std::sqrt(area / 3.14159265f);
    } else {
        const float aspect = (g.halfW > 0.f && g.halfH > 0.f) ? g.halfW / g.halfH : 1.f;
        g.halfH = 0.5f * std::sqrt(area / aspect);
        g.halfW = g.halfH * aspect;
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

// A persistent-color pattern laid over the spawn block; SceneColorRegions win where they overlap.
enum class ColorPatternKind { None, VStripes, HStripes, Checker, Rings, Gradient, Image };

struct SceneColorPattern {
    ColorPatternKind kind = ColorPatternKind::None;
    Color colorA = { 1.f, 0.35f, 0.25f, 1.f };
    Color colorB = { 0.25f, 0.55f, 1.f, 1.f };
    float cellSize = 60.f;       // px: stripe width / checker cell / ring spacing (unused by Gradient/Image)
    std::string imagePath;       // Image: PNG/BMP scaled to fit the spawn block; transparent pixels are left uncolored
};

struct Scene {
    std::string     name;
    std::string     description;
    SceneBoundary   boundary;
    ScenePhysics    physics;
    SceneGravity    gravity;
    SceneParticles  particles;
    SceneSpawn      spawn;

    // Hand-tuned grid size this scene runs smoothly at in real time; shown for reference in the
    // menu only. Independent of spawn.gridCountX/Y.
    int             realTimeLimitX = 0;
    int             realTimeLimitY = 0;

    std::vector<SceneEvent> events;
    std::vector<SceneColorRegion> colorRegions;
    std::vector<SceneObstacle>    obstacles;
    bool                          useGroups = false; // true: spawn exactly `groups` (see SceneGroup)
    std::vector<SceneGroup>       groups;
    bool  blendColorRegions = false; // gradient between areas / pattern cells instead of hard patches (BlendColorRegions)
    float blendStrength     = 1.f;   // 0..1: how much of the gradient is applied
    float blendRadius       = 0.f;   // px: distance the gradient reaches; 0 = automatic
    SceneColorPattern colorPattern;
};

extern const std::vector<Scene> ScenePresets;

// Rebuilds ScenePresets[presetIndex] with its grid resized to gridCountX x gridCountY (clamped to
// >= 1), re-deriving any count-dependent calibration.
Scene BuildScene(int presetIndex, int gridCountX, int gridCountY);

// The SceneSpawn block expressed as a SceneGroup inside a container of the given half-extents.
// Clamps and writes the resulting grid counts back into scene.spawn.
SceneGroup SpawnBlockAsGroup(Scene& scene, float halfWidth, float halfHeight);

// Total particles the scene will spawn (before obstacle culling), from its groups if it has them.
int SceneParticleCount(const Scene& scene);

// The scene currently governing physics/spawn/boundary. Set via LoadScene(), not assigned
// directly, so the derived SPH constants stay in sync.
extern Scene ActiveScene;

// Makes `scene` active and recomputes cached kernel constants. Does not spawn; call Init() after.
void LoadScene(const Scene& scene);
