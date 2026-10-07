#include <algorithm>
#include "util/ColorPattern.h"
#include <cmath>
#include <cstdint>
#include <vector>

#include "engine/Engine.h"
#include "engine/Scene.h"
#include "Config.h"
#include "physics/Gravity.h"
#include "util/ThreadPool.h"

// ----------- Spatial partitioning
// Uniform grid (cell = 2*SmoothingRadius) stored as a CSR-sorted flat hash: no per-cell
// allocations, and cellStart[h]..cellStart[h+1] is a contiguous slice.
//
// The same sort reorders a "hot" copy of pos/vel/density/pressure/force by cell, separate from
// Object/Renderable, so the density/force loops read mostly-contiguous memory.
//
// Cells can collide in the hash; cellX/cellY reject false positives.
namespace {
    // Derived from ActiveScene.particles by RecomputeSphConstants().
    float Radius   = 0.f;
    float RadiusSq = 0.f;
    float CellSize = 0.f;

    std::vector<int32_t>  rawCellX, rawCellY;   // per real particle, scratch for BuildGrid
    std::vector<uint32_t> rawHash;

    std::vector<Vec2>    hotPos, hotVel, hotForce, hotGravAcc;
    std::vector<float>   hotDensity, hotPressure;
    std::vector<int32_t> hotToReal;    // hot slot -> index into `objects`
    std::vector<int32_t> cellX, cellY; // per hot slot, integer cell coords
    std::vector<int32_t> cellStart;    // [tableSize+1] CSR offsets into the hot arrays
    std::vector<int32_t> cursor;       // scratch write cursor per bucket during the scatter pass
    uint32_t tableSize = 0;
}

// Clamped and NaN-safe (casting NaN/out-of-range floats to int32_t is UB); a non-finite particle lands in cell 0.
static int32_t CellCoord(float p) {
    constexpr float Limit = 1.0e9f;
    const float c = std::floor(p / CellSize);
    if (!std::isfinite(c)) return 0;
    return static_cast<int32_t>(std::clamp(c, -Limit, Limit));
}

// Teschner et al.'s spatial-hash primes; unsigned to avoid signed overflow.
static uint32_t HashCell(int32_t x, int32_t y) {
    return (static_cast<uint32_t>(x) * 73856093u) ^ (static_cast<uint32_t>(y) * 19349663u);
}

static uint32_t NextPow2(uint32_t v) {
    v--; v |= v >> 1; v |= v >> 2; v |= v >> 4; v |= v >> 8; v |= v >> 16;
    return v + 1;
}

// Rebuilt once per Update(). Arrays are reused across calls, so a stable particle count allocates nothing.
static void BuildGrid() {
    const int32_t n = static_cast<int32_t>(objects.size());
    tableSize = std::max<uint32_t>(16u, NextPow2(static_cast<uint32_t>(n) * 2u));

    rawCellX.resize(n);
    rawCellY.resize(n);
    rawHash.resize(n);
    cellStart.assign(tableSize + 1, 0);

    for (int32_t i = 0; i < n; i++) {
        const int32_t cx = CellCoord(objects[i].pos.x);
        const int32_t cy = CellCoord(objects[i].pos.y);
        rawCellX[i] = cx;
        rawCellY[i] = cy;
        const uint32_t h = HashCell(cx, cy) & (tableSize - 1);
        rawHash[i] = h;
        cellStart[h + 1]++;
    }
    for (uint32_t h = 0; h < tableSize; h++) cellStart[h + 1] += cellStart[h]; // counts -> start offsets

    hotToReal.resize(n);
    cellX.resize(n);
    cellY.resize(n);
    hotPos.resize(n);
    hotVel.resize(n);
    hotDensity.resize(n);
    hotPressure.resize(n);
    hotForce.resize(n);
    hotGravAcc.resize(n);

    cursor.assign(cellStart.begin(), cellStart.end() - 1);
    for (int32_t i = 0; i < n; i++) {
        const int32_t k = cursor[rawHash[i]]++;
        hotToReal[k] = i;
        cellX[k] = rawCellX[i];
        cellY[k] = rawCellY[i];
        hotPos[k] = objects[i].pos;
        hotVel[k] = objects[i].vel;
    }
}

// Invokes fn(int32_t hotSlot) for every hot slot in pos's cell or its 8 neighbors. `pos` need not
// belong to a real particle (see ForEachGhost); callers exclude themselves by hot-slot index.
template <typename Fn>
static void ForEachNeighbor(const Vec2& pos, Fn&& fn) {
    const int32_t cx = CellCoord(pos.x);
    const int32_t cy = CellCoord(pos.y);
    for (int32_t dx = -1; dx <= 1; dx++) {
        for (int32_t dy = -1; dy <= 1; dy++) {
            const int32_t nx = cx + dx, ny = cy + dy;
            const uint32_t h = HashCell(nx, ny) & (tableSize - 1);
            for (int32_t k = cellStart[h]; k < cellStart[h + 1]; k++) {
                if (cellX[k] != nx || cellY[k] != ny) continue; // hash-collision false positive
                fn(k);
            }
        }
    }
}

// Mirror-image ("method of images") boundary particles. SPH density near a wall is under-counted,
// which zero-clamps pressure and lets particles pile up. Reflecting nearby real particles across
// the wall and summing them like normal neighbors restores the missing mass.
//
// Position mirrors exactly; velocity mirrors with the wall-normal component negated (free-slip);
// density/pressure carry over unchanged. The ghost carries the hot-slot index rather than
// density/pressure, which other threads are still writing during the density phase.
struct Ghost { Vec2 pos; Vec2 vel; int32_t src; };

struct WallMirror { bool active = false; float wall = 0.f; };

// Which wall (if either) `coord` is within Radius of, on one axis. `half` is the true container
// edge, not the particle-radius-inset range centers clamp to.
static WallMirror NearestWall(float coord, float half) {
    if (coord + half < Radius) return { true, -half };
    if (half - coord < Radius) return { true, half };
    return {};
}

// Invokes fn(const Ghost&) for every real particle whose mirror image (across whichever wall(s)
// `pos` is within Radius of, including the corner double-mirror) falls within Radius of `pos`.
template <typename Fn>
static void ForEachGhost(const Vec2& pos, Fn&& fn) {
    // No X wall to mirror across when the domain wraps; ForEachPeriodicX handles that axis.
    const WallMirror wx = ActiveScene.boundary.periodicX ? WallMirror{} : NearestWall(pos.x, ScreenHalfWidth());
    const WallMirror wy = NearestWall(pos.y, ScreenHalfHeight());

    const bool noSlip = ActiveScene.physics.noSlipWalls;

    auto pass = [&](bool mirrorX, bool mirrorY) {
        Vec2 query = pos;
        if (mirrorX) query.x = 2.f * wx.wall - pos.x;
        if (mirrorY) query.y = 2.f * wy.wall - pos.y;

        ForEachNeighbor(query, [&](int32_t j) {
            Vec2 gp = hotPos[j];
            if (mirrorX) gp.x = 2.f * wx.wall - hotPos[j].x;
            if (mirrorY) gp.y = 2.f * wy.wall - hotPos[j].y;

            // Free-slip: only the wall-normal velocity reverses. No-slip: the whole velocity
            // reverses, so the viscosity force drives velocity to zero at the wall; this produces the
            // wall drag a Poiseuille profile needs, which the friction in resolveAxis alone can't
            // once flow settles and stops colliding.
            Vec2 gv = hotVel[j];
            if (noSlip) {
                gv = gv * -1.f;
            } else {
                if (mirrorX) gv.x = -hotVel[j].x;
                if (mirrorY) gv.y = -hotVel[j].y;
            }
            fn(Ghost{ gp, gv, j });
        });
    };

    if (wx.active)               pass(true,  false);
    if (wy.active)               pass(false, true);
    if (wx.active && wy.active)  pass(true,  true);
}

// Same method of images as ForEachGhost, across each obstacle's surface: every obstacle within
// Radius of `pos` is mirrored across the tangent plane at its nearest surface point (exact for a
// rect face, good for circles much larger than the smoothing radius). Particles are never inside
// an obstacle, so sd < 0 is ignored.
template <typename Fn>
static void ForEachObstacleGhost(const Vec2& pos, Fn&& fn) {
    const auto& obstacles = ActiveScene.obstacles;
    if (obstacles.empty()) return;
    const bool noSlip = ActiveScene.physics.noSlipWalls;

    for (const SceneObstacle& o : obstacles) {
        float nx, ny;
        const float sd = ObstacleSurface(o, pos.x, pos.y, nx, ny);
        if (sd < 0.f || sd >= Radius) continue;

        const Vec2 q(pos.x - nx * sd, pos.y - ny * sd);                 // nearest surface point
        const Vec2 query(pos.x - 2.f * sd * nx, pos.y - 2.f * sd * ny); // pos mirrored across the plane
        ForEachNeighbor(query, [&](int32_t j) {
            const Vec2  p  = hotPos[j];
            const float dn = (p.x - q.x) * nx + (p.y - q.y) * ny;
            const Vec2  gp(p.x - 2.f * dn * nx, p.y - 2.f * dn * ny);

            // A true boundary image lives inside the solid. Around a convex corner or curved
            // surface the tangent-plane mirror would land in the fluid, creating phantom particles
            // that over-repel.
            float gnx, gny;
            if (ObstacleSurface(o, gp.x, gp.y, gnx, gny) >= 0.f) return;

            Vec2 gv = hotVel[j];
            if (noSlip) {
                gv = gv * -1.f;
            } else {
                const float vn = gv.x * nx + gv.y * ny; // free-slip: only the normal component flips
                gv = Vec2(gv.x - 2.f * vn * nx, gv.y - 2.f * vn * ny);
            }
            fn(Ghost{ gp, gv, j });
        });
    }
}

// Periodic-image particles for a wrapped X boundary (see SceneBoundary::periodicX): a particle
// near one edge sees the real particles past the opposite edge as neighbors. Unlike ForEachGhost,
// this is a translation by the domain width, and velocities carry through unchanged.
template <typename Fn>
static void ForEachPeriodicX(const Vec2& pos, Fn&& fn) {
    if (!ActiveScene.boundary.periodicX) return;
    const float hw          = ScreenHalfWidth();
    const float domainWidth = 2.f * hw;
    // A particle near a corner (both a Y wall and the seam) also needs the shifted-and-mirrored
    // diagonal image, which ForEachGhost never produces since periodicX turns its X-mirror off.
    const WallMirror wy = NearestWall(pos.y, ScreenHalfHeight());
    const bool noSlip   = ActiveScene.physics.noSlipWalls;

    auto pass = [&](float shift) {
        Vec2 query = pos;
        query.x -= shift;
        ForEachNeighbor(query, [&](int32_t j) {
            Vec2 gp = hotPos[j];
            gp.x += shift;
            fn(Ghost{ gp, hotVel[j], j });
        });

        if (wy.active) {
            Vec2 cornerQuery = query;
            cornerQuery.y = 2.f * wy.wall - pos.y;
            ForEachNeighbor(cornerQuery, [&](int32_t j) {
                Vec2 gp = hotPos[j];
                gp.x += shift;
                gp.y = 2.f * wy.wall - hotPos[j].y;
                // Same free-slip/no-slip rule as ForEachGhost; the X shift never touches velocity.
                Vec2 gv = hotVel[j];
                if (noSlip) gv = gv * -1.f;
                else        gv.y = -hotVel[j].y;
                fn(Ghost{ gp, gv, j });
            });
        }
    };

    if (pos.x + hw < Radius) pass(-domainWidth);  // near the left edge: pull in images from the right
    if (hw - pos.x < Radius) pass(domainWidth);   // near the right edge: pull in images from the left
}

// ----------- Kernels
namespace {
    constexpr float Pi = 3.14159265358979323846f;
    // Derived from Radius/RadiusSq (and Stiffness) by RecomputeSphConstants().
    float DensityNorm      = 0.f;
    float PressureGradNorm = 0.f;
    float SoundSpeed       = 0.f;
    float PolytropicGamma  = 0.f; // 1 + 1/polytropicIndex; only used when eos == Polytropic
}

// Recomputes every SPH constant derived from ActiveScene.particles. Called by LoadScene(), so the
// hot loops read plain floats.
void RecomputeSphConstants() {
    // Floored: a zero/negative radius would divide by zero in the cell size and kernel norms.
    Radius   = std::max(ActiveScene.particles.smoothingRadius, 0.1f);
    RadiusSq = Radius * Radius;
    CellSize = 2.f * Radius;

    // radius^5 / radius^8, expanded by hand to avoid std::pow.
    const float radius4 = RadiusSq * RadiusSq;
    const float radius5 = radius4 * Radius;
    const float radius8 = radius4 * radius4;
    // 2D Poly6/Spiky normalizations: integrating each kernel over a disk of radius h gives
    // 4/(pi*h^8) for Poly6 and -30/(pi*h^5) for Spiky's gradient.
    DensityNorm      = 4.f / (Pi * radius8);
    PressureGradNorm = -30.f / (Pi * radius5);
    PolytropicGamma  = 1.f + 1.f / ActiveScene.particles.polytropicIndex;

    // Reference sound speed sqrt(dP/drho) for the artificial-viscosity term. WCSPH's stiffness is
    // approximately c^2, but the Polytropic stiffness K (pressure = K*density^gamma) is on a
    // completely different scale, so evaluate dP/drho = K*gamma*rho^(gamma-1) at targetDensity.
    if (ActiveScene.particles.eos == EosModel::Polytropic) {
        const auto& p = ActiveScene.particles;
        SoundSpeed = std::sqrt(p.stiffness * PolytropicGamma * std::pow(p.targetDensity, PolytropicGamma - 1.f));
    } else {
        SoundSpeed = std::sqrt(ActiveScene.particles.stiffness);
    }
}

// Poly6-style kernel. Takes squared distance, which callers already have.
static float DensityKernel (float distSq)
{
    float val = RadiusSq - distSq;
    return DensityNorm * val * val * val;
}

// 'Spiky' Kernel Gradient: to prevent lack of pressure at dst=0 in DensityKernel.
// Caller guarantees 0 < dst < Radius.
static float PressureKernelGradient (float dst)
{
    float val = Radius - dst;
    return PressureGradNorm * val * val;
}

// Integer power by repeated squaring (avoids std::pow).
static float IntPow (float base, int exp)
{
    float result = 1.f;
    while (exp) {
        if (exp & 1) result *= base;
        base *= base;
        exp >>= 1;
    }
    return result;
}

// --------- Calculations
// Operate on hot-array slots; Object is untouched until Update() scatters the results back.

static void CalculatePressure (int32_t k)
{
    const auto& p = ActiveScene.particles;
    if (p.eos == EosModel::Polytropic) {
        // Pure polytrope: density is always > 0 (it includes the self-term), so there's nothing to clamp.
        hotPressure[k] = p.stiffness * std::pow(hotDensity[k], PolytropicGamma);
        return;
    }

    const float frac = p.stiffness * p.targetDensity * (1.f / p.exponent);
    const float parenth = IntPow(hotDensity[k] / p.targetDensity, p.exponent) - 1;
    // Clamp negative pressure so that it is never attractive
    hotPressure[k] = std::max(0.f, frac * parenth);
}

static void CalculateDensity (int32_t k)
{
    const Vec2 pos = hotPos[k];
    float density = 0.f;

    auto accumulate = [&](const Vec2& bPos) {
        Vec2 diff = pos - bPos;
        float distSq = dot(diff, diff);
        if (distSq > RadiusSq) return; // includes the dist==0 self-term; TargetDensity assumes it

        density += DensityKernel(distSq);
    };

    ForEachNeighbor(pos, [&](int32_t j) { accumulate(hotPos[j]); });
    ForEachGhost(pos, [&](const Ghost& g) { accumulate(g.pos); }); // no self-exclusion, see Ghost comment
    ForEachObstacleGhost(pos, [&](const Ghost& g) { accumulate(g.pos); });
    ForEachPeriodicX(pos, [&](const Ghost& g) { accumulate(g.pos); }); // ditto — real neighbors, not self

    hotDensity[k] = density;
}

static void CalculatePressureForce (int32_t k)
{
    const auto& p = ActiveScene.particles;
    const float MinDensity = p.targetDensity * 0.01f;

    const Vec2  pos       = hotPos[k];
    const Vec2  vel       = hotVel[k];
    const float pDensity  = std::max(hotDensity[k], MinDensity);
    const float pPressure = hotPressure[k];
    const bool  morris    = p.viscosityModel == ViscosityModel::Morris;
    Vec2 forceVec(0.f, 0.f);

    // Pairwise force shared by real neighbors and ghost images.
    auto accumulate = [&](const Vec2& bPos, const Vec2& bVel, float bDensityRaw, float bPressure) {
        Vec2 difference = pos - bPos;
        float distSq = dot(difference, difference);
        // Reject on squared distance before the sqrt; most candidates in the 3x3 block fail.
        if (distSq >= RadiusSq || distSq == 0.f) return;

        float dist = std::sqrt(distSq);
        Vec2 dir = difference / dist;

        float grad = PressureKernelGradient(dist);
        float bDensity = std::max(bDensityRaw, MinDensity);
        float coefficient = pPressure / (pDensity * pDensity) + bPressure / (bDensity * bDensity);

        Vec2 velDiff = vel - bVel;
        float approach = dot(velDiff, difference);
        if (!morris && approach < 0.f) {
            const float mu = Radius * approach / (distSq + 0.01f * RadiusSq);
            const float avgDensity = (pDensity + bDensity) * 0.5f;
            coefficient += (-p.viscosity * SoundSpeed * mu
                             + p.viscosityQuadratic * mu * mu) / avgDensity;
        }

        forceVec -= dir * (grad * coefficient);

        // Morris, Fox & Zhu (1997) viscosity, in unit-mass / number-density form:
        //   a_i += nu * (1/rho_i + 1/rho_j) * (r * dW/dr) / (r^2 + 0.01 h^2) * (v_i - v_j)
        // Brookshaw's SPH Laplacian, tending to nu * laplacian(v). dW/dr < 0, so it always drags v_i
        // toward v_j along the full relative velocity (Monaghan's term only sees the part along
        // `dir`). Wall ghosts carry -v under no-slip, pinning u = 0 at the wall.
        if (morris) {
            const float lap = p.kinematicViscosity * (1.f / pDensity + 1.f / bDensity)
                            * dist * grad / (distSq + 0.01f * RadiusSq);
            forceVec += velDiff * lap;
        }
    };

    ForEachNeighbor(pos, [&](int32_t j) {
        if (j == k) return;
        accumulate(hotPos[j], hotVel[j], hotDensity[j], hotPressure[j]);
    });

    // No self-exclusion: a particle right at the wall does feel a force from its own mirror image
    ForEachGhost(pos, [&](const Ghost& g) {
        accumulate(g.pos, g.vel, hotDensity[g.src], hotPressure[g.src]);
    });
    ForEachObstacleGhost(pos, [&](const Ghost& g) {
        accumulate(g.pos, g.vel, hotDensity[g.src], hotPressure[g.src]);
    });
    ForEachPeriodicX(pos, [&](const Ghost& g) {
        accumulate(g.pos, g.vel, hotDensity[g.src], hotPressure[g.src]);
    });

    hotForce[k] = forceVec;
}

// ------------- Simulation
void Init() {
    const auto& particles = ActiveScene.particles;
    const float radius    = particles.circleRadius;

    // A scene without groups spawns the one block its SceneSpawn describes.
    std::vector<SceneGroup> groups;
    if (ActiveScene.useGroups) groups = ActiveScene.groups;
    else                       groups.push_back(SpawnBlockAsGroup(ActiveScene, ScreenHalfWidth(), ScreenHalfHeight()));

    // The box every group fits in — the frame a color pattern is laid over.
    float boxMinX = 0.f, boxMinY = 0.f, boxMaxX = 0.f, boxMaxY = 0.f;
    bool  boxSet = false;
    for (const SceneGroup& g : groups) {
        const float hx = g.shape == EventShape::Circle ? g.radius : g.halfW;
        const float hy = g.shape == EventShape::Circle ? g.radius : g.halfH;
        boxMinX = boxSet ? std::min(boxMinX, g.cx - hx) : g.cx - hx;
        boxMaxX = boxSet ? std::max(boxMaxX, g.cx + hx) : g.cx + hx;
        boxMinY = boxSet ? std::min(boxMinY, g.cy - hy) : g.cy - hy;
        boxMaxY = boxSet ? std::max(boxMaxY, g.cy + hy) : g.cy + hy;
        boxSet = true;
    }
    const PatternSampler pattern(ActiveScene.colorPattern, boxMinX, boxMinY, boxMaxX, boxMaxY,
                                 ActiveScene.blendColorRegions ? ActiveScene.blendStrength : 0.f, ActiveScene.blendRadius);

    Color baseColor = particles.circleColor; // the group being spawned right now
    auto spawnAt = [&](float x, float y) {
        // Obstacles are solid: nothing spawns inside one (grown by the particle radius so a
        // particle never starts overlapping the surface).
        if (InsideAnyObstacle(ActiveScene.obstacles, x, y, radius)) return;
        Object& o = CreateObject(x, y, Renderable{ .color = baseColor, .shader = Shader::Circle, .geometry = Circle{radius} });
        if (Color pc; pattern.Sample(x, y, pc)) { o.renderable.color = pc; o.pinnedColor = true; }
        if (ActiveScene.blendColorRegions) {
            if (Color bc = o.renderable.color; BlendColorRegions(ActiveScene.colorRegions, x, y, bc, ActiveScene.blendStrength, ActiveScene.blendRadius)) { o.renderable.color = bc; o.pinnedColor = true; }
            return;
        }
        for (const SceneColorRegion& r : ActiveScene.colorRegions) {
            const float dx = x - r.cx, dy = y - r.cy;
            const bool inside = r.shape == EventShape::Circle
                ? dx * dx + dy * dy <= r.radius * r.radius
                : std::fabs(dx) <= r.halfW && std::fabs(dy) <= r.halfH;
            if (inside) { o.renderable.color = r.color; o.pinnedColor = true; }
        }
    };

    // See ForEachGroupPoint (Scene.h): a circle is a Fibonacci spiral with one particle per
    // spacing^2 (preserving the calibrated initial density); a rect is a square lattice.
    for (const SceneGroup& g : groups) {
        baseColor = g.customColor ? g.color : particles.circleColor;
        ForEachGroupPoint(g, GroupSpacing(g, radius), spawnAt);
    }
}

void Update(float) {
    // A zero/negative/non-finite smoothing radius degenerates the grid and kernels: skip the pass.
    if (!(CellSize > 0.f) || !std::isfinite(CellSize) || objects.empty()) return;
    BuildGrid();

    const int32_t n = static_cast<int32_t>(hotPos.size());

    // Each k writes only its own hot slot, so these are datarace-free; ParallelFor blocks, so
    // density/pressure are settled before the force pass.
    ParallelFor(n, [](int32_t begin, int32_t end) {
        for (int32_t k = begin; k < end; k++) { CalculateDensity(k); CalculatePressure(k); }
    });
    ParallelFor(n, [](int32_t begin, int32_t end) {
        for (int32_t k = begin; k < end; k++) CalculatePressureForce(k);
    });

    // Self-gravity on top of the SPH pressure force; the tree is only built when a scene enables it.
    const auto& grav = ActiveScene.gravity;
    if (grav.enabled) {
        if (grav.method == GravityMethod::Genuine2D) {
            Gravity::BuildTree(hotPos, n);
            ParallelFor(n, [](int32_t begin, int32_t end) {
                for (int32_t k = begin; k < end; k++) hotGravAcc[k] = Gravity::EvaluateForce2D(k, hotPos);
            });
            if (grav.validate) Gravity::ValidateAccuracy2D(hotPos, n, 500);
        } else {
            Gravity::BuildTree(hotPos, n);
            ParallelFor(n, [](int32_t begin, int32_t end) {
                for (int32_t k = begin; k < end; k++) hotGravAcc[k] = Gravity::EvaluateForce(k, hotPos);
            });
            if (grav.validate) Gravity::ValidateAccuracy(hotPos, n, 500);
        }
    }

    // Scatter hot results back onto the real objects.
    const Vec2 force = ActiveScene.physics.force;
    for (int32_t k = 0; k < n; k++) {
        Object& o = objects[hotToReal[k]];
        o.density   = hotDensity[k];
        o.pressure  = hotPressure[k];
        o.acc      += hotForce[k];
        o.acc      += force;
        if (grav.enabled) o.acc += hotGravAcc[k];
    }
}
