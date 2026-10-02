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
// Uniform grid (cell = 2*SmoothingRadius) as a CSR-sorted flat hash rather than
// unordered_map<Cell, vector<int>> — no per-cell heap allocations, and cellStart[h]..cellStart[h+1]
// is a contiguous slice instead of a pointer-chased bucket walk.
//
// The same sort reorders a "hot" copy of pos/vel/density/pressure/force by cell, kept separate
// from Object/Renderable. The density/force loops then only touch ~40 bytes/particle and read
// mostly-contiguous memory per cell, instead of scattering through `objects` in creation order.
//
// The table is unbounded, so cells can collide in the hash; cellX/cellY reject false positives so
// a collision only wastes a candidate rather than double-counting one.
namespace {
    // Derived from ActiveScene.particles; recomputed by RecomputeSphConstants() whenever a scene
    // is loaded, not per-frame, so the hot loop still just reads a plain float.
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

static int32_t CellCoord(float p) { return static_cast<int32_t>(std::floor(p / CellSize)); }

// Teschner et al.'s spatial-hash primes; unsigned so the multiply can't hit signed overflow UB.
static uint32_t HashCell(int32_t x, int32_t y) {
    return (static_cast<uint32_t>(x) * 73856093u) ^ (static_cast<uint32_t>(y) * 19349663u);
}

static uint32_t NextPow2(uint32_t v) {
    v--; v |= v >> 1; v |= v >> 2; v |= v >> 4; v |= v >> 8; v |= v >> 16;
    return v + 1;
}

// Rebuilt once per Update(). All arrays are namespace-scope and reused across calls, so a stable
// particle count allocates nothing after warm-up.
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

// Invokes fn(int32_t hotSlot) for every hot-array slot sharing pos's cell or one of its 8
// neighbors. `pos` need not belong to a real particle (see ForEachGhost); a caller wanting to
// exclude itself compares hot-slot indices.
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

// Mirror-image ("method of images") boundary particles. SPH density near a wall is under-counted
// (missing kernel mass from beyond the wall), which zero-clamps pressure there and lets particles
// pile up against it. Reflecting nearby real particles across the wall and folding them into the
// density/pressure sums like normal neighbors restores that missing mass directly.
//
// Position mirrors exactly; velocity mirrors with the wall-normal component negated (free-slip);
// density/pressure carry over unchanged (the field is assumed locally symmetric across the wall).
// Carries the hot-slot index rather than density/pressure directly — those are still being written
// by other threads during the (parallel) density phase, so reading them here would race.
struct Ghost { Vec2 pos; Vec2 vel; int32_t src; };

struct WallMirror { bool active = false; float wall = 0.f; };

// Which wall (if either) `coord` is within Radius of, on one axis. `half` is the true container
// edge (ScreenHalfWidth/Height), not the particle-radius-inset range particle centers clamp to.
static WallMirror NearestWall(float coord, float half) {
    if (coord + half < Radius) return { true, -half };
    if (half - coord < Radius) return { true, half };
    return {};
}

// Invokes fn(const Ghost&) for every real particle whose mirror image — across whichever wall(s)
// `pos` is within Radius of, including the diagonal double-mirror in a corner — falls within
// Radius of `pos`. No-op away from all walls.
template <typename Fn>
static void ForEachGhost(const Vec2& pos, Fn&& fn) {
    // No X wall to mirror across when the domain wraps — ForEachPeriodicX below handles that axis.
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

            // Free-slip: only the wall-normal velocity component reverses (impermeable, but no
            // drag on the tangential flow). No-slip: the whole velocity reverses, so the SPH
            // viscosity force between a real particle and its ghost averages their velocities
            // toward zero right at the wall in every direction — that's what actually produces
            // the wall drag a Poiseuille profile depends on; the friction hack in resolveAxis
            // alone only fires on an actual wall collision, which barely happens once flow settles.
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

// Periodic-image particles for a wrapped X boundary (see SceneBoundary::periodicX): a particle
// near the right edge needs the real particles just past the left edge as neighbors (and vice
// versa) for its density/pressure to come out as if the domain just continued — exactly what an
// infinite channel would give. Unlike ForEachGhost's wall mirror, this is a translation (by the
// domain width), not a reflection, and it carries real particles' velocities through unchanged.
template <typename Fn>
static void ForEachPeriodicX(const Vec2& pos, Fn&& fn) {
    if (!ActiveScene.boundary.periodicX) return;
    const float hw          = ScreenHalfWidth();
    const float domainWidth = 2.f * hw;
    // A particle near one of the channel's four corners (both a Y wall and the periodic seam)
    // needs the shifted-AND-mirrored diagonal image too — ForEachGhost never produces it since
    // periodicX forces its own X-mirror off, leaving that corner case to be handled here instead.
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
                // Same free-slip/no-slip rule as ForEachGhost's own Y-mirror case (see its
                // comment) — the periodic X shift never touches velocity either way.
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
    // Derived from Radius/RadiusSq (and Stiffness) by RecomputeSphConstants(); computed once per
    // scene load rather than per neighbor pair.
    float DensityNorm      = 0.f;
    float PressureGradNorm = 0.f;
    float SoundSpeed       = 0.f;
    float PolytropicGamma  = 0.f; // 1 + 1/polytropicIndex; only used when eos == Polytropic
}

// Recomputes every SPH constant derived from ActiveScene.particles. Called by LoadScene()
// whenever a new scene is applied — never per-frame, so the hot loop below just reads plain
// floats instead of paying for std::pow/std::sqrt or a scene-struct indirection per particle pair.
void RecomputeSphConstants() {
    Radius   = ActiveScene.particles.smoothingRadius;
    RadiusSq = Radius * Radius;
    CellSize = 2.f * Radius;

    // radius^5 / radius^8, expanded by hand since std::pow isn't constexpr — turns a std::pow
    // call on every particle pair into a plain multiply against a value computed once.
    const float radius4 = RadiusSq * RadiusSq;
    const float radius5 = radius4 * Radius;
    const float radius8 = radius4 * radius4;
    // True 2D Poly6/Spiky normalizations (this sim is 2D — see Genuine2D gravity), not the 3D ones
    // (315/64/h^9, -45/h^6) an earlier version of this code used: integrating each kernel over a
    // disk of radius h (rather than a 3D ball) gives 4/(pi*h^8) for Poly6 and -30/(pi*h^5) for
    // Spiky's gradient. This changes the actual density values CalculateDensity reports for a given
    // packing, which invalidates every targetDensity/stiffness calibration tuned against the old 3D
    // norm (see Scene.cpp's MakeSelfGravity — its Lane-Emden stiffness derivation and its
    // resolution's targetDensity/scale^3 scaling both assumed the old normalization and need
    // re-deriving against this one).
    DensityNorm      = 4.f / (Pi * radius8);
    PressureGradNorm = -30.f / (Pi * radius5);
    PolytropicGamma  = 1.f + 1.f / ActiveScene.particles.polytropicIndex;

    // Characteristic speed for the artificial-viscosity term (CalculatePressureForce), not an
    // exact per-particle quantity — just a reference sqrt(dP/drho). WCSPH's stiffness IS
    // approximately that (c^2 ~ stiffness by construction), but the Polytropic EOS's stiffness is
    // K in pressure=K*density^gamma, a completely different scale (~1e23 for this sim's Self-Gravity
    // calibration, vs WCSPH's ~1e4) — sqrt(K) directly would give a nonsense "sound speed" and blow
    // the viscosity term up on the first substep. Evaluate the real dP/drho = K*gamma*rho^(gamma-1)
    // at targetDensity instead.
    if (ActiveScene.particles.eos == EosModel::Polytropic) {
        const auto& p = ActiveScene.particles;
        SoundSpeed = std::sqrt(p.stiffness * PolytropicGamma * std::pow(p.targetDensity, PolytropicGamma - 1.f));
    } else {
        SoundSpeed = std::sqrt(ActiveScene.particles.stiffness);
    }
}

// Poly6-style kernel. Takes squared distance directly — callers already have it, and the kernel
// only ever needs dst^2, so there's no reason to sqrt just to square it back.
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

// Repeated-squaring integer power — avoids std::pow's exp/log path for a fixed small exponent.
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
        // Pure polytrope: no targetDensity offset, so unlike WCSPH below there's nothing to clamp
        // — density is always > 0 (CalculateDensity always includes the dist==0 self-term), and a
        // positive base raised to any real exponent stays positive.
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

    // Shared pairwise force, used for both real neighbors and ghost mirror images below.
    auto accumulate = [&](const Vec2& bPos, const Vec2& bVel, float bDensityRaw, float bPressure) {
        Vec2 difference = pos - bPos;
        float distSq = dot(difference, difference);
        // Reject on squared distance before paying for a sqrt — the 3x3 cell block covers a much
        // larger area than the smoothing radius circle, so most candidates fail this check.
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

        // Morris, Fox & Zhu (1997) viscosity, in this code's unit-mass / number-density form:
        //   a_i += nu * (1/rho_i + 1/rho_j) * (r * dW/dr) / (r^2 + 0.01 h^2) * (v_i - v_j)
        // i.e. Brookshaw's SPH Laplacian, which for smooth v tends to nu * laplacian(v) (in 2D
        // and 3D alike). dW/dr < 0, so this always drags v_i toward v_j — along the FULL relative
        // velocity, unlike Monaghan's term above, which only sees its component along `dir`.
        // Uses the Spiky gradient, whose kernel is normalised in 2D like the pressure term's.
        // Wall ghosts carry -v under no-slip (see ForEachGhost), pinning u = 0 at the wall.
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
    ForEachPeriodicX(pos, [&](const Ghost& g) {
        accumulate(g.pos, g.vel, hotDensity[g.src], hotPressure[g.src]);
    });

    hotForce[k] = forceVec;
}

// ------------- Simulation
void Init() {
    const auto& particles = ActiveScene.particles;
    auto&       spawn     = ActiveScene.spawn;

    const float radius  = particles.circleRadius;
    const float spacing = radius * 3.0f * spawn.spacingScale;
    if (spawn.fillBoundary && !spawn.circular) {
        spawn.gridCountX = std::max(static_cast<int>(2.f * ScreenHalfWidth()  / spacing), 1);
        spawn.gridCountY = std::max(static_cast<int>(2.f * ScreenHalfHeight() / spacing), 1);
    }
    const float centerX = spawn.offsetXFrac * ScreenHalfWidth();

    // The block the spawn lattice occupies — the frame a color pattern is laid over.
    const float blockHalfW = spawn.circular ? spacing * std::sqrt(static_cast<float>(spawn.gridCountX * spawn.gridCountY) / Pi)
                                            : spawn.gridCountX * spacing * 0.5f;
    const float blockHalfH = spawn.circular ? blockHalfW : spawn.gridCountY * spacing * 0.5f;
    const PatternSampler pattern(ActiveScene.colorPattern, centerX - blockHalfW, -blockHalfH, centerX + blockHalfW, blockHalfH);

    auto spawnAt = [&](float x, float y) {
        Object& o = CreateObject(x, y, Renderable{ .color = particles.circleColor, .shader = Shader::Circle, .geometry = Circle{radius} });
        if (Color pc; pattern.Sample(x, y, pc)) { o.renderable.color = pc; o.pinnedColor = true; }
        for (const SceneColorRegion& r : ActiveScene.colorRegions) {
            const float dx = x - r.cx, dy = y - r.cy;
            const bool inside = r.shape == EventShape::Circle
                ? dx * dx + dy * dy <= r.radius * r.radius
                : std::fabs(dx) <= r.halfW && std::fabs(dy) <= r.halfH;
            if (inside) { o.renderable.color = r.color; o.pinnedColor = true; }
        }
    };

    if (spawn.circular) {
        // Exactly gridCountX*gridCountY particles, placed directly into a disk via a Fibonacci
        // ("sunflower"/Vogel) spiral instead of masking a square lattice against a circle -- that
        // approach only ever approximates the requested count (a circle inscribed in an NxN square
        // keeps just ~pi/4 of its points). The disk radius is sized so its areal density matches
        // the square lattice's (one particle per spacing^2), preserving the initial SPH density
        // this scene's constants were calibrated against.
        const int   count      = spawn.gridCountX * spawn.gridCountY;
        const float diskRadius = spacing * std::sqrt(static_cast<float>(count) / Pi);
        constexpr float GoldenAngle = 2.39996323f; // pi * (3 - sqrt(5))
        for (int i = 0; i < count; i++) {
            const float r     = diskRadius * std::sqrt((static_cast<float>(i) + 0.5f) / static_cast<float>(count));
            const float theta = static_cast<float>(i) * GoldenAngle;
            spawnAt(centerX + r * std::cos(theta), r * std::sin(theta));
        }
    } else {
        const float startX = centerX - (spawn.gridCountX - 1) * spacing / 2.0f;
        const float startY = -(spawn.gridCountY - 1) * spacing / 2.0f;
        for (int x = 0; x < spawn.gridCountX; x++)
            for (int y = 0; y < spawn.gridCountY; y++)
                spawnAt(startX + static_cast<float>(x) * spacing, startY + static_cast<float>(y) * spacing);
    }
}

void Update(float) {
    BuildGrid();

    const int32_t n = static_cast<int32_t>(hotPos.size());

    // Each k writes only its own hot slot, so these are datarace-free; ParallelFor blocks until a
    // call's chunks all finish, so density/pressure are fully settled before the force pass reads them.
    ParallelFor(n, [](int32_t begin, int32_t end) {
        for (int32_t k = begin; k < end; k++) { CalculateDensity(k); CalculatePressure(k); }
    });
    ParallelFor(n, [](int32_t begin, int32_t end) {
        for (int32_t k = begin; k < end; k++) CalculatePressureForce(k);
    });

    // Self-gravity, on top of the SPH pressure force above — off for most scenes (see
    // SceneGravity), so only pay for a tree build/traversal when a scene actually wants it.
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
