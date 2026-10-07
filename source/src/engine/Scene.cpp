#include "engine/Scene.h"
#include "engine/SceneHelpers.h"
#include "physics/Gravity.h"
#include <algorithm>
#include <cmath>

// Defined in physics/Sph.cpp: recomputes the cached SPH kernel constants derived from SceneParticles.
void RecomputeSphConstants();

namespace {
    Scene MakeOpenTank(int gridCountX = 100, int gridCountY = 100) {
        Scene s{
            .name = "Open Tank",
            .description = "Full-window box; a centered block of fluid falls and settles.",
        };
        // This scene was tuned against free-slip walls, so don't inherit ScenePhysics::noSlipWalls'
        // default (no-slip ghosts add much stronger wall drag).
        s.physics.noSlipWalls = false;
        s.spawn.gridCountX = gridCountX;
        s.spawn.gridCountY = gridCountY;
        // Measured real-time-stable grid size.
        s.realTimeLimitX = 125;
        s.realTimeLimitY = 125;
        return s;
    }

    // Force-driven planar Poiseuille flow: no gravity, a uniform horizontal force through a narrow
    // no-slip channel with a periodic left/right boundary (an effectively infinite channel). Wall
    // drag should develop the parabolic velocity profile.
    //
    // Geometry is sized from the particle grid, not the window: gridCountY rows tall and
    // gridCountX columns long, in particle spacings, so
    //  - the fluid fills the channel wall to wall, with the outer rows half a spacing from each
    //    wall (where the mirrored ghost rows put it); a looser fit never feels wall drag, since
    //    WCSPH clamps negative pressure and the block doesn't expand to the walls.
    //  - the periodic length is a whole number of spacings, so there is no density gap at the seam.
    //  - H and L don't change with the window or recording resolution.
    // The flow is x-invariant, so 160 columns (1440 px, 5H) is long enough for bin statistics.
    Scene MakePoiseuille(int gridCountX = 160, int gridCountY = 32) {
        Scene s = MakeOpenTank(gridCountX, gridCountY);
        s.name        = "Poiseuille Flow";
        s.description = "Periodic horizontal channel; a uniform force (no gravity) drives fluid "
                         "left-to-right, developing the classic parabolic velocity profile against "
                         "the no-slip walls.";
        const float spacing = s.particles.circleRadius * 3.0f; // must match Init()'s spacing
        s.boundary.halfWidthPx  = 0.5f * static_cast<float>(gridCountX) * spacing; // L/2
        s.boundary.halfHeightPx = 0.5f * static_cast<float>(gridCountY) * spacing; // H/2
        s.boundary.periodicX    = true;
        // Rest density is what the 2D Poly6 kernel reports on the spawn lattice (spacing 9, h = 15,
        // self-term included: 1.20895e-2), not the shared default (7.90e-3). With the default the
        // lattice starts 53% over-compressed and shears like an elastic solid instead of diffusing
        // momentum. With this value the Morris run tracks the analytic start-up profile to <0.5% of
        // u_max. Recompute if smoothingRadius or circleRadius changes.
        s.particles.targetDensity = 1.20895e-2f;
        // Viscosity, set up so either model can be picked without retuning the force below:
        //  - Morris: nu = 800 px^2/s directly.
        //  - Monaghan: alpha = 4, whose estimated effective viscosity alpha*h*c/8 ~ 825 px^2/s
        //    (c = sqrt(stiffness) ~ 110 px/s) is within ~3% of that. beta = 0, since the
        //    quadratic term only matters for shocks.
        s.particles.viscosityModel     = ViscosityModel::Monaghan;
        s.particles.viscosity          = 4.0f;
        s.particles.viscosityQuadratic = 0.0f;
        s.particles.kinematicViscosity = 800.f;
        // Driving force from a target centreline speed. Steady Poiseuille flow peaks at
        // u_max = F H^2 / (8 nu); WCSPH needs u_max << c, so aim for Mach ~0.1: u_max = 11 px/s
        // -> F = 8 nu u_max / H^2 (= 0.849 px/s^2 at H = 288 px). Recomputed from H, so any
        // gridCountY keeps the same Mach number.
        {
            constexpr float TargetUMax = 11.f;
            const float H = 2.f * s.boundary.halfHeightPx;
            s.physics.force = Vec2(8.f * s.particles.kinematicViscosity * TargetUMax / (H * H), 0.f);
        }
        s.physics.noSlipWalls   = true; // the wall drag this scene demonstrates
        s.particles.circleColor = { 0.2f, 0.9f, 0.8f, 1.0f };
        // Not independently measured: this scene's own default grid.
        s.realTimeLimitX = 160;
        s.realTimeLimitY = 32;
        return s;
    }

    // Open Tank's container without gravity, with the lattice sized by Init()
    // (SceneSpawn::fillBoundary) to fill it wall to wall.
    Scene MakePacked() {
        Scene s = MakeOpenTank();
        s.name        = "Packed";
        s.description = "Full-window box completely packed with fluid, no gravity; the count is "
                         "whatever fills the window.";
        s.spawn.fillBoundary = true;
        // Slightly over-packed: spacing 0.92x the rest spacing (+~18% particles, ~1.18x the rest
        // density), so the fluid fills the box with no gaps. targetDensity is untouched.
        s.spawn.spacingScale = 0.92f;
        s.physics.force = Vec2(0.f, 0.f); // no gravity
        // Lattice-sum rest density at spacing 9, h = 15 (see MakePoiseuille).
        s.particles.targetDensity = 1.20895e-2f;
        return s;
    }

    // Open Tank's container and physics with no particles: a blank slate to build up from the Groups tab.
    Scene MakeEmpty() {
        Scene s = MakeOpenTank();
        s.name        = "Empty";
        s.description = "Open Tank's container with no particles; add some in the Groups tab.";
        s.useGroups   = true; // an empty group list spawns nothing
        s.realTimeLimitX = 0;
        s.realTimeLimitY = 0;
        return s;
    }

    // No external force: only mutual N-body attraction (genuine-2D treecode) against SPH pressure.
    // Uses Open Tank's full-window box, leaving room to contract before reaching a wall.
    Scene MakeSelfGravity(int gridCountX = 250, int gridCountY = 200) {
        Scene s = MakeOpenTank(gridCountX, gridCountY);
        s.name        = "Self-Gravity";
        s.description = "No external force; particles pull on each other via a genuine 2D (1/r) "
                         "self-gravity treecode while SPH pressure resists the collapse.";
        s.physics.force = Vec2(0.f, 0.f);

        s.gravity.enabled  = true;
        s.gravity.method   = GravityMethod::Genuine2D;

        // The default grid is the count the substeps and K/alpha calibration below were validated at.
        s.realTimeLimitX = 250;
        s.realTimeLimitY = 200;

        // Circular spawn: a radially symmetric start settles into the radially symmetric
        // equilibrium a Lane-Emden comparison assumes.
        s.spawn.circular = true;

        // Resolution scaling: the disk keeps the physical size the calibration below targets
        // (Init()'s diskRadius = circleRadius*3*sqrt(count/pi)) by scaling circleRadius and
        // smoothingRadius by sqrt(oldCount/newCount), which keeps the smoothingRadius/spacing
        // ratio. targetDensity scales as 1/scale^2, like a 2D areal density, since the 2D Poly6
        // kernel integrates to 1 over the disk. At gridCountX*gridCountY == OldCount, scale == 1
        // and this is a no-op. Circular spawn only uses the product gridCountX*gridCountY.
        constexpr int OldCount = 100 * 100;
        // s.spawn.gridCountX/Y are already set via MakeOpenTank(...) above; the default
        // 250*200 == 50000 is the count the calibration below was validated at.
        const float scale = std::sqrt(static_cast<float>(OldCount)
                                     / static_cast<float>(s.spawn.gridCountX * s.spawn.gridCountY));
        s.particles.circleRadius    *= scale;
        s.particles.smoothingRadius *= scale;
        s.particles.targetDensity   /= (scale * scale);

        // CalculateDensity is an unweighted count, so the gravitating mass (particleCount *
        // particleMass) is independent of the scaling above. Scaling particleMass by
        // scale^2 = oldCount/newCount holds the enclosed mass M(r) fixed, so gravity doesn't
        // overpower the recalibrated pressure. A no-op at the default count.
        s.gravity.particleMass *= scale * scale;

        // Substeps raised from the default 12: a finer grid raises the CFL number
        // (soundspeed/smoothingRadius at targetDensity), and 50k particles was measured to need 96.
        // Fixed regardless of count, so a much larger count may need more.
        s.physics.substeps = 96;

        // Softening ~= smoothing radius (as SceneGravity's default); set after the resolution
        // scaling so it picks up the scaled smoothingRadius.
        s.gravity.softening = s.particles.smoothingRadius;

        // Pure polytrope: a Lane-Emden equilibrium assumes pressure = K*density^gamma with no
        // offset, and WCSPH's -1 term skews the fit near the cloud's edge. n=1/6 gives gamma=7.
        s.particles.eos             = EosModel::Polytropic;
        s.particles.polytropicIndex = 1.f / 6.f;

        // Stiffness chosen so the Lane-Emden (cylindrical k=1, n=1/6) equilibrium has surface
        // radius TargetR. K is a free parameter of the problem (like G), so pick the equilibrium
        // radius and solve for the K that produces it:
        //
        //   length scale:   alpha = TargetR / xi1
        //   mass:           N = 2*pi * rho_c * alpha^2 * xi1 * |theta'(xi1)|
        //                   (rho_c is a number density, which is what CalculateDensity's
        //                   unweighted kernel sum gives)
        //   hydrostatics:   alpha^2 = (n+1) * K * rho_c^(1/n - 1) / (4*pi*G*m)
        //
        // The 4*pi*G*m source comes from EvaluateForce2D (accel = 2*G*m/r per particle). Since m
        // appears explicitly, the same TargetR holds at any particle count.
        //
        // Measured R comes out 1.7% below TargetR at every N: the discrete Spiky-gradient
        // normalization (~0.83 at ~6 neighbours inside h) weakens the SPH pressure force ~17%.
        // That is deliberately not folded into K.
        //
        // Recompute Xi1 and DThetaXi1 (scratchpad/lane_emden_2d.py) if polytropicIndex changes.
        constexpr float Xi1       = 2.060505463f;  // first zero of theta(xi), n=1/6, k=1
        constexpr float DThetaXi1 = 0.8777755755f; // |dtheta/dxi| at xi1
        constexpr float TargetR   = 540.f;         // px, desired equilibrium surface radius
        const int   count = s.spawn.gridCountX * s.spawn.gridCountY;
        const float n     = s.particles.polytropicIndex;
        const float m     = s.gravity.particleMass;
        const float alpha = TargetR / Xi1;
        const float rhoC  = static_cast<float>(count)
                          / (2.f * 3.14159265f * alpha * alpha * Xi1 * DThetaXi1);
        s.particles.stiffness = 4.f * 3.14159265f * s.gravity.g * m * alpha * alpha
                              * std::pow(rhoC, 1.f - 1.f / n) / (n + 1.f);
        return s;
    }
}

const std::vector<Scene> ScenePresets = {
    MakeOpenTank(), MakePoiseuille(), MakeSelfGravity(), MakePacked(), MakeEmpty(),
};

Scene BuildScene(int presetIndex, int gridCountX, int gridCountY) {
    gridCountX = std::max(gridCountX, 1);
    gridCountY = std::max(gridCountY, 1);
    switch (presetIndex) {
        case 0:  return MakeOpenTank(gridCountX, gridCountY);
        case 1:  return MakePoiseuille(gridCountX, gridCountY);
        case 2:  return MakeSelfGravity(gridCountX, gridCountY);
        case 3:  return MakePacked();
        case 4:  return MakeEmpty();
        default: return ScenePresets[presetIndex];
    }
}

void AutoTuneSph(SceneParticles& p) {
    const SceneParticles ref; // the default-initialized values are the calibration
    const float q  = std::max(p.circleRadius, 0.5f) / ref.circleRadius;
    const float q2 = q * q;
    p.smoothingRadius    = ref.smoothingRadius * q;
    p.targetDensity      = ref.targetDensity / q2;
    if (p.eos == EosModel::WCSPH) p.stiffness = ref.stiffness * q2;
    p.kinematicViscosity = ref.kinematicViscosity * q2;
}

SceneGroup SpawnBlockAsGroup(Scene& scene, float halfWidth, float halfHeight) {
    auto& spawn = scene.spawn;
    const float spacing = scene.particles.circleRadius * 3.0f * spawn.spacingScale;
    constexpr float Pi = 3.14159265358979323846f;

    // Bounds the lattice so gridCountX * gridCountY can't overflow int or request gigabytes.
    auto cellsAcross = [&](float extent) {
        const float cells = 2.f * extent / spacing;
        return std::isfinite(cells) ? static_cast<int>(std::min(cells, static_cast<float>(MaxSpawnParticles))) : 1;
    };
    if (spawn.fillBoundary && !spawn.circular) {
        spawn.gridCountX = std::max(cellsAcross(halfWidth),  1);
        spawn.gridCountY = std::max(cellsAcross(halfHeight), 1);
    }
    spawn.gridCountX = std::clamp(spawn.gridCountX, 1, MaxSpawnParticles);
    spawn.gridCountY = std::clamp(spawn.gridCountY, 1, MaxSpawnParticles / spawn.gridCountX);

    SceneGroup g;
    g.spacingScale = spawn.spacingScale;
    g.color        = scene.particles.circleColor; // shown by the gizmo; customColor stays off
    g.cx = spawn.offsetXFrac * halfWidth;
    g.cy = 0.f;
    if (spawn.circular) {
        // gridCountX * gridCountY particles on a disk of the lattice's areal density.
        g.shape  = EventShape::Circle;
        g.radius = spacing * std::sqrt(static_cast<float>(spawn.gridCountX * spawn.gridCountY) / Pi);
    } else {
        g.shape = EventShape::Rect;
        g.halfW = static_cast<float>(spawn.gridCountX) * spacing * 0.5f;
        g.halfH = static_cast<float>(spawn.gridCountY) * spacing * 0.5f;
    }
    return g;
}

void SetSceneResolution(Scene& s, int width, int height) {
    constexpr int MinRes = 64, MaxRes = 8192;
    width  = std::clamp(width,  MinRes, MaxRes) & ~1;
    height = std::clamp(height, MinRes, MaxRes) & ~1;

    SceneBoundary& b = s.boundary;
    const float sx = static_cast<float>(width)  / static_cast<float>(std::max(b.resWidth,  1));
    const float sy = static_cast<float>(height) / static_cast<float>(std::max(b.resHeight, 1));
    b.resWidth  = width;
    b.resHeight = height;
    if (sx == 1.f && sy == 1.f) return;

    const float sr = std::min(sx, sy); // lengths with no axis (radii, cell sizes) shrink with the tighter one
    auto scaleShape = [&](auto& o) {
        o.cx *= sx; o.cy *= sy;
        o.radius *= sr;
        o.halfW *= sx; o.halfH *= sy;
    };
    for (SceneEvent& e : s.events)           scaleShape(e);
    for (SceneColorRegion& r : s.colorRegions) scaleShape(r);
    for (SceneObstacle& o : s.obstacles)     scaleShape(o);
    for (SceneGroup& g : s.groups)           scaleShape(g);

    if (b.halfWidthPx  > 0.f) b.halfWidthPx  *= sx;
    if (b.halfHeightPx > 0.f) b.halfHeightPx *= sy;

    if (!s.spawn.fillBoundary) {
        auto scaleCount = [](int count, float scale) {
            return std::clamp(static_cast<int>(std::lround(static_cast<float>(count) * scale)), 1, MaxSpawnParticles);
        };
        s.spawn.gridCountX = scaleCount(s.spawn.gridCountX, sx);
        s.spawn.gridCountY = scaleCount(s.spawn.gridCountY, sy);
    }

    s.colorPattern.cellSize *= sr;
    s.blendRadius           *= sr;
}

int SceneParticleCount(const Scene& scene) {
    if (!scene.useGroups) return scene.spawn.gridCountX * scene.spawn.gridCountY;
    long long total = 0;
    for (const SceneGroup& g : scene.groups)
        total += GroupParticleCount(g, GroupSpacing(g, scene.particles.circleRadius));
    return static_cast<int>(std::min<long long>(total, MaxSpawnParticles));
}

Scene ActiveScene = ScenePresets[0];

void LoadScene(const Scene& scene) {
    ActiveScene = scene;
    RecomputeSphConstants();
    Gravity::SetParams({
        .g                = scene.gravity.g,
        .particleMass     = scene.gravity.particleMass,
        .softening        = scene.gravity.softening,
        .macTheta         = scene.gravity.macTheta,
        .maxLeafParticles = scene.gravity.maxLeafParticles,
    });
}
