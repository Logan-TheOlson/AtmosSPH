#include "engine/Scene.h"
#include "physics/Gravity.h"
#include <algorithm>
#include <cmath>

// Defined in main.cpp — recomputes the SPH kernel constants (DensityNorm, PressureGradNorm, ...)
// that are derived from SceneParticles and cached for the hot loop rather than recomputed per pair.
void RecomputeSphConstants();

namespace {
    Scene MakeOpenTank(int gridCountX = 100, int gridCountY = 100) {
        Scene s{
            .name = "Open Tank",
            .description = "Full-window box; a centered block of fluid falls and settles.",
        };
        // ForEachGhost's no-slip ghosts (full velocity reversal, not just the wall-normal
        // component) were added for Poiseuille's wall drag and are much stronger than the old
        // occasional friction-on-contact — this scene predates and wasn't tuned against that
        // effect, so opt back into the original free-slip walls explicitly rather than silently
        // inheriting ScenePhysics::noSlipWalls' default.
        s.physics.noSlipWalls = false;
        s.spawn.gridCountX = gridCountX;
        s.spawn.gridCountY = gridCountY;
        // Measured real-time-stable grid size on the developer's own machine.
        s.realTimeLimitX = 125;
        s.realTimeLimitY = 125;
        return s;
    }

    // Classic force-driven planar Poiseuille flow: no gravity, just a uniform horizontal force
    // pushing fluid left-to-right through a narrow no-slip channel (solid top/bottom walls). The
    // left/right boundary is periodic — a particle exiting the right edge reappears at the left,
    // and main.cpp's ForEachPeriodicX mirrors real particles across that seam for density/pressure
    // too — so the channel behaves as if it were infinitely long, the standard setup for this
    // test. Viscous drag against the walls (vs. the free interior) should develop into the classic
    // parabolic velocity profile: fastest at the channel's center, near-zero at the walls.
    //
    // Geometry is sized from the particle grid, not the window: the channel is exactly
    // gridCountY rows tall and gridCountX columns long (in particle spacings), so
    //  - the fluid fills the channel wall to wall, with the outermost rows half a spacing from
    //    each wall (where the mirrored ghost rows put the wall exactly midway). The old
    //    heightFrac = 0.2 gave a 288 px channel around a ~190 px block; WCSPH clamps negative
    //    pressure, so the block never expanded to the walls and felt no wall drag at all.
    //  - the periodic length is a whole number of spacings, so the particle across the seam is
    //    exactly one spacing away (the old 280 x 9 = 2520 px block in a 2560 px window left a
    //    40 px density gap at the seam).
    //  - H and L no longer change with the window or recording resolution (see SceneBoundary).
    // The flow is x-invariant, so the channel only needs to be long enough for bin statistics;
    // 160 columns (1440 px, 5H) instead of 280 cuts the particle count ~1.7x for the same H.
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
        // Rest density = what the 2D Poly6 kernel actually reports on the spawn lattice (spacing
        // 9, h = 15, self-term included: 1.20895e-2), not the shared SceneParticles default
        // (7.90e-3, carried over from the old 3D-kernel calibration). With the default, the
        // channel started 53% over-compressed; with WCSPH's exponent 7 that's ~20x the natural
        // pressure scale, and the pressurised lattice sheared like an elastic solid — momentum
        // sloshed between centre and walls with a ~10 s period instead of diffusing. With this
        // value the Morris run tracks the analytic start-up profile to <0.5% of u_max.
        // (Recompute if smoothingRadius or circleRadius changes.)
        s.particles.targetDensity = 1.20895e-2f;
        // Viscosity. Both models are set up so either can be picked from the menu without
        // retuning the force below:
        //  - Morris: nu = 800 px^2/s directly.
        //  - Monaghan: alpha = 4, whose textbook effective viscosity alpha*h*c/8 ~ 825 px^2/s
        //    (c = sqrt(stiffness) ~ 110 px/s) is within ~3% of that — though for Monaghan it's
        //    only an estimate and has to be measured from the run. beta = 0: the quadratic term
        //    only matters for shocks, and dropping it keeps the viscous force linear in velocity
        //    (at these speeds its contribution would be negligible anyway).
        s.particles.viscosityModel     = ViscosityModel::Monaghan;
        s.particles.viscosity          = 4.0f;
        s.particles.viscosityQuadratic = 0.0f;
        s.particles.kinematicViscosity = 800.f;
        // Driving force from a target centreline speed, not tuned by eye. Steady Poiseuille flow
        // peaks at u_max = F H^2 / (8 nu); WCSPH is only near-incompressible for u_max << c, so
        // aim for Mach ~0.1: u_max = 11 px/s -> F = 8 nu u_max / H^2 (= 0.849 px/s^2 at the
        // default H = 288 px). Recomputed from H, so a different gridCountY keeps the same Mach
        // number. The old F = 100 would have given u_max ~ 1250 px/s, i.e. Mach ~11.
        {
            constexpr float TargetUMax = 11.f;
            const float H = 2.f * s.boundary.halfHeightPx;
            s.physics.force = Vec2(8.f * s.particles.kinematicViscosity * TargetUMax / (H * H), 0.f);
        }
        s.physics.noSlipWalls   = true; // the wall drag this whole scene exists to demonstrate
        s.particles.circleColor = { 0.2f, 0.9f, 0.8f, 1.0f };
        // Not independently measured — this scene's own (long, thin) default grid.
        s.realTimeLimitX = 160;
        s.realTimeLimitY = 32;
        return s;
    }

    // Same container as Open Tank (the full window and walls, but no gravity), but the lattice is
    // sized by Init() (SceneSpawn::fillBoundary) to fill it wall to wall instead of a centered block.
    Scene MakePacked() {
        Scene s = MakeOpenTank();
        s.name        = "Packed";
        s.description = "Full-window box completely packed with fluid, no gravity; the count is "
                         "whatever fills the window.";
        s.spawn.fillBoundary = true;
        // Slightly over-packed: lattice spacing 0.92x the rest spacing (+~18% particles, ~1.18x the
        // rest density), so the fluid starts under positive pressure and fills the box with
        // essentially no gaps. Done through particle count only -- targetDensity is untouched.
        s.spawn.spacingScale = 0.92f;
        s.physics.force = Vec2(0.f, 0.f); // no gravity: the packed fluid just sits until something (e.g. an event) moves it
        // Lattice-sum rest density at spacing 9, h = 15 (see the old Poiseuille/Event Box notes),
        // so the packed block starts at rest instead of over-compressed.
        s.particles.targetDensity = 1.20895e-2f;
        return s;
    }

    // No external force at all — the only thing holding the block together (or pulling it apart)
    // is mutual N-body attraction between particles, via the genuine-2D treecode, plus SPH
    // pressure resisting the collapse. Default window boundaries (full-window box, free-slip
    // walls from MakeOpenTank), so the block has room to contract before it ever reaches a wall.
    Scene MakeSelfGravity(int gridCountX = 250, int gridCountY = 200) {
        Scene s = MakeOpenTank(gridCountX, gridCountY);
        s.name        = "Self-Gravity";
        s.description = "No external force; particles pull on each other via a genuine 2D (1/r) "
                         "self-gravity treecode while SPH pressure resists the collapse.";
        s.physics.force = Vec2(0.f, 0.f);

        s.gravity.enabled  = true;
        s.gravity.method   = GravityMethod::Genuine2D;

        // This scene's own default grid is also the exact count its substep count (see below)
        // and K/alpha calibration (see the Xi1 comment below) were validated at running smoothly.
        s.realTimeLimitX = 250;
        s.realTimeLimitY = 200;

        // Circular spawn: a radially symmetric initial condition settles into a radially symmetric
        // equilibrium, which is what a Lane-Emden comparison (spherically/circularly symmetric by
        // construction) assumes — a rectangular block would settle off-center and skew any radial
        // density profile pulled from it.
        s.spawn.circular = true;

        // 50k particles — bumped up from this scene's original 10k baseline (OldCount below),
        // packed into the SAME physical disk size a different resolution of this scene would use
        // (same diskRadius, so the K/alpha calibration below targets the cloud this sim validated
        // against Lane-Emden) by scaling circleRadius by sqrt(oldCount/newCount): Init()'s
        // diskRadius = circleRadius*3*sqrt(count/pi), so this scale-factor machinery cancels any
        // count change relative to OldCount. smoothingRadius scales the same way to keep the same
        // smoothingRadius/spacing ratio (and hence a still-valid density kernel), which is also
        // what makes the particles render slightly smaller at this resolution — circleRadius
        // shrinks by the same sqrt(oldCount/newCount) factor. targetDensity scales as 1/scale^2,
        // same as a plain 2D areal density would: this kernel's DensityNorm ~ 1/h^8 with a
        // (h^2-r^2)^3 falloff (the true 2D Poly6 normalization — see main.cpp's DensityKernel/
        // DensityNorm) integrates to a dimensionless 1 over the disk, so a uniform spatial rescale
        // by `scale` moves every kernel evaluation — and hence the summed density — by exactly
        // 1/scale^2, matching real areal density's own 1/scale^2 falloff under the same rescale.
        // At gridCountX*gridCountY == OldCount, scale == 1 and all of
        // this is a no-op — the knob to turn for a different resolution. (gridCountX/Y need not be
        // square-ish: circular spawn only ever uses their product, see Init()'s circular branch.)
        constexpr int OldCount = 100 * 100;
        // s.spawn.gridCountX/Y are already gridCountX/gridCountY, set via the MakeOpenTank(...)
        // call above — default 250*200 == 50000, the resolution this scene's K/alpha calibration
        // below was validated against; any other count just re-derives the same calibration at
        // that count's own scale (see the comment block below).
        const float scale = std::sqrt(static_cast<float>(OldCount)
                                     / static_cast<float>(s.spawn.gridCountX * s.spawn.gridCountY));
        s.particles.circleRadius    *= scale;
        s.particles.smoothingRadius *= scale;
        s.particles.targetDensity   /= (scale * scale);

        // CalculateDensity's kernel sum has no mass weighting at all (see main.cpp — it's a plain
        // unit-per-particle count convolved with the kernel), so nothing above touches how much
        // *gravitating* mass this cloud has: that's particleCount*particleMass, entirely separate,
        // straight in Gravity's N-body sum. Left at particleMass=1, more particles at the same disk
        // size means more enclosed mass at every radius, so gravity would overpower the (correctly
        // recalibrated) pressure and the cloud would run away outward instead of sitting at
        // equilibrium. Scaling particleMass by scale^2 = oldCount/newCount holds enclosed-mass(r)
        // exactly fixed relative to OldCount (more particles x proportionally less mass each = the
        // same M(r) profile this scene's K was derived for) — a no-op at the current count.
        s.gravity.particleMass *= scale * scale;

        // substeps bumped from ScenePhysics's default (12, inherited from MakeOpenTank, and what
        // the original 10k baseline was validated stable at). A finer grid (smaller
        // smoothingRadius, stiffer EOS to match) pushes this scene's characteristic CFL number
        // (soundspeed/smoothingRadius, evaluated at targetDensity) up sharply — 50k particles (5x
        // the 10k baseline) was separately measured to need 96 substeps (8x). Fixed regardless of
        // gridCountX/gridCountY above (not re-derived from the CFL number per count), so a much
        // larger user-chosen count than the 50k this was tuned at may need more than 96 to stay
        // stable — not auto-scaled here.
        s.physics.substeps = 96;

        // Softening ~= smoothing radius, same "avoids the 1/r^2 singularity at ~mean interparticle
        // spacing" reasoning as SceneGravity's own default — kept in sync explicitly here (and
        // computed after the resolution scaling above, so it picks up the shrunk smoothingRadius).
        s.gravity.softening = s.particles.smoothingRadius;

        // Pure polytrope, not WCSPH's targetDensity-relative EOS: a Lane-Emden equilibrium is
        // derived against pressure = K*density^gamma with no offset, while WCSPH's -1 term only
        // becomes negligible once density >> targetDensity — it measurably skews the fit right
        // where the cloud's edge approaches targetDensity. n=1/6 keeps the same effective gamma=7
        // stiffness this scene used before switching off WCSPH.
        s.particles.eos             = EosModel::Polytropic;
        s.particles.polytropicIndex = 1.f / 6.f;

        // Stiffness chosen so this cloud's Lane-Emden (cylindrical k=1, n=1/6) equilibrium has a
        // prescribed surface radius TargetR -- derived exactly, with no empirical factor.
        //
        // In a polytrope test K is a free parameter of the problem (like G): every K has its own
        // equilibrium. So rather than trying to make the *spawned* cloud start in equilibrium
        // (impossible with a uniform spawn anyway: a Lane-Emden star with the spawn's radius and
        // central density holds only 1/1.174 of the spawn's mass), pick the equilibrium radius and
        // solve for the K that produces it:
        //
        //   length scale:   alpha = TargetR / xi1
        //   mass:           N = 2*pi * rho_c * alpha^2 * xi1 * |theta'(xi1)|
        //                   (rho_c is a *number* density -- with the 2D Poly6 normalization,
        //                   CalculateDensity's unweighted kernel sum is exactly that)
        //   hydrostatics:   alpha^2 = (n+1) * K * rho_c^(1/n - 1) / (4*pi*G*m)
        //
        // The 4*pi*G*m source term comes from Gravity.cpp's EvaluateForce2D (accel = 2*G*m/r per
        // particle, i.e. Poisson source 4*pi*G*m*numberDensity). The pressure side matches
        // CalculatePressureForce, which adds -sum (P_i/rho_i^2 + P_j/rho_j^2) gradW straight to the
        // acceleration (unit weight per particle), i.e. -(1/rho) grad P in number-density units.
        // Because m (particleMass) appears explicitly, the same TargetR holds at any particle count.
        //
        // Measured result (fixed-mass sweep, 3k-100k): R comes out 1.7% below TargetR at every N.
        // That shortfall is the discrete Spiky-gradient normalization (~0.83 at the ~6 neighbours
        // inside h for h/spacing = 5/3), which makes the SPH pressure force ~17% weaker than the
        // continuum -(1/rho) grad P -- a discretization error, deliberately NOT folded into K.
        //
        // Recompute Xi1 and DThetaXi1 (scratchpad/lane_emden_2d.py) if polytropicIndex changes.
        // TargetR = 540 reproduces the stiffness of the earlier "x0.1" calibration to within
        // 0.8% in K -- 0.06% in R, since R ~ K^(1/12) -- so the 2026-09-28 fixed-mass runs are
        // valid data for this formula as-is.
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
    MakeOpenTank(), MakePoiseuille(), MakeSelfGravity(), MakePacked(),
};

Scene BuildScene(int presetIndex, int gridCountX, int gridCountY) {
    gridCountX = std::max(gridCountX, 1);
    gridCountY = std::max(gridCountY, 1);
    switch (presetIndex) {
        case 0:  return MakeOpenTank(gridCountX, gridCountY);
        case 1:  return MakePoiseuille(gridCountX, gridCountY);
        case 2:  return MakeSelfGravity(gridCountX, gridCountY);
        case 3:  return MakePacked();
        default: return ScenePresets[presetIndex];
    }
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
