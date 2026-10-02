#pragma once
#include "Config.h"
#include "util/Vector.h"
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

// Simulation container half-extents, as a fraction of the window's half-extents (1.0 = walls at
// the window edge). Kept relative rather than absolute so presets don't hardcode window pixels.
struct SceneBoundary {
    float widthFrac  = 1.0f;
    float heightFrac = 1.0f;
    // Absolute half-extents in simulation pixels. When > 0, each overrides its *Frac above, so
    // the container no longer depends on the window size or on whether a recording (with its own
    // fixed resolution) is active. Scenes whose geometry is part of the physics — e.g. the
    // Poiseuille channel height H, which sets the analytic profile — size the box from their own
    // particle grid this way instead of from a window fraction.
    float halfWidthPx  = 0.f;
    float halfHeightPx = 0.f;
    // If set, the left/right walls don't bounce particles — a particle crossing one teleports to
    // the other side instead (position only; velocity untouched), simulating an infinite domain
    // for a channel-flow test. main.cpp mirrors real particles across the seam too (see
    // ForEachPeriodicX) so density/pressure near the seam are computed as if particles from the
    // opposite edge really are nearby, matching what an infinite domain would give.
    bool  periodicX  = false;
};

struct ScenePhysics {
    // Constant acceleration applied to every particle every frame (pixels/s^2) — gravity is just
    // the common case of this pointing down; a horizontal force drives channel-flow tests instead.
    Vec2  force          = Vec2(0.f, -600.f);
    float restitution   = 0.85f;
    // Fraction of tangential velocity removed on wall contact when noSlipWalls is enabled.
    float friction       = 0.3f;
    bool  noSlipWalls    = true;
    int   substeps       = 12;
    int   forceInterval  = 1;
};

// BarnesHut implements the 3D Newtonian (1/r^2) law restricted to the simulation plane (see
// Quadtree.h's QuadNode comment); Genuine2D instead implements the softened log-potential (1/r)
// law a real 2D universe's Poisson equation gives (see Gravity.cpp's EvaluateForce2D) —
// physically correct for a simulation that's actually 2D, and the default for that reason.
enum class GravityMethod { BarnesHut, Genuine2D };

// Self-gravity between particles, layered on top of ScenePhysics::force. Off by default — most
// scenes (channel flow, open tank) want only the constant external force, not an N-body pull.
// method/parameters are per-scene so, e.g., a stellar-formation preset can run at its own tuned
// G/softening while another scene keeps gravity off entirely.
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

// Which pressure-from-density law main.cpp's CalculatePressure applies. Per-scene, since a scene
// meant to match a known analytical equilibrium (e.g. Self-Gravity vs. Lane-Emden) wants the pure
// polytrope, while the fluid scenes want WCSPH's stiffness against a specific rest density.
enum class EosModel {
    // Becker-Teschner weakly-compressible SPH: pressure = stiffness*targetDensity/exponent *
    // ((density/targetDensity)^exponent - 1), clamped to >=0. Only approximates a pure polytrope
    // once density >> targetDensity; the -1 offset matters close to targetDensity.
    WCSPH,
    // Pure polytrope: pressure = stiffness * density^(1 + 1/polytropicIndex), no targetDensity
    // offset — the equation of state a Lane-Emden equilibrium is derived against.
    Polytropic,
};

// Which viscous force main.cpp's CalculatePressureForce applies between particle pairs.
enum class ViscosityModel {
    // Monaghan artificial viscosity: a pressure-like term along the pair axis, active only for
    // approaching pairs, strength set by `viscosity` (alpha) and `viscosityQuadratic` (beta).
    // Its effective kinematic viscosity (~alpha*h*c/8 in 2D) is not an input — it has to be
    // measured, and it scales with h.
    Monaghan,
    // Morris, Fox & Zhu (1997) physical viscosity: a discretised nu * laplacian(v) acting on the
    // full relative velocity of every pair, with nu = `kinematicViscosity` set directly.
    Morris,
};

struct SceneParticles {
    float circleRadius = 3.0f;
    Color circleColor  = { 0.2f, 0.6f, 1.0f, 1.0f };

    // Calibrated to smoothingRadius=15 and a grid spacing of circleRadius*3=9px — recompute
    // (lattice-sum the Poly6 kernel at the new spacing/radius) if either changes.
    float smoothingRadius    = 15.0f;
    // 6.48e-4 was the lattice sum against the OLD 3D Poly6 normalization (315/(64*pi*h^9)).
    // DensityKernel's shape, (h^2-r^2)^3, didn't change — only main.cpp's DensityNorm prefactor
    // did, to the true 2D normalization (4/(pi*h^8)) — so this is corrected by the exact ratio
    // newNorm/oldNorm = 256*h/315 at this struct's own h=15 (256*15/315 = 12.190476...), not
    // re-derived from scratch.
    float targetDensity      = 6.48e-4f * (256.f * 15.f / 315.f); // = 7.8994...e-3
    float stiffness          = 12000.f;

    EosModel eos             = EosModel::WCSPH;
    int      exponent        = 7;         // WCSPH only
    float    polytropicIndex = 1.f / 6.f; // Polytropic only; gamma = 1 + 1/polytropicIndex

    ViscosityModel viscosityModel = ViscosityModel::Monaghan;

    float viscosity          = 0.5f;  // Monaghan only (alpha)
    // Monaghan artificial viscosity quadratic-term coefficient (beta) — see main.cpp's
    // CalculatePressureForce for why this must stay well under 1. Monaghan only.
    float viscosityQuadratic = 0.05f;
    // Kinematic viscosity nu in px^2/s. Morris only.
    float kinematicViscosity = 800.f;
};

// How Init() lays out the starting particles: a gridCountX x gridCountY block (or, if `circular`,
// a circle inscribed in a block of that size) of spacing-separated particles, centered vertically
// and horizontally offset by offsetXFrac * the boundary's half-width.
struct SceneSpawn {
    int   gridCountX  = 100;
    int   gridCountY  = 100;
    float offsetXFrac = 0.0f;
    bool  circular     = false;
    // If set, Init() ignores gridCountX/Y and packs the lattice wall to wall into the boundary
    // (whatever its size resolves to), writing the resulting counts back into gridCountX/Y.
    bool  fillBoundary = false;
    // Lattice spacing as a multiple of the default (3 * circleRadius). < 1 packs more particles into
    // the same area than the rest density calls for, i.e. starts the fluid compressed (positive
    // pressure); the particle count rises by 1 / spacingScale^2.
    float spacingScale = 1.f;
};

// A sudden acceleration applied to every particle inside an area. Circle/Rect pick the area's
// shape; Continuous applies `accel` (px/s^2) every substep while the event is active, Once adds
// `accel` (px/s) to the velocity of everything inside, a single time, when startTime is reached.
enum class EventShape { Circle, Rect };
// Field: a continuous position-dependent acceleration (see EventFieldAccel) instead of one uniform push.
enum class EventMode  { Continuous, Once, Field };
// Field shapes: Curl swirls counter-clockwise around the area's center (negative strength = clockwise),
// Attract pulls toward it, Repel pushes away.
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

// Field acceleration at world point (px, py) for a Field-mode event; false if the point is outside
// the area. Zero exactly at the center (direction undefined there).
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

// A persistent-color patch: every particle whose *spawn* position lies inside the area keeps `color`
// for the whole run unless a live color-by mode (Engine.cpp) overrides it. Coordinates
// are world pixels (origin = window center, +y up), same as SceneEvent. Later regions win overlaps.
struct SceneColorRegion {
    EventShape shape = EventShape::Rect;
    float cx = 0.f, cy = 0.f;
    float radius = 100.f;                // Circle
    float halfW = 100.f, halfH = 100.f;  // Rect
    Color color = { 1.f, 0.3f, 0.2f, 1.f };
};

// A persistent-color pattern laid over the whole spawn block (before SceneColorRegions, which win where
// they overlap). Like regions, colors are assigned at spawn and never change during the run.
enum class ColorPatternKind { None, VStripes, HStripes, Checker, Rings, Gradient, Image };

struct SceneColorPattern {
    ColorPatternKind kind = ColorPatternKind::None;
    Color colorA = { 1.f, 0.35f, 0.25f, 1.f };
    Color colorB = { 0.25f, 0.55f, 1.f, 1.f };
    float cellSize = 60.f;       // px: stripe width / checker cell / ring spacing (unused by Gradient/Image)
    std::string imagePath;       // Image: PNG/BMP, scaled to fit the spawn block (aspect kept); transparent pixels stay unpinned
};

struct Scene {
    // Owned (not a pointer into a string literal) so a preset loaded from disk at runtime
    // (ScenePresetIO) can populate these safely — only the 3 compiled presets get away with a
    // literal today, but a loaded Scene needs storage of its own.
    std::string     name;
    std::string     description;
    SceneBoundary   boundary;
    ScenePhysics    physics;
    SceneGravity    gravity;
    SceneParticles  particles;
    SceneSpawn      spawn;

    // Real-Time Limit: hand-tuned grid size this scene stays smooth/stable at in real time on
    // the developer's own machine — not measured by the sim itself, just shown for reference in
    // the menu (Engine.cpp's DrawMenu). Independent of spawn.gridCountX/Y (the scene's own
    // default count), since BuildScene() only overrides the latter.
    int             realTimeLimitX = 0;
    int             realTimeLimitY = 0;

    std::vector<SceneEvent> events;
    std::vector<SceneColorRegion> colorRegions;
    SceneColorPattern colorPattern;
};

extern const std::vector<Scene> ScenePresets;

// Rebuilds ScenePresets[presetIndex] with its particle grid resized to gridCountX x gridCountY
// instead of the preset's own default count — re-deriving whatever calibration depends on that
// count (e.g. MakeSelfGravity's mass/softening/stiffness scaling), not just overwriting
// SceneSpawn's fields on a copy, which would leave a resolution-dependent scene tuned for the
// wrong count. gridCountX/Y are clamped to >= 1.
Scene BuildScene(int presetIndex, int gridCountX, int gridCountY);

// The scene currently governing physics/spawn/boundary. Mutable — set via LoadScene(), not
// assigned directly, so derived SPH kernel constants (main.cpp's RecomputeSphConstants) stay in
// sync with it.
extern Scene ActiveScene;

// Recomputes cached kernel constants (main.cpp) from `scene`'s particle settings. Does not spawn
// particles — call Init() after, once `objects` has been reset.
void LoadScene(const Scene& scene);
