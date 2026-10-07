#pragma once
#include "Config.h"
#include "util/Vector.h"
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

// Simulation container half-extents, as a fraction of the simulated area's half-extents.
struct SceneBoundary {
    // The simulated area in pixels: the world every other pixel value in the scene lives in, and
    // the resolution recordings are saved at (kept even, for the encoder). The window only shows
    // it scaled to fit. Change it through SetSceneResolution so the layout scales along.
    int   resWidth   = Config::WindowWidth;
    int   resHeight  = Config::WindowHeight;
    float widthFrac  = 1.0f;
    float heightFrac = 1.0f;
    // Absolute half-extents in simulation pixels. When > 0, each overrides its *Frac above, making
    // the container independent of window size and recording resolution.
    float halfWidthPx  = 0.f;
    float halfHeightPx = 0.f;
    // If set, a particle crossing the left/right wall teleports to the other side (velocity
    // untouched) instead of bouncing. Sph.cpp also mirrors particles across the seam (see
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

// Which pressure-from-density law Sph.cpp's CalculatePressure applies.
enum class EosModel {
    // Becker-Teschner weakly-compressible SPH: pressure = stiffness*targetDensity/exponent *
    // ((density/targetDensity)^exponent - 1), clamped to >=0.
    WCSPH,
    // Pure polytrope: pressure = stiffness * density^(1 + 1/polytropicIndex), no targetDensity offset.
    Polytropic,
};

// Which viscous force Sph.cpp's CalculatePressureForce applies between particle pairs.
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
// for SPH density (Sph.cpp's ForEachObstacleGhost). World pixels, as SceneEvent.
struct SceneObstacle {
    EventShape shape = EventShape::Circle;
    float cx = 0.f, cy = 0.f;
    float radius = 100.f;                // Circle
    float halfW = 100.f, halfH = 100.f;  // Rect
    Color color = { 0.55f, 0.58f, 0.65f, 1.f };
};

// A block of starting particles: a square lattice filling a rect, or a Fibonacci-spiral disk for a
// circle, at spacing 3 * circleRadius * spacingScale. A Scene with `useGroups` spawns exactly its
// groups; otherwise Init() derives one from SceneSpawn (SpawnBlockAsGroup). World pixels.
struct SceneGroup {
    EventShape shape = EventShape::Rect;
    float cx = 0.f, cy = 0.f;
    float radius = 100.f;                // Circle
    float halfW = 100.f, halfH = 100.f;  // Rect
    float spacingScale = 1.f;            // < 1 packs tighter, like SceneSpawn::spacingScale
    bool  fixSize = false;               // Rect: the box is sized to fit cols x rows particles
    int   cols = 10, rows = 10;          // used when fixSize
    bool  customColor = false;           // false: the scene's particle color (particles.circleColor)
    Color color = { 0.2f, 0.6f, 1.0f, 1.0f };
};

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

// The container's half-extents in simulation pixels.
inline float ContainerHalfWidth(const SceneBoundary& b)  { return b.halfWidthPx  > 0.f ? b.halfWidthPx  : 0.5f * static_cast<float>(b.resWidth)  * b.widthFrac; }
inline float ContainerHalfHeight(const SceneBoundary& b) { return b.halfHeightPx > 0.f ? b.halfHeightPx : 0.5f * static_cast<float>(b.resHeight) * b.heightFrac; }

// Rescales the size-dependent SPH constants (smoothing radius, target density, stiffness,
// kinematic viscosity) from p.circleRadius so the fluid behaves like SceneParticles' defaults at
// any particle size: h and the lattice spacing scale together, density as 1/size^2, and stiffness
// (sound speed^2) and viscosity as size^2 so the CFL number at a fixed substep count is unchanged.
void AutoTuneSph(SceneParticles& p);

// Resizes the simulated area (clamped, and made even) and scales everything laid out in it to
// match: groups, obstacles, events, color areas, absolute container sizes and the spawn grid
// (unless it fills the boundary). Particle and kernel sizes are physical and stay.
void SetSceneResolution(Scene& scene, int width, int height);

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
