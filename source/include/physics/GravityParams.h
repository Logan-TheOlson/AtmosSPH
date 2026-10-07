#pragma once

// Runtime gravity tuning, mirroring Sph.cpp's scene-driven SPH constants
// (RecomputeSphConstants): set once per scene load via Gravity::SetParams(),
// shared by Gravity.cpp and Quadtree.cpp so each reads live values instead
// of a compile-time Config constant, without threading five extra
// parameters through every call.
namespace Gravity {

struct GravityParams {
    float g                = 20.f;
    float particleMass     = 1.f;
    float softening        = 7.9f; // Plummer softening length, avoids the 1/r^2 singularity as r -> 0
    float macTheta         = 0.5f; // Barnes-Hut node-acceptance criterion
    int   maxLeafParticles = 8;
};

extern GravityParams params;

} // namespace Gravity
