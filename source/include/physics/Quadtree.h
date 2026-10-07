#pragma once
#include <cstdint>
#include <vector>

#include "util/Vector.h"

namespace Gravity {

// Flattened quadtree: children of node i occupy [nodes[i].childStart, nodes[i].childStart + 4).
// Children always have a larger index than their parent, so a reverse scan visits children first.
struct QuadNode {
    Vec2    center{};
    float   halfSize = 0.f;
    int32_t parent = -1;

    float   mass = 0.f;
    Vec2    centerOfMass{};
    // Traceless 3D quadrupole tensor components (masses in the z=0 plane), so qyy is not simply
    // -qxx. Used only by EvaluateForce(); EvaluateForce2D() is monopole-only.
    float   qxx = 0.f, qxy = 0.f, qyy = 0.f;

    int32_t childStart    = -1; // -1 = leaf
    int32_t particleStart = 0;
    int32_t particleCount = 0;
};

extern std::vector<QuadNode>  nodes;
extern std::vector<int32_t>   particleIndex; // permutation of [0,n) grouped by leaf

// Builds the tree over `pos[0..n)`; `particleIndex` stores indices into `pos`.
void BuildQuadtree(const std::vector<Vec2>& pos, int32_t n);

// Bottom-up pass computing mass/center-of-mass/quadrupole per node. Call after BuildQuadtree()
// with the same `pos`.
void ComputeMoments(const std::vector<Vec2>& pos);

} // namespace Gravity
