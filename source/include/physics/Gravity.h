#pragma once
#include <cstdint>
#include <vector>

#include "physics/GravityParams.h"
#include "util/Vector.h"

namespace Gravity {

// Applies `p` (G, particle mass, softening, MAC theta, leaf size) to every gravity path below.
// Call whenever a scene loads, before BuildTree.
void SetParams(const GravityParams& p);

// Rebuilds the quadtree over pos[0..n) with its mass/COM/quadrupole moments. Call once per force pass.
void BuildTree(const std::vector<Vec2>& pos, int32_t n);

// Barnes-Hut force (monopole + quadrupole, softened) on particle `selfIdx` for the 3D Newtonian
// (1/r^2) law restricted to the plane. Requires BuildTree() on the same `pos`.
Vec2 EvaluateForce(int32_t selfIdx, const std::vector<Vec2>& pos);

// Barnes-Hut force for the genuine 2D (log-potential, 1/r) law. Monopole-only, since QuadNode's
// quadrupole is only valid for EvaluateForce()'s law. Requires BuildTree() on the same `pos`.
Vec2 EvaluateForce2D(int32_t selfIdx, const std::vector<Vec2>& pos);

// O(n) direct pairwise summation, for validating EvaluateForce().
Vec2 DirectSumGravity(int32_t selfIdx, const std::vector<Vec2>& pos, int32_t n);

// O(n) direct pairwise summation of the genuine 2D law, for validating EvaluateForce2D().
Vec2 DirectSumGravity2D(int32_t selfIdx, const std::vector<Vec2>& pos, int32_t n);

// Compares EvaluateForce() against DirectSumGravity() over the first `sampleCount` particles and
// logs the max/mean relative error to stdout.
void ValidateAccuracy(const std::vector<Vec2>& pos, int32_t n, int32_t sampleCount);

// Same as ValidateAccuracy, for EvaluateForce2D() against DirectSumGravity2D().
void ValidateAccuracy2D(const std::vector<Vec2>& pos, int32_t n, int32_t sampleCount);

} // namespace Gravity
