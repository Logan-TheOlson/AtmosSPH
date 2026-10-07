#include "physics/Gravity.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "physics/Quadtree.h"

namespace Gravity {

namespace {
    // Cached from `params`; recomputed only when SetParams() runs.
    float G       = 20.f;
    // 2D Gauss's law gives g(r) = 2*G*M/r (vs. 3D's G*M/r^2); cached to skip a multiply per pair.
    float TwoG    = 40.f;
    float Mass    = 1.f;
    float Eps2    = 0.f;
    float ThetaSq = 0.f;
}

void SetParams(const GravityParams& p) {
    params  = p;
    G       = p.g;
    TwoG    = 2.f * p.g;
    Mass    = p.particleMass;
    Eps2    = p.softening * p.softening;
    ThetaSq = p.macTheta * p.macTheta;
}

void BuildTree(const std::vector<Vec2>& pos, int32_t n) {
    BuildQuadtree(pos, n);
    ComputeMoments(pos);
}

Vec2 EvaluateForce(int32_t selfIdx, const std::vector<Vec2>& pos) {
    const Vec2 p = pos[selfIdx];
    Vec2 accel(0.f, 0.f);

    if (nodes.empty()) return accel;

    int32_t stack[256];
    int32_t sp = 0;
    stack[sp++] = 0; // root

    while (sp > 0) {
        const QuadNode& node = nodes[stack[--sp]];
        if (node.mass <= 0.f) continue;

        const Vec2  diff = p - node.centerOfMass;
        const float r2   = dot(diff, diff);
        const bool  isLeaf = node.childStart < 0;

        // MAC: treat the node as a single mass if it's a leaf or far enough away ((2*halfSize)/dist < theta).
        if (isLeaf || (4.f * node.halfSize * node.halfSize) < ThetaSq * r2) {
            if (isLeaf) {
                for (int32_t k = 0; k < node.particleCount; k++) {
                    const int32_t idx = particleIndex[node.particleStart + k];
                    if (idx == selfIdx) continue;

                    const Vec2  d   = p - pos[idx];
                    const float rr2 = dot(d, d) + Eps2;
                    const float invR3 = 1.f / (rr2 * std::sqrt(rr2));
                    accel -= (G * Mass) * d * invR3;
                }
            } else {
                const float r2s   = r2 + Eps2;
                const float r     = std::sqrt(r2s);
                const float invR3 = 1.f / (r2s * r);
                accel -= (G * node.mass) * diff * invR3;

                // Quadrupole correction: the 3D multipole term with z dropped (everything is in
                // the z=0 plane; see Quadtree.h for why qyy != -qxx).
                const float invR5 = invR3 / r2s;
                const float invR7 = invR5 / r2s;
                const Vec2  QR(node.qxx * diff.x + node.qxy * diff.y,
                                node.qxy * diff.x + node.qyy * diff.y);
                const float S = diff.x * QR.x + diff.y * QR.y;
                accel += (G * invR5) * QR - (2.5f * G * S * invR7) * diff;
            }
        } else {
            for (int32_t c = 0; c < 4; c++) stack[sp++] = node.childStart + c;
        }
    }

    return accel;
}

// A real 2D universe's Poisson equation (nabla^2 phi = 4*pi*G*Sigma) gives the softened log
// potential phi(r) = G*m*ln(r^2 + eps^2) and force -grad(phi) = -2*G*m*d/(r^2+eps^2): a 1/r
// force, not EvaluateForce()'s 1/r^2. Monopole-only, since QuadNode's quadrupole tensor is
// derived for the 1/r^2 law.
Vec2 EvaluateForce2D(int32_t selfIdx, const std::vector<Vec2>& pos) {
    const Vec2 p = pos[selfIdx];
    Vec2 accel(0.f, 0.f);

    if (nodes.empty()) return accel;

    int32_t stack[256];
    int32_t sp = 0;
    stack[sp++] = 0; // root

    while (sp > 0) {
        const QuadNode& node = nodes[stack[--sp]];
        if (node.mass <= 0.f) continue;

        const Vec2  diff = p - node.centerOfMass;
        const float r2   = dot(diff, diff);
        const bool  isLeaf = node.childStart < 0;

        // Same MAC as EvaluateForce(): acceptance depends only on tree geometry.
        if (isLeaf || (4.f * node.halfSize * node.halfSize) < ThetaSq * r2) {
            if (isLeaf) {
                for (int32_t k = 0; k < node.particleCount; k++) {
                    const int32_t idx = particleIndex[node.particleStart + k];
                    if (idx == selfIdx) continue;

                    const Vec2  d      = p - pos[idx];
                    const float rr2    = dot(d, d) + Eps2;
                    const float invR2  = 1.f / rr2;
                    accel -= (TwoG * Mass) * d * invR2;
                }
            } else {
                const float r2s   = r2 + Eps2;
                const float invR2 = 1.f / r2s;
                accel -= (TwoG * node.mass) * diff * invR2;
            }
        } else {
            for (int32_t c = 0; c < 4; c++) stack[sp++] = node.childStart + c;
        }
    }

    return accel;
}

Vec2 DirectSumGravity(int32_t selfIdx, const std::vector<Vec2>& pos, int32_t n) {
    const Vec2 p = pos[selfIdx];
    Vec2 accel(0.f, 0.f);

    for (int32_t j = 0; j < n; j++) {
        if (j == selfIdx) continue;
        const Vec2  d   = p - pos[j];
        const float rr2 = dot(d, d) + Eps2;
        const float invR3 = 1.f / (rr2 * std::sqrt(rr2));
        accel -= (G * Mass) * d * invR3;
    }

    return accel;
}

Vec2 DirectSumGravity2D(int32_t selfIdx, const std::vector<Vec2>& pos, int32_t n) {
    const Vec2 p = pos[selfIdx];
    Vec2 accel(0.f, 0.f);

    for (int32_t j = 0; j < n; j++) {
        if (j == selfIdx) continue;
        const Vec2  d     = p - pos[j];
        const float rr2   = dot(d, d) + Eps2;
        const float invR2 = 1.f / rr2;
        accel -= (TwoG * Mass) * d * invR2;
    }

    return accel;
}

void ValidateAccuracy(const std::vector<Vec2>& pos, int32_t n, int32_t sampleCount) {
    const int32_t count = std::min(sampleCount, n);
    float maxRelErr = 0.f, sumRelErr = 0.f;

    for (int32_t i = 0; i < count; i++) {
        const Vec2  approx = EvaluateForce(i, pos);
        const Vec2  exact  = DirectSumGravity(i, pos, n);
        const float exactMag = magnitude(exact);
        if (exactMag <= 0.f) continue;

        const float relErr = magnitude(approx - exact) / exactMag;
        maxRelErr = std::max(maxRelErr, relErr);
        sumRelErr += relErr;
    }

    std::printf("[Gravity] validated %d/%d particles: max rel err %.4f, mean rel err %.4f\n",
                count, n, maxRelErr, count > 0 ? sumRelErr / static_cast<float>(count) : 0.f);
}

void ValidateAccuracy2D(const std::vector<Vec2>& pos, int32_t n, int32_t sampleCount) {
    const int32_t count = std::min(sampleCount, n);
    float maxRelErr = 0.f, sumRelErr = 0.f;

    for (int32_t i = 0; i < count; i++) {
        const Vec2  approx = EvaluateForce2D(i, pos);
        const Vec2  exact  = DirectSumGravity2D(i, pos, n);
        const float exactMag = magnitude(exact);
        if (exactMag <= 0.f) continue;

        const float relErr = magnitude(approx - exact) / exactMag;
        maxRelErr = std::max(maxRelErr, relErr);
        sumRelErr += relErr;
    }

    std::printf("[Gravity2D] validated %d/%d particles: max rel err %.4f, mean rel err %.4f\n",
                count, n, maxRelErr, count > 0 ? sumRelErr / static_cast<float>(count) : 0.f);
}

} // namespace Gravity
