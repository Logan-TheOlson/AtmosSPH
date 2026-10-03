#include "physics/Quadtree.h"

#include <algorithm>
#include <array>

#include "physics/GravityParams.h"
#include "util/ThreadPool.h"

namespace Gravity {

std::vector<QuadNode>  nodes;
std::vector<int32_t>   particleIndex;

namespace {
    // Below this half-size, stop subdividing even if a leaf is over-full
    // (guards against unbounded recursion on coincident/near-coincident
    // particle positions).
    constexpr float MinHalfSize = 0.01f;

    std::vector<int32_t> scratch;

    int32_t Quadrant(const Vec2& p, const Vec2& center) {
        return (p.x >= center.x ? 1 : 0) | (p.y >= center.y ? 2 : 0);
    }
}

void BuildQuadtree(const std::vector<Vec2>& pos, int32_t n) {
    nodes.clear();
    particleIndex.resize(n);
    for (int32_t i = 0; i < n; i++) particleIndex[i] = i;

    if (n == 0) return;

    Vec2 lo = pos[0], hi = pos[0];
    for (int32_t i = 1; i < n; i++) {
        lo.x = std::min(lo.x, pos[i].x); lo.y = std::min(lo.y, pos[i].y);
        hi.x = std::max(hi.x, pos[i].x); hi.y = std::max(hi.y, pos[i].y);
    }
    const Vec2  center   = (lo + hi) * 0.5f;
    const float halfSize = std::max(std::max(0.5f * (hi.x - lo.x), 0.5f * (hi.y - lo.y)), 1.f);

    QuadNode root;
    root.center = center;
    root.halfSize = halfSize;
    root.particleStart = 0;
    root.particleCount = n;
    nodes.push_back(root);

    scratch.resize(n);

    // BFS by level: [levelBegin, levelEnd) is the current level's node range.
    // Children are always appended past levelEnd, so this never touches a
    // node created during the same pass.
    int32_t levelBegin = 0;
    int32_t levelEnd   = static_cast<int32_t>(nodes.size());
    int32_t level      = 0;
    // A hard depth cap: with non-finite positions halfSize is NaN, the MinHalfSize test never
    // fires, and the tree would otherwise grow until memory runs out. Also keeps the traversal's
    // fixed-size stack safe (depth * 3 pending siblings + 1 must stay under it).
    constexpr int32_t MaxLevels = 64;
    while (levelBegin < levelEnd && level++ < MaxLevels) {
        for (int32_t ni = levelBegin; ni < levelEnd; ni++) {
            const QuadNode node = nodes[ni]; // copy: nodes may reallocate below
            if (node.particleCount <= params.maxLeafParticles || node.halfSize <= MinHalfSize)
                continue; // stays a leaf (childStart left at -1)

            std::array<int32_t, 4> counts{ 0, 0, 0, 0 };
            for (int32_t k = 0; k < node.particleCount; k++) {
                const int32_t idx = particleIndex[node.particleStart + k];
                counts[Quadrant(pos[idx], node.center)]++;
            }
            std::array<int32_t, 5> offsets{ 0, 0, 0, 0, 0 };
            for (int32_t q = 0; q < 4; q++) offsets[q + 1] = offsets[q] + counts[q];

            std::array<int32_t, 4> cursor{ offsets[0], offsets[1], offsets[2], offsets[3] };
            for (int32_t k = 0; k < node.particleCount; k++) {
                const int32_t idx = particleIndex[node.particleStart + k];
                const int32_t q   = Quadrant(pos[idx], node.center);
                scratch[node.particleStart + cursor[q]++] = idx;
            }
            for (int32_t k = 0; k < node.particleCount; k++)
                particleIndex[node.particleStart + k] = scratch[node.particleStart + k];

            const int32_t childStart = static_cast<int32_t>(nodes.size());
            nodes[ni].childStart = childStart;
            const float childHalf = node.halfSize * 0.5f;
            for (int32_t q = 0; q < 4; q++) {
                QuadNode child;
                child.halfSize = childHalf;
                child.center = Vec2(
                    node.center.x + ((q & 1) ? childHalf : -childHalf),
                    node.center.y + ((q & 2) ? childHalf : -childHalf));
                child.particleStart = node.particleStart + offsets[q];
                child.particleCount = counts[q];
                child.parent = ni;
                nodes.push_back(child);
            }
        }
        levelBegin = levelEnd;
        levelEnd   = static_cast<int32_t>(nodes.size());
    }
}

void ComputeMoments(const std::vector<Vec2>& pos) {
    const float mass = params.particleMass;
    const int32_t nodeCount = static_cast<int32_t>(nodes.size());

    // Leaves only read `pos`/`particleIndex` and each writes its own node, so they're
    // datarace-free across threads — unlike internal nodes below, they don't depend on any
    // other node having been processed yet, so this pass can run before/independent of that scan.
    ParallelFor(nodeCount, [&](int32_t begin, int32_t end) {
        for (int32_t ni = begin; ni < end; ni++) {
            QuadNode& node = nodes[ni];
            if (node.childStart >= 0) continue;

            Vec2 sum(0.f, 0.f);
            for (int32_t k = 0; k < node.particleCount; k++)
                sum += pos[particleIndex[node.particleStart + k]];

            node.mass = static_cast<float>(node.particleCount) * mass;
            node.centerOfMass = node.particleCount > 0 ? sum / static_cast<float>(node.particleCount) : node.center;

            float qxx = 0.f, qxy = 0.f, qyy = 0.f;
            for (int32_t k = 0; k < node.particleCount; k++) {
                const Vec2 d = pos[particleIndex[node.particleStart + k]] - node.centerOfMass;
                const float r2 = dot(d, d);
                qxx += mass * (3.f * d.x * d.x - r2);
                qxy += mass * (3.f * d.x * d.y);
                qyy += mass * (3.f * d.y * d.y - r2);
            }
            node.qxx = qxx; node.qxy = qxy; node.qyy = qyy;
        }
    });

    // Children always have a larger index than their parent (see BuildQuadtree), so a reverse
    // scan is a valid bottom-up order. Leaves were already filled in above; only internal nodes
    // (whose children — leaf or internal — are guaranteed done by the time we reach them) are
    // computed here, serially, since each depends on the last.
    for (int32_t ni = nodeCount - 1; ni >= 0; ni--) {
        QuadNode& node = nodes[ni];
        if (node.childStart < 0) continue;

        float totalMass = 0.f;
        Vec2  weighted(0.f, 0.f);
        for (int32_t c = 0; c < 4; c++) {
            const QuadNode& child = nodes[node.childStart + c];
            totalMass += child.mass;
            weighted  += child.centerOfMass * child.mass;
        }
        node.mass = totalMass;
        node.centerOfMass = totalMass > 0.f ? weighted / totalMass : node.center;

        float qxx = 0.f, qxy = 0.f, qyy = 0.f;
        for (int32_t c = 0; c < 4; c++) {
            const QuadNode& child = nodes[node.childStart + c];
            if (child.mass <= 0.f) continue;
            const Vec2  d  = child.centerOfMass - node.centerOfMass;
            const float r2 = dot(d, d);
            // Parallel-axis shift of the child's own quadrupole to the parent's COM,
            // plus the point-mass contribution of the child treated as concentrated at its COM.
            qxx += child.qxx + child.mass * (3.f * d.x * d.x - r2);
            qxy += child.qxy + child.mass * (3.f * d.x * d.y);
            qyy += child.qyy + child.mass * (3.f * d.y * d.y - r2);
        }
        node.qxx = qxx; node.qxy = qxy; node.qyy = qyy;
    }
}

} // namespace Gravity
