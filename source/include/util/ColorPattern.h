#pragma once
#include "engine/Scene.h"
#include <string>

struct ColorImage;

// Evaluates a SceneColorPattern at world positions inside a rectangle (the spawn block's bounds).
// Images go through a one-entry cache keyed by path, so per-frame samplers don't re-read the file.
class PatternSampler {
public:
    // blendStrength > 0 softens the hard A/B transitions of stripes, checker and rings into gradients,
    // reaching `blendRadius` px either side of each boundary (<= 0 = half a cell, i.e. a full ramp).
    PatternSampler(const SceneColorPattern& pattern, float minX, float minY, float maxX, float maxY,
                   float blendStrength = 0.f, float blendRadius = 0.f);

    // False if the pattern leaves (x, y) unpinned (kind None, or outside/transparent in an image).
    bool Sample(float x, float y, Color& out) const;

private:
    const SceneColorPattern& p;
    float minX, minY, maxX, maxY;
    float blendStrength, blendRadius;
    const ColorImage* image = nullptr;
};

// Status of the image the pattern points at, for the UI: "" when not an image pattern.
std::string PatternImageStatus(const SceneColorPattern& pattern);
