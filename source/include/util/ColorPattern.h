#pragma once
#include "engine/Scene.h"
#include <string>

struct ColorImage;

// Evaluates a SceneColorPattern at world positions inside a rectangle (the spawn block's bounds).
// Image patterns are loaded through a one-entry cache keyed by path, so building a sampler every
// frame (the setup preview) doesn't re-read the file.
class PatternSampler {
public:
    PatternSampler(const SceneColorPattern& pattern, float minX, float minY, float maxX, float maxY);

    // False if the pattern leaves (x, y) unpinned (kind None, or outside/transparent in an image).
    bool Sample(float x, float y, Color& out) const;

private:
    const SceneColorPattern& p;
    float minX, minY, maxX, maxY;
    const ColorImage* image = nullptr;
};

// Status of the image the pattern points at, for the UI: "" when not an image pattern.
std::string PatternImageStatus(const SceneColorPattern& pattern);
