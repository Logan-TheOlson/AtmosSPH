#pragma once
#include "Renderable.h"
#include "util/Vector.h"

struct App;

struct Object {
    Vec2 pos;
    Vec2 vel = Vec2(0, 0);
    Vec2 acc = Vec2(0,0);  // net acceleration; reset every ActiveScene.physics.forceInterval substeps
    Renderable renderable;
    float density = 0.f;
    float pressure = 0.f;
    // Set at spawn by a SceneColorRegion: just a marker; color-by modes override it.
    bool  pinnedColor = false;

    float Radius() const;
    // colorOverride, when non-null, is drawn instead of renderable.color (live color-by modes).
    // xOverride, when non-null, is drawn instead of pos.x (camera-follow display offset).
    void  Draw(App& app, const Color* colorOverride = nullptr, const float* xOverride = nullptr) const;
};

