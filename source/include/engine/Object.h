#pragma once
#include "Renderable.h"
#include "util/Vector.h"

struct App;

struct Object {
    Vec2 pos;
    Vec2 vel = Vec2(0, 0);
    Vec2 acc = Vec2(0,0);  // net acceleration — reset every ActiveScene.physics.forceInterval substeps, not every integrate (see Engine.cpp's main loop)
    Renderable renderable;
    float density = 0.f;
    float pressure = 0.f;
    // Set at spawn by a SceneColorRegion: just a marker; color-by modes override it.
    bool  pinnedColor = false;

    float Radius() const;
    // colorOverride, when non-null, is drawn instead of renderable.color — used to recolor
    // particles by a live field (speed/pressure/density) without touching the scene's own color.
    // xOverride, when non-null, is drawn instead of pos.x — used by the camera-follow display
    // offset (Engine.cpp) so the on-screen position can differ from the simulated one.
    void  Draw(App& app, const Color* colorOverride = nullptr, const float* xOverride = nullptr) const;
};

