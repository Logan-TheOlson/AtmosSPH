// Physics substep, frame drawing/submission, batch rendering and the RunEngine() main loop.
#include <imgui_internal.h>
#include <SDL3/SDL.h>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include "util/ColorPattern.h"
#include "engine/Engine.h"
#include "engine/Scene.h"
#include "engine/SceneHelpers.h"
#include "engine/ScenePresetIO.h"
#include "renderer/App.h"
#include "ui/EventGizmo.h"
#include "ui/Theme.h"
#include "ui/Widgets.h"
#include "ui/Workspace.h"
#include "util/DataRecorder.h"
#include "util/Filename.h"
#include "util/Profiler.h"
#include "util/RunNaming.h"
#include "Config.h"
#include <imgui.h>
#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <string>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include "engine/EngineInternal.h"

Object& CreateObject(float x, float y, Renderable r) {
    objects.push_back({ .pos = Vec2(x, y), .renderable = r });
    return objects.back();
}

// The simulation container's half-extents: the scene's boundary fractions applied to its
// simulated area (SceneBoundary::resWidth/resHeight), which is independent of the window.
// SceneBoundary::halfWidthPx/halfHeightPx override both.
float ScreenHalfWidth() {
    const auto& b = ActiveScene.boundary;
    if (b.halfWidthPx > 0.f) return b.halfWidthPx;
    return frozenHalfW > 0.f ? frozenHalfW : ContainerHalfWidth(b);
}
float ScreenHalfHeight() {
    const auto& b = ActiveScene.boundary;
    if (b.halfHeightPx > 0.f) return b.halfHeightPx;
    return frozenHalfH > 0.f ? frozenHalfH : ContainerHalfHeight(b);
}

// Draws a black frame at the scene's boundary so a container narrower than the window reads as a
// wall. AddRectOutline draws its border inward, so the rect is inflated by BorderThickness to put
// the band entirely outside the wall.
static void DrawBoundary(App& app) {
    constexpr float BorderThickness = 6.0f;
    constexpr Color BorderColor     = { 0.f, 0.f, 0.f, 1.f };
    const float hw = ScreenHalfWidth(), hh = ScreenHalfHeight();

    if (ActiveScene.boundary.periodicX) {
        // No side walls when X wraps: just solid top/bottom bars. A border thicker than the bar
        // fills it solid (see AddRectOutline).
        constexpr float FillSolid = 1e6f;
        const float barHalfHeight = BorderThickness * 0.5f;
        app.AddRectOutline(0.f,  hh + barHalfHeight, hw + BorderThickness, barHalfHeight, FillSolid, BorderColor);
        app.AddRectOutline(0.f, -hh - barHalfHeight, hw + BorderThickness, barHalfHeight, FillSolid, BorderColor);
    } else {
        app.AddRectOutline(0.f, 0.f, hw + BorderThickness, hh + BorderThickness, BorderThickness, BorderColor);
    }
}

// Draws the scene's obstacles through the renderer (not ImGui), so they show in recordings. A rect
// is an outline thick enough to fill it solid (there is no filled-rect call).
static float WrapX(float x, float hw); // defined below
static void DrawObstacles(App& app, bool followCam, float hw) {
    constexpr size_t MaxObstacleRects = 56;
    size_t rects = 0;
    for (const SceneObstacle& o : ActiveScene.obstacles) {
        const float x = followCam ? WrapX(o.cx - cameraOffsetX, hw) : o.cx;
        if (o.shape == EventShape::Circle) {
            app.AddCircle(x, o.cy, o.radius, o.color);
        } else if (rects++ < MaxObstacleRects) {
            app.AddRectOutline(x, o.cy, o.halfW, o.halfH, std::max(o.halfW, o.halfH), o.color);
        }
    }
}

// vt is the velocity component tangential to this axis's wall (e.g. vel.y for the x-axis walls);
// with noSlipWalls it is damped on contact.
static void resolveAxis(float& p, float& v, float& vt, float half, float r) {
    const auto& physics = ActiveScene.physics;
    bool hit = false;
    if (p - r < -half) { p = -half + r; if (v < 0.0f) { v *= -physics.restitution; hit = true; } }
    if (p + r >  half) { p =  half - r; if (v > 0.0f) { v *= -physics.restitution; hit = true; } }
    if (hit && physics.noSlipWalls) vt *= 1.0f - physics.friction;
}

// Pushes a particle of radius r out of every overlapping obstacle and, if it's moving into the
// surface, reflects the normal velocity and (with no-slip walls) drags the tangential one, as
// resolveAxis does for the container walls.
static void ResolveObstacles(Object& obj, float r) {
    const auto& physics = ActiveScene.physics;
    for (const SceneObstacle& o : ActiveScene.obstacles) {
        float nx, ny;
        const float sd = ObstacleSurface(o, obj.pos.x, obj.pos.y, nx, ny);
        if (sd >= r) continue;

        obj.pos.x += nx * (r - sd);
        obj.pos.y += ny * (r - sd);

        const float vn = obj.vel.x * nx + obj.vel.y * ny;
        if (vn >= 0.f) continue; // already moving away
        obj.vel.x -= (1.f + physics.restitution) * vn * nx;
        obj.vel.y -= (1.f + physics.restitution) * vn * ny;
        if (physics.noSlipWalls) {
            const float vnNew = obj.vel.x * nx + obj.vel.y * ny;
            obj.vel.x -= physics.friction * (obj.vel.x - vnNew * nx);
            obj.vel.y -= physics.friction * (obj.vel.y - vnNew * ny);
        }
    }
}

// Wraps x into (-hw, hw], the range periodicX keeps positions in. Unlike the single-wrap teleport,
// it handles any accumulated camera-follow offset.
static float WrapX(float x, float hw) {
    const float span = 2.f * hw;
    if (span <= 0.f) return x;
    x = std::fmod(x + hw, span);
    if (x < 0.f) x += span;
    return x - hw;
}

// Integrates one substep, reusing the force (particle.acc) from the last Update(), held across
// ActiveScene.physics.forceInterval substeps to trade staleness for compute. acc is reset only
// when about to be recomputed, not here.
//
// Returns false if any particle came out non-finite or absurdly fast (the sim has diverged); every
// particle is still integrated and the caller decides what to do (see AdvanceSimulationLocked).
// Not compatible with -ffast-math, which may optimize the isfinite test away.
bool Integrate(float subDt) {
    const float hw = ScreenHalfWidth(), hh = ScreenHalfHeight();
    const bool  periodicX = ActiveScene.boundary.periodicX;
    constexpr float MaxSpeedSq = 1.0e12f; // 1e6 px/s, far beyond any stable scene
    bool   finite = true;
    const bool hasObstacles = !ActiveScene.obstacles.empty();
    double sumVx = 0.0; // mean flow velocity this substep, for the camera-follow display offset
    for (auto& obj : objects) {
        const float r = obj.Radius();
        obj.vel += obj.acc * subDt;
        obj.pos += obj.vel * subDt;
        if (periodicX) {
            // Teleport across the seam rather than bounce; velocity is untouched.
            if      (obj.pos.x >  hw) obj.pos.x -= 2.f * hw;
            else if (obj.pos.x < -hw) obj.pos.x += 2.f * hw;
            sumVx += obj.vel.x;
        } else {
            resolveAxis(obj.pos.x, obj.vel.x, obj.vel.y, hw, r);
        }
        resolveAxis(obj.pos.y, obj.vel.y, obj.vel.x, hh, r);
        if (hasObstacles) {
            // Obstacle, walls, obstacle: a particle pinned between an obstacle and a wall ends up legal against both.
            ResolveObstacles(obj, r);
            if (!periodicX) resolveAxis(obj.pos.x, obj.vel.x, obj.vel.y, hw, r);
            resolveAxis(obj.pos.y, obj.vel.y, obj.vel.x, hh, r);
            ResolveObstacles(obj, r);
        }

        // NaN fails both tests (the sum is NaN, and NaN < MaxSpeedSq is false); so does inf.
        if (!std::isfinite(obj.pos.x + obj.pos.y + obj.vel.x + obj.vel.y) ||
            !(obj.vel.x * obj.vel.x + obj.vel.y * obj.vel.y < MaxSpeedSq))
            finite = false;
    }
    if (!finite) return false;
    if (periodicX && cameraFollow && !objects.empty()) {
        cameraOffsetX += static_cast<float>(sumVx / objects.size()) * subDt;
        cameraOffsetX  = WrapX(cameraOffsetX, hw);
    }
    return true;
}

// Draws the current particles/boundary, submits the frame (capturing it if recording), then
// records the CSV data sample. One call is one simulated+rendered frame at whatever dt physics
// just advanced by. Running calls it once per real frame; AdvanceRendering calls it back-to-back
// at a fixed 1/fps dt.
static void DrawAndSubmitFrame(float dt, bool offscreen) {
    // Particles are read under simMutex. If the worker is wedged the frame is still presented,
    // without particles, so the UI can say so. The lock is dropped before RenderFrame blocks.
    SimLock lk;
    if (!LockSim(lk, offscreen ? 5000 : 100)) {
        app.RenderFrame(dt);
        return;
    }

    const bool coloring = colorMode != ColorMode::Off && !objects.empty();
    float colorLo = 0.f, colorHi = 0.f;
    if (coloring) colorLo = colorHi = FieldValue(objects[0], colorMode);
    for (auto& obj : objects) {
        if (coloring) {
            const float v = FieldValue(obj, colorMode);
            colorLo = std::min(colorLo, v);
            colorHi = std::max(colorHi, v);
        }
    }

    // Display-only x shift (see Integrate's cameraOffsetX); never touches obj.pos.
    const bool  followCam = ActiveScene.boundary.periodicX && cameraFollow;
    const float hw        = ScreenHalfWidth();
    if (coloring) {
        // Auto-scaled to the current frame's range, since field magnitudes vary wildly across scenes and over a run.
        const ColorRamp& ramp = colorRamps[static_cast<int>(colorMode)];
        const float range = colorHi - colorLo;
        for (auto& obj : objects) {
            float t = range > 0.f ? (FieldValue(obj, colorMode) - colorLo) / range : 0.f;
            Color c = LerpColor(ramp.low, ramp.high, t);
            if (followCam) {
                float dispX = WrapX(obj.pos.x - cameraOffsetX, hw);
                obj.Draw(app, &c, &dispX);
            } else {
                obj.Draw(app, &c);
            }
        }
    } else {
        for (auto& obj : objects) {
            if (followCam) {
                float dispX = WrapX(obj.pos.x - cameraOffsetX, hw);
                obj.Draw(app, nullptr, &dispX);
            } else {
                obj.Draw(app);
            }
        }
    }
    DrawBoundary(app);
    DrawObstacles(app, followCam, hw);

    // Batch rendering draws and captures offscreen every frame, with no swapchain, so vsync can't
    // throttle it. RenderFrame draws the ImGui callback (which takes simMutex itself) and blocks on
    // vsync/the GPU, so release the lock meanwhile.
    lk.unlock();
    if (offscreen) app.RenderOffscreenFrame();
    else           app.RenderFrame(dt);
    if (!LockSim(lk, offscreen ? 5000 : 100)) return;

    if (dataRecorder.IsActive()) {
        dataAccum += dt;
        const float interval = 1.f / static_cast<float>(dataRateActive);
        // Sampled on its own accumulator, independent of the render/physics rate.
        if (dataAccum >= interval) {
            dataAccum -= interval;

            std::vector<ParticleSample> samples;
            samples.reserve(objects.size());
            for (auto& obj : objects)
                samples.push_back({ obj.pos.x, obj.pos.y, obj.vel.x, obj.vel.y,
                                     magnitude(obj.vel), obj.density, obj.pressure });

            dataRecorder.SubmitFrame(static_cast<int>(dataSavedFrames), dataElapsed, std::move(samples));
            dataSavedFrames++;
            if (dataLengthActive > 0.f && dataSavedFrames >= static_cast<uint32_t>(dataLengthActive * dataRateActive))
                dataRecorder.Stop();
        }
        dataElapsed += dt;
    }
}

// Runs physics and rendering/capture back-to-back at a fixed 1/fps dt instead of real-time pacing,
// drawing and capturing offscreen (see DrawAndSubmitFrame/RenderOffscreenFrame). Relies on the
// recorder's length cap to know when it's done; once app.IsRecording() goes false there is no
// playback, so it returns to Menu. Budgeted to a wall-clock slice per call so PollEvents() keeps
// the window responsive; the caller refreshes the progress overlay once per real tick.
// True while the active run still has an output driving the batch loop. A data-only entry never
// sets app.IsRecording(), so both are checked.
static bool BatchRunActive() { return app.IsRecording() || dataRecorder.IsActive() || profileLogger.IsActive(); }

static void AdvanceRendering() {
    using Clock = std::chrono::steady_clock;
    const auto  budgetStart            = Clock::now();
    constexpr float FrameBudgetSeconds = 1.f / 30.f;
    const float frameDt = 1.f / static_cast<float>(std::max(activeRun.fps, 1));

    while (BatchRunActive() && !simFaulted) {
        AdvanceSimulation(frameDt);
        if (simFaulted) break; // don't capture the diverged frame
        DrawAndSubmitFrame(frameDt, /*offscreen=*/true);
        if (std::chrono::duration<float>(Clock::now() - budgetStart).count() >= FrameBudgetSeconds)
            break;
    }

    // A diverged batch run can't progress: finish its outputs (keeping what was captured) and fall
    // through to the normal "run finished" handling. The fault stays flagged for the top bar.
    if (simFaulted) {
        app.StopRecording();
        dataRecorder.Stop();
        profileLogger.Stop();
    }

    // These stops are no-ops if an output already stopped itself at its length cap.
    if (!BatchRunActive()) {
        if (queueRunningIndex >= 0) AdvanceQueue();
        else                        ReturnToMenu();
    }
}

int RunEngine() {
    // Assigned here, not via a default member initializer: menuConfig and ScenePresets are globals
    // in different translation units with unspecified init order.
    menuConfig.scene = ScenePresets[0];
    RefreshUserPresets();

    app.Init(Config::WindowTitle, Config::WindowWidth, Config::WindowHeight);
    app.SetUICallback([]() {
        SimLock lk;
        if (LockSim(lk)) DrawUi();
        else             DrawUnresponsiveUi();
    });

    objects.reserve(100000);
    StartSimThread();

    using Clock = std::chrono::steady_clock;
    auto last = Clock::now();

    profiler.MarkFrameStart();
    while (app.PollEvents()) {
        // Drained every frame regardless of appState so a stale press can't leak into the next run.
        const bool stepPressed    = app.TakeStepForward();
        const bool bigStepPressed = app.TakeStepForwardBig();
        const bool spacePressed   = app.TakeSpaceToggle();

        auto  now = Clock::now();
        float dt  = std::min(std::chrono::duration<float>(now - last).count(), 1.0f / 30.0f);
        last      = now;

        if (appState == AppState::Running) {
            if (spacePressed)   TogglePause();
            if (stepPressed)    StepOnce();
            if (bigStepPressed) StepBig();
            // The worker does the stepping; this tells it how much time has passed.
            if (!paused && !simFaulted) QueueSimTime(dt);
            profiler.MarkComputeEnd();
            lockWaitMs = 0.f;
            const auto drawStart = Clock::now();
            DrawAndSubmitFrame(dt, /*offscreen=*/false);
            profiler.MarkRenderEnd();
            // The frame's CPU render work: DrawAndSubmitFrame minus GPU/vsync and sim-lock waits.
            const float drawMs = std::chrono::duration<float, std::milli>(Clock::now() - drawStart).count();
            const float workMs = std::max(drawMs - app.LastBlockedMs() - lockWaitMs, 0.f);
            renderCpuMs = renderCpuMs * 0.9f + workMs * 0.1f;
        } else if (appState == AppState::Rendering) {
            // Same key Running uses for pause/resume; equivalent to the Cancel button.
            if (spacePressed) {
                ReturnToMenu();
            } else {
                profiler.MarkComputeEnd();
                AdvanceRendering();
                profiler.MarkRenderEnd();
            }
            // Refreshes the progress overlay (or the menu if AdvanceRendering just finished): one
            // swapchain present per real tick, separate from the offscreen capture loop above.
            app.RenderFrame(dt);
        } else {
            AddMenuPreviewParticles(); // Menu: a physics-free preview of the scene's particles
            app.RenderFrame(dt);
            profiler.MarkComputeEnd();
            profiler.MarkRenderEnd();
        }

        if (profiler.Tick()) {
            if (appState == AppState::Running) {
                SimLock lk;
                if (LockSim(lk, 20)) WriteVelocityProfileCsv();
            }
        }

        profiler.MarkFrameStart();
    }
    // The worker may be stuck in a substep and unjoinable; static destructors must not run under it.
    const bool workerStopped = StopSimThread();
    app.Shutdown();
    if (!workerStopped) std::_Exit(0);
    return 0;
}

