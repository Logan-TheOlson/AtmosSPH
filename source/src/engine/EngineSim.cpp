// Shared application state, the simulation worker thread, and run control (start, queue, reset, stop).
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

// User-defined entry points
void Init();
void Update(float dt);

std::vector<Object> objects;

// Directory of the running executable, so output files land next to it regardless of the working directory.
std::filesystem::path ExecutableDir() {
#ifdef _WIN32
    char path[MAX_PATH];
    DWORD len = GetModuleFileNameA(nullptr, path, MAX_PATH);
    if (len > 0 && len < MAX_PATH)
        return std::filesystem::path(path, path + len).parent_path();
#endif
    return std::filesystem::current_path();
}

// Bins particles by y-position across the channel height and writes each bin's average x-velocity
// to a CSV next to the executable. Overwritten on every call, so it always holds a recent snapshot.
void WriteVelocityProfileCsv() {
    constexpr int kBins = 40;
    const float hh = ScreenHalfHeight();
    if (hh <= 0.f || objects.empty()) return;

    std::array<double, kBins> sumVx{};
    std::array<int, kBins>    count{};
    for (auto& obj : objects) {
        const float t   = (obj.pos.y + hh) / (2.f * hh); // 0..1 across the channel
        const int   bin = std::clamp(static_cast<int>(t * kBins), 0, kBins - 1);
        sumVx[bin] += obj.vel.x;
        count[bin]++;
    }

    const auto outPath = ExecutableDir() / "velocity_profile.csv";
    std::ofstream out(outPath);
    if (!out) return;

    out << "y,avg_vx,count\n";
    const float binHeight = 2.f * hh / kBins;
    for (int b = 0; b < kBins; b++) {
        if (count[b] == 0) continue;
        const float yCenter = -hh + (b + 0.5f) * binHeight;
        out << yCenter << ',' << (sumVx[b] / count[b]) << ',' << count[b] << '\n';
    }
}


App      app;
Profiler profiler; // frame/render timing, shown in the live panel's Run card

AppState appState = AppState::Menu;

// Freezes physics (rendering, recording and data-saving keep running). Toggled by Space or the
// Pause/Resume button; only Restart/Menu/StartSimulation reset it.
bool paused = false;

// Periodic-channel scenes drift sideways as the force advects the cloud, burying the profile
// under bulk translation. When enabled, the drawn (not simulated) x position is offset by the
// accumulated mean flow velocity each substep (see Integrate), so only relative motion shows.
bool  cameraFollow  = true;
float cameraOffsetX = 0.f;

MenuConfig menuConfig;

// Presets saved via the menu, scanned at startup and after each save. The scene combo lists the
// built-ins at [0, ScenePresets.size()), then these.
std::vector<ScenePresetEntry> userPresets;

void RefreshUserPresets() {
    userPresets = ScanUserPresets(ExecutableDir() / "presets");
}

int CombinedCount() {
    return static_cast<int>(ScenePresets.size() + userPresets.size());
}
bool CombinedIsBuiltin(int i) {
    return i < static_cast<int>(ScenePresets.size());
}
const std::string& CombinedName(int i) {
    return CombinedIsBuiltin(i) ? ScenePresets[i].name
                                 : userPresets[i - static_cast<int>(ScenePresets.size())].name;
}
const std::string& CombinedDescription(int i) {
    return CombinedIsBuiltin(i) ? ScenePresets[i].description
                                 : userPresets[i - static_cast<int>(ScenePresets.size())].description;
}

// Owned here (unlike Recorder) because the data it samples, `objects`, lives here.
DataRecorder dataRecorder;
float        dataAccum{ 0.f };        // seconds since the last sample, gated at the active run's dataRate
float        dataElapsed{ 0.f };       // sim seconds since this data save started (CSV time column)
uint32_t     dataSavedFrames{ 0 };
int          dataRateActive{ 0 };
float        dataLengthActive{ 0.f };  // 0 = unlimited
ProfileLogger profileLogger;

// Simulated seconds since the last ResetSimulation(); advanced only by AdvanceSimulation, so it stops while paused.
double simTime = 0.0;


const char* ColorModeNames[] = { "Off (scene color)", "Speed", "Pressure", "Density" };

ColorMode colorMode = ColorMode::Off;
// Indexed by ColorMode; entry 0 (Off) is unused.
ColorRamp colorRamps[] = {
    {},
    { {0.15f, 0.25f, 0.95f, 1.f}, {0.95f, 0.15f, 0.10f, 1.f} }, // Speed:    blue   -> red
    { {0.05f, 0.55f, 0.15f, 1.f}, {1.00f, 0.90f, 0.10f, 1.f} }, // Pressure: green  -> yellow
    { {0.55f, 0.05f, 0.65f, 1.f}, {1.00f, 0.55f, 0.05f, 1.f} }, // Density:  purple -> orange
};

float FieldValue(const Object& o, ColorMode mode) {
    switch (mode) {
        case ColorMode::Speed:    return magnitude(o.vel);
        case ColorMode::Pressure: return o.pressure;
        case ColorMode::Density:  return o.density;
        default:                  return 0.f;
    }
}

Color LerpColor(const Color& a, const Color& b, float t) {
    return { a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t, 1.f };
}

// The mode + gradient picker used by DrawAndSubmitFrame, shared by the live panel and the setup

// Whether each Once event has fired this run. Parallel to ActiveScene.events; reset by ResetSimulation.
std::vector<char> eventFired;

bool InsideEvent(const SceneEvent& e, const Vec2& p) {
    const float dx = p.x - e.cx, dy = p.y - e.cy;
    if (e.shape == EventShape::Circle) return dx * dx + dy * dy <= e.radius * e.radius;
    return std::fabs(dx) <= e.halfW && std::fabs(dy) <= e.halfH;
}

// Adds each due Once event's velocity delta to every particle inside its area, once per run.
void ApplyOnceEvents() {
    const auto& events = ActiveScene.events;
    eventFired.resize(events.size(), 0);
    for (size_t i = 0; i < events.size(); i++) {
        const SceneEvent& e = events[i];
        if (e.mode != EventMode::Once || eventFired[i] || simTime < e.startTime) continue;
        eventFired[i] = 1;
        for (auto& obj : objects)
            if (InsideEvent(e, obj.pos)) obj.vel += e.accel;
    }
}

// Applies every Continuous event active at sim time `t` as an acceleration over `subDt`. Goes onto
// vel, not acc, because acc is only recomputed every forceInterval substeps.
void ApplyContinuousEvents(double t, float subDt) {
    for (const SceneEvent& e : ActiveScene.events) {
        if ((e.mode != EventMode::Continuous && e.mode != EventMode::Field) || t < e.startTime) continue;
        if (e.duration > 0.f && t >= static_cast<double>(e.startTime) + e.duration) continue;
        if (e.mode == EventMode::Field) {
            for (auto& obj : objects)
                if (float ax, ay; EventFieldAccel(e, obj.pos.x, obj.pos.y, ax, ay)) obj.vel += Vec2(ax, ay) * subDt;
            continue;
        }
        const Vec2 dv = e.accel * subDt;
        for (auto& obj : objects)
            if (InsideEvent(e, obj.pos)) obj.vel += dv;
    }
}

// Container half-extents are fixed once per run in ResetSimulation, so editing the scene
// mid-run can't move the walls under the particles. <= 0 = not frozen (menu).
float frozenHalfW = 0.f, frozenHalfH = 0.f;

// ---- Sim thread ---------------------------------------------------------------------------------
// Live (Running) physics runs on a worker thread so a slow or diverging sim can't freeze the
// window. The UI thread asks for time to be simulated; the worker advances it.
//
// simMutex guards all simulation state (objects, ActiveScene, eventFired, simTime, cameraOffsetX,
// profileLogger sampling, the fault message). The worker holds it during a substep and releases it
// between substeps; the UI thread asks the worker to yield (uiWaiting) so it waits at most one
// substep. If the lock can't be taken in time the UI shows a "not responding" notice instead.
// Batch rendering calls AdvanceSimulation from the main thread, where the lock is uncontended.
std::timed_mutex          simMutex;
std::atomic<int>          uiWaiting{ 0 };
std::atomic<uint32_t>     simGeneration{ 0 }; // bumped by ResetSimulation; a step in flight abandons itself
std::atomic<bool>         simAbort{ false };  // set at shutdown; the worker stops at the next substep
std::atomic<bool>         simFaulted{ false };
std::string               simFaultMessage;    // guarded by simMutex

std::mutex                workMutex;
std::condition_variable   workCv;
double                    workPendingDt      = 0.0; // simulated seconds the UI thread wants advanced
int                       workManualSteps    = 0;   // StepDt-sized steps requested by Step / Step x10
bool                      workQuit           = false;
std::thread               simThread;
std::atomic<bool>         simThreadExited{ false };
std::atomic<float>        simStepMs{ 0.f }; // smoothed wall time of one worker step ("Compute")
std::atomic<float>        simSpeed{ 1.f };  // smoothed fraction of real time the sim keeps up with
float                     renderCpuMs  = 0.f; // UI thread: smoothed draw/submit time, excluding waits
float                     lockWaitMs   = 0.f; // UI thread: time LockSim waited this frame

bool LockSim(SimLock& lk, int timeoutMs) {
    const auto start = std::chrono::steady_clock::now();
    uiWaiting++;
    lk = SimLock(simMutex, std::defer_lock);
    const bool ok = lk.try_lock_for(std::chrono::milliseconds(timeoutMs));
    uiWaiting--;
    lockWaitMs += std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start).count();
    return ok;
}

// Marks the sim diverged and stops it advancing until the next reset. Caller holds simMutex.
void SetFaultLocked(const char* message) {
    if (simFaulted) return;
    simFaultMessage = message;
    simFaulted      = true;
    std::fprintf(stderr, "[sim] fault: %s\n", message);
}

void ClearPendingWork() {
    std::lock_guard<std::mutex> lk(workMutex);
    workPendingDt   = 0.0;
    workManualSteps = 0;
}

// Restarts from the active scene: the container is its simulated area (SceneBoundary::resWidth/Height).
void ResetSimulation() {
    simGeneration++;
    ClearPendingWork();
    simFaulted = false;
    simFaultMessage.clear();
    objects.clear();
    {
        const auto& b = ActiveScene.boundary;
        frozenHalfW = 0.5f * static_cast<float>(b.resWidth)  * b.widthFrac;
        frozenHalfH = 0.5f * static_cast<float>(b.resHeight) * b.heightFrac;
        app.SetSimArea(static_cast<float>(b.resWidth), static_cast<float>(b.resHeight));
    }
    Init();
    eventFired.assign(ActiveScene.events.size(), 0);
    paused = false;
    cameraOffsetX = 0.f;
    // A restart begins again at t = 0, so end any in-progress profile (StartRun starts a fresh one).
    profileLogger.Stop();
    simTime = 0.0;
}

// Advances physics by dt. Shared by normal advancement and manual stepping (which uses a fixed dt).
void AdvanceSimulationLocked(float dt) {
    SimLock lk(simMutex);
    if (simFaulted) return;
    const uint32_t generation = simGeneration;
    // Force is recomputed every forceInterval substeps (see Integrate() on stale-force energy gain).
    const int   substeps = std::max(ActiveScene.physics.substeps, 1);
    const float subDt    = dt / static_cast<float>(substeps);
    ApplyOnceEvents();
    for (int step = 0; step < substeps; step++) {
        if (step > 0) {
            // State is consistent between substeps: let a waiting UI thread in.
            lk.unlock();
            while (uiWaiting.load() > 0 && !simAbort) std::this_thread::sleep_for(std::chrono::microseconds(50));
            lk.lock();
            if (simAbort || simFaulted || generation != simGeneration) return; // reset/aborted meanwhile
        }
        if (step % std::max(ActiveScene.physics.forceInterval, 1) == 0) {
            for (auto& obj : objects) obj.acc = Vec2(0.0f, 0.0f);
            Update(subDt);
        }
        ApplyContinuousEvents(simTime + static_cast<double>(step) * subDt, subDt);
        if (!Integrate(subDt)) {
            SetFaultLocked("Simulation diverged: a particle became non-finite or exceeded 1e6 px/s.");
            return;
        }
    }

    simTime += dt;
    // A loop, not an if: dt can span several sample periods (e.g. a 60 Hz profile in a 30 fps
    // batch render).
    while (profileLogger.IsActive() && simTime + 1e-6 >= profileLogger.NextSampleTime())
        profileLogger.Sample();
}

// Never lets an exception escape (it would terminate the worker thread and the process): any
// failure becomes a fault the UI reports, and the sim stops advancing.
void AdvanceSimulation(float dt) {
    try {
        AdvanceSimulationLocked(dt);
    } catch (const std::exception& e) {
        SimLock lk(simMutex);
        SetFaultLocked(e.what());
    } catch (...) {
        SimLock lk(simMutex);
        SetFaultLocked("Simulation failed with an unknown error.");
    }
}

void SimWorkerMain() {
    for (;;) {
        double dt     = 0.0;
        int    manual = 0;
        {
            std::unique_lock<std::mutex> lk(workMutex);
            workCv.wait(lk, [] { return workQuit || workPendingDt > 0.0 || workManualSteps > 0; });
            if (workQuit) break;
            if (workManualSteps > 0) { manual = workManualSteps; workManualSteps = 0; }
            else                     { dt = workPendingDt;       workPendingDt   = 0.0; }
        }
        const auto t0 = std::chrono::steady_clock::now();
        if (manual > 0) {
            for (int i = 0; i < manual && !simAbort && !simFaulted; i++) AdvanceSimulation(StepDt);
        } else {
            AdvanceSimulation(static_cast<float>(dt));
        }
        // Per call, not per manual step: a Step x10 burst counts as one long step.
        const float ms = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
        simStepMs = simStepMs * 0.9f + ms * 0.1f;
        // Simulated seconds per wall second, capped at real time; under 100% means slow motion.
        // Manual steps aren't paced, so they don't count.
        if (manual == 0 && ms > 0.f) {
            const float speed = std::min(1.f, static_cast<float>(dt) / (ms * 0.001f));
            simSpeed = simSpeed * 0.9f + speed * 0.1f;
        }
    }
    simThreadExited = true;
}

void StartSimThread() { simThread = std::thread(SimWorkerMain); }

// Stops the worker. If it's wedged in a substep it is detached instead, and the caller must exit
// without running static destructors (returns false).
bool StopSimThread() {
    simAbort = true;
    {
        std::lock_guard<std::mutex> lk(workMutex);
        workQuit = true;
    }
    workCv.notify_all();
    for (int i = 0; i < 200 && !simThreadExited; i++) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (simThreadExited) { simThread.join(); return true; }
    simThread.detach();
    return false;
}

// Asks the worker to simulate `dt` more seconds, capped at one clamped frame so a slow sim runs
// in slow motion instead of building a backlog.
void QueueSimTime(float dt) {
    {
        std::lock_guard<std::mutex> lk(workMutex);
        workPendingDt = std::min(workPendingDt + static_cast<double>(dt), 1.0 / 30.0);
    }
    workCv.notify_one();
}

void QueueManualSteps(int count) {
    {
        std::lock_guard<std::mutex> lk(workMutex);
        workManualSteps += count;
    }
    workCv.notify_one();
}

void TogglePause() { paused = !paused; }

// Stepping pauses first, so Step/arrow keys freeze the sim on that frame.
void StepOnce() { paused = true; QueueManualSteps(1); }
void StepBig()  { paused = true; QueueManualSteps(BigStepFrames); }

// The run configuration currently driving Running/Rendering (manual or queued), set by StartRun.
RunSettings activeRun;

std::vector<QueueEntry> runQueue;
int  queueRunningIndex = -1; // -1 = no queue run in progress; runQueue itself is kept for re-running

// A queued entry needs a finite end or it would stall the queue. Every enabled output must be
// finite (and at least one enabled), since AdvanceRendering runs until all outputs stop.
bool HasDeterministicEnd(const RunSettings& run) {
    if (!run.record && !run.saveData && !run.saveProfile) return false;
    return (!run.record      || run.lengthSeconds        > 0.f) &&
           (!run.saveData    || run.dataLengthSeconds    > 0.f) &&
           (!run.saveProfile || run.profileLengthSeconds > 0.f);
}

void AdvanceQueue();

// Loads `scene`, resets the sim, and starts the outputs `run` requests. Shared by a manual start
// and queued entries. `forceBatch` is true for queue entries (only batch mode has a deterministic
// end); a manual run uses `run.batchRender`.
void StartRun(const Scene& scene, const RunSettings& run, bool forceBatch) {
    LoadScene(scene);
    activeRun = run;
    ResetSimulation();

    // Shared by both outputs so a run's video and CSV carry the same run number. A title is
    // appended to it; a blank or unsanitizable one leaves the generated name as is. The particle
    // count is the spawned one (after obstacle culling).
    auto withTitle = [](std::string name, const char* title) {
        const std::string clean = SanitizeFilename(title, "");
        return clean.empty() ? name : name + "_" + clean;
    };
    const std::string runBase = RunNaming::BuildBaseName({
        .sceneName      = scene.name,
        .particleCount  = static_cast<int>(objects.size()),
        .gravityEnabled = scene.gravity.enabled,
        .gravityG       = scene.gravity.g,
        .stiffness      = scene.particles.stiffness,
        .runNumber      = RunNaming::NextRunNumber(),
    });

    bool recording   = false;
    bool dataStarted = false;

    if (run.record) {
        // Sanitized here too (idempotent) so the existence check sees the exact path written.
        std::string name = SanitizeFilename(withTitle(runBase, run.title), runBase.c_str());
        name = RunNaming::EnsureUniqueBaseName(name, { { ExecutableDir() / "recordings", ".mp4" } });
        recording = app.StartRecording(name, run.fps, run.lengthSeconds, scene.boundary.resWidth, scene.boundary.resHeight);
    }

    if (run.saveData) {
        DataRecorder::Fields fields;
        fields.position = run.dataPosition;
        fields.velocity = run.dataVelocity;
        fields.speed    = run.dataSpeed;
        fields.density  = run.dataDensity;
        fields.pressure = run.dataPressure;

        const auto  dir = ExecutableDir() / "data";
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        std::string name = withTitle(runBase, run.dataTitle);
        name = RunNaming::EnsureUniqueBaseName(name, { { dir, ".csv" } });
        if (!ec && dataRecorder.Start((dir / (name + ".csv")).string(), fields)) {
            dataStarted      = true;
            dataRateActive   = std::clamp(run.dataRate, 1, 240);
            dataLengthActive = run.dataLengthSeconds;
            dataAccum        = 0.f;
            dataElapsed      = 0.f;
            dataSavedFrames  = 0;
        }
    }

    bool profileStarted = false;
    if (run.saveProfile) {
        const auto  dir = ExecutableDir() / "data";
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        // Viscosity model + parameter go in the name, since RunNaming's base name omits them.
        const auto& p = scene.particles;
        char tag[96];
        if (p.viscosityModel == ViscosityModel::Morris)
            std::snprintf(tag, sizeof(tag), "_Morris_nu%g", p.kinematicViscosity);
        else
            std::snprintf(tag, sizeof(tag), "_Monaghan_a%g_b%g", p.viscosity, p.viscosityQuadratic);
        std::string name = withTitle(runBase + tag, run.profileTitle);
        name = RunNaming::EnsureUniqueBaseName(name, { { dir, "_profile.csv" } });
        if (!ec)
            profileStarted = profileLogger.Start(dir / name, run.profileRate,
                                                 run.profileLengthSeconds, run.profileBins);
    }

    // Entering Rendering only makes sense if something is actually active to drive it — if every
    // output failed to start (e.g. ffmpeg missing and a disk error), fall back to Running rather
    // than a Rendering state AdvanceRendering would find already finished.
    if ((forceBatch || run.batchRender) && (recording || dataStarted || profileStarted)) {
        // AdvanceRendering captures frames via RenderOffscreenFrame; the per-tick RenderFrame()
        // only refreshes the overlay and must not feed the recorder (see SetCaptureFromSwapchain).
        app.SetCaptureFromSwapchain(false);
        appState = AppState::Rendering;
    } else {
        appState = AppState::Running;
    }
}

void StartSimulationFromMenu() {
    StartRun(menuConfig.scene, menuConfig.run, /*forceBatch=*/false);
}

// Starts queue entry `entry` in forced-batch mode; if nothing actually starts (see StartRun's
// fallback), advances past it, since AdvanceQueue is only reachable from AdvanceRendering.
void StartQueueEntry(const QueueEntry& entry) {
    StartRun(entry.scene, entry.run, /*forceBatch=*/true);
    if (appState != AppState::Rendering) AdvanceQueue();
}

void StartQueue() {
    if (runQueue.empty()) return;
    queueRunningIndex = 0;
    StartQueueEntry(runQueue[0]);
}

// Resets the sim in place without leaving Running or touching active outputs (unlike ReturnToMenu).
void RestartSimulation() {
    ResetSimulation();
}

void ReturnToMenu() {
    queueRunningIndex = -1; // abort the rest of the queue; runQueue itself stays intact
    frozenHalfW = frozenHalfH = 0.f;
    app.StopRecording();
    dataRecorder.Stop();
    profileLogger.Stop();
    objects.clear();
    // Restore the default so the next live recording captures normally; only batch rendering turns this off.
    app.SetCaptureFromSwapchain(true);
    appState = AppState::Menu;
}

// Stops what the finished queue entry left active and starts the next, or returns to Menu once
// the queue is exhausted. Leaves runQueue intact so it can be run again.
void AdvanceQueue() {
    app.StopRecording();
    dataRecorder.Stop();
    profileLogger.Stop();
    queueRunningIndex++;
    if (queueRunningIndex >= static_cast<int>(runQueue.size())) {
        queueRunningIndex = -1;
        frozenHalfW = frozenHalfH = 0.f;
        objects.clear();
        app.SetCaptureFromSwapchain(true);
        appState = AppState::Menu;
        return;
    }
    StartQueueEntry(runQueue[queueRunningIndex]);
}
