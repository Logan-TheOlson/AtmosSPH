#pragma once
// Declarations shared between the Engine*.cpp files. Engine.h is the public API; this header is
// private to src/engine.
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "engine/Object.h"
#include "engine/ProfileLogger.h"
#include "engine/Scene.h"
#include "engine/ScenePresetIO.h"
#include "renderer/App.h"
#include "ui/EventGizmo.h"
#include "ui/Workspace.h"
#include "util/DataRecorder.h"
#include "util/Profiler.h"

// ---- Types -------------------------------------------------------------------------------------

// The simulation only advances in Running/Rendering; Menu skips physics entirely. Rendering runs
// the same physics back-to-back at a fixed 1/fps dt, drawn and captured straight to video with
// no real-time pacing (see AdvanceRendering/DrawRenderingOverlay).
enum class AppState { Menu, Running, Rendering };

// Recording + data-save configuration for one run (a manual start or one queued entry), kept
// separate from the Scene so a queue entry can snapshot both.
struct RunSettings {
    bool  record = false;
    // Optional short label appended to the generated filename (see RunNaming).
    char  title[128] = "";
    int   fps = 60;
    float lengthSeconds = 10.f;
    // Only with `record`: renders and captures every frame back-to-back at 1/fps instead of
    // pacing to real time (see AdvanceRendering).
    bool  batchRender = false;

    bool  saveData = false;
    char  dataTitle[128] = "";
    int   dataRate = 30; // samples/sec, independent of render fps
    float dataLengthSeconds = 10.f;
    bool  dataPosition = true;
    bool  dataVelocity = true;
    bool  dataSpeed = false;
    bool  dataDensity = true;
    bool  dataPressure = true;

    // Channel velocity profile (see ProfileLogger): per-bin averages instead of per-particle rows.
    bool  saveProfile = false;
    char  profileTitle[128] = "";
    int   profileRate = 5;               // samples per SIM second
    float profileLengthSeconds = 75.f;   // sim seconds; 0 = unlimited
    int   profileBins = 0;               // 0 = one bin per particle row (spawn.gridCountY)
};

struct MenuConfig {
    // Index into the built-in ScenePresets followed by userPresets (see the Combined helpers).
    int          sceneIndex = 0;
    // Editable working copy, reseeded from BuildScene()/LoadScenePreset() only when sceneIndex or
    // the grid count changes. Assigned in RunEngine(), not via a default member initializer.
    Scene        scene;
    RunSettings  run;
};

// One queued run: a Scene snapshot plus its own output settings, captured by value so later menu
// edits don't change it.
struct QueueEntry {
    Scene       scene;
    RunSettings run;
};

// Live coloring: recolors every particle by a simulation field instead of its scene color. Each
// field has its own editable low/high color pair.
enum class ColorMode { Off, Speed, Pressure, Density };
struct ColorRamp { Color low; Color high; };

// ---- Constants ---------------------------------------------------------------------------------

constexpr float StepDt        = 1.f / 60.f; // fixed frame length for manual stepping
constexpr int   BigStepFrames = 10;         // frames advanced by Left Arrow / the "Step x10" button

// ---- Application state (EngineSim.cpp) ---------------------------------------------------------

extern App          app;
extern Profiler     profiler; // frame/render timing, shown in the live panel's Run card
extern AppState     appState;
extern MenuConfig   menuConfig;
extern RunSettings  activeRun; // the run currently driving Running/Rendering, set by StartRun

// Freezes physics (rendering, recording and data-saving keep running). Toggled by Space or the
// Pause/Resume button; only Restart/Menu/StartSimulation reset it.
extern bool paused;

// Periodic-channel scenes drift sideways as the force advects the cloud, burying the profile
// under bulk translation. When enabled, the drawn (not simulated) x position is offset by the
// accumulated mean flow velocity each substep (see Integrate), so only relative motion shows.
extern bool  cameraFollow;
extern float cameraOffsetX;

// Simulated seconds since the last ResetSimulation(); advanced only by AdvanceSimulation.
extern double simTime;

// Container half-extents are fixed once per run in ResetSimulation, so editing the scene
// mid-run can't move the walls under the particles. <= 0 = not frozen (menu).
extern float frozenHalfW, frozenHalfH;

std::filesystem::path ExecutableDir();
void WriteVelocityProfileCsv();

// Container half-extents for the current state (Engine.cpp).
float ScreenHalfWidth();
float ScreenHalfHeight();

// Presets saved via the menu; the scene combo lists the built-ins first, then these.
extern std::vector<ScenePresetEntry> userPresets;
void RefreshUserPresets();
int  CombinedCount();
bool CombinedIsBuiltin(int i);
const std::string& CombinedName(int i);
const std::string& CombinedDescription(int i);

// Data saving / profile logging. Owned here because the data they sample, `objects`, lives here.
extern DataRecorder  dataRecorder;
extern float         dataAccum;        // seconds since the last sample, gated at the active run's dataRate
extern float         dataElapsed;      // sim seconds since this data save started (CSV time column)
extern uint32_t      dataSavedFrames;
extern int           dataRateActive;
extern float         dataLengthActive; // 0 = unlimited
extern ProfileLogger profileLogger;

// Live coloring.
extern ColorMode  colorMode;
extern const char* ColorModeNames[4];
extern ColorRamp  colorRamps[]; // indexed by ColorMode; entry 0 (Off) is unused
float FieldValue(const Object& o, ColorMode mode);
Color LerpColor(const Color& a, const Color& b, float t);

// ---- Events ------------------------------------------------------------------------------------

// Whether each Once event has fired this run. Parallel to ActiveScene.events; reset by ResetSimulation.
extern std::vector<char> eventFired;
bool InsideEvent(const SceneEvent& e, const Vec2& p);
void ApplyOnceEvents();
void ApplyContinuousEvents(double t, float subDt);

// Physics substep (Engine.cpp). Returns false if the sim has diverged.
bool Integrate(float subDt);

// ---- Sim thread and run control (EngineSim.cpp) ------------------------------------------------

extern std::timed_mutex          simMutex;
extern std::atomic<int>          uiWaiting;
extern std::atomic<uint32_t>     simGeneration;
extern std::atomic<bool>         simAbort;
extern std::atomic<bool>         simFaulted;
extern std::string               simFaultMessage; // guarded by simMutex
extern std::atomic<float>        simStepMs;       // smoothed wall time of one worker step ("Compute")
extern std::atomic<float>        simSpeed;        // smoothed fraction of real time the sim keeps up with
extern float                     renderCpuMs;     // UI thread: smoothed draw/submit time, excluding waits
extern float                     lockWaitMs;      // UI thread: time LockSim waited this frame

using SimLock = std::unique_lock<std::timed_mutex>;

// Takes simMutex for the UI thread, making the worker yield. False if the worker is stuck in a
// single substep longer than `timeoutMs`; the caller must then not touch sim state.
bool LockSim(SimLock& lk, int timeoutMs = 100);

void ResetSimulation();
void AdvanceSimulation(float dt);
void StartSimThread();
bool StopSimThread();
void QueueSimTime(float dt);
void QueueManualSteps(int count);
void TogglePause();
void StepOnce();
void StepBig();

extern std::vector<QueueEntry> runQueue;
extern int queueRunningIndex; // -1 = no queue run in progress; runQueue itself is kept for re-running
bool HasDeterministicEnd(const RunSettings& run);
void StartRun(const Scene& scene, const RunSettings& run, bool forceBatch);
void StartSimulationFromMenu();
void StartQueue();
void RestartSimulation();
void ReturnToMenu();
void AdvanceQueue();

// ---- UI (EngineUi.cpp, EnginePanels.cpp) -------------------------------------------------------

// The single UI callback, and the stripped-down one used when the sim thread is unresponsive.
void DrawUi();
void DrawUnresponsiveUi();
// Menu state: draws a physics-free preview of the scene's particles through the renderer.
void AddMenuPreviewParticles();

// Panels drawn by DrawUi (EnginePanels.cpp). DrawSimOverlay draws the sim-view gizmos and the setup preview.
void DrawEventsPanel();
void DrawColorsWindow();
void DrawGroupsWindow();
void DrawObstaclesWindow();
void DrawSimOverlay(const Workspace::Frame& frame);

// Container half-extents as the setup view shows them (same rule as DrawSimOverlay's preview).
void MenuHalfExtents(const Scene& s, float& halfW, float& halfH);

// Layout helpers shared by the panels.
void FormRow(float contentWidth, const char* label);
void BeginCard(const char* id);
void EndCard();
std::vector<SceneEvent>& UiEvents();

// Selection and placement state for the editable lists.
extern int  selectedEvent;
extern int  selectedRegion;
extern int  selectedObstacle;
extern int  selectedGroup;
extern bool colorsTabActive;
extern bool obstaclesTabActive;
extern bool groupsTabActive;
extern bool showEventAreas;
extern SceneEvent       placeEventTemplate;
extern SceneColorRegion placeRegionTemplate;
extern SceneObstacle    placeObstacleTemplate;
extern SceneGroup       placeGroupTemplate;
