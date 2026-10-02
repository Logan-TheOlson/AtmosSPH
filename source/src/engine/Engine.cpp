#include <imgui_internal.h>
#include <SDL3/SDL.h>
#include <mutex>
#include "util/ColorPattern.h"
#include "engine/Engine.h"
#include "engine/Scene.h"
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
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

// User-defined entry points
void Init();
void Update(float dt);

std::vector<Object> objects;

// Directory the running executable lives in, so output files land next to it regardless of the
// process's current working directory.
static std::filesystem::path ExecutableDir() {
#ifdef _WIN32
    char path[MAX_PATH];
    DWORD len = GetModuleFileNameA(nullptr, path, MAX_PATH);
    if (len > 0 && len < MAX_PATH)
        return std::filesystem::path(path, path + len).parent_path();
#endif
    return std::filesystem::current_path();
}

// One kinetic-energy sample: `run` distinguishes separate Start-Simulation/Restart sessions
// within the same process, so a discontinuity from a reset doesn't read as an instability event
// when the CSV is plotted.
struct KESample { int run; int frame; float ke; };

// Appends run,frame,kineticEnergy rows collected over the process's lifetime to a CSV next to the
// executable, for offline comparison (e.g. plotting energy stability across runs in Python).
static void WriteKineticEnergyCsv(const std::vector<KESample>& keByFrame) {
    const auto outPath = ExecutableDir() / "kinetic_energy.csv";
    std::ofstream out(outPath);
    if (!out) return;

    out << "run,frame,kinetic_energy\n";
    for (const auto& s : keByFrame)
        out << s.run << ',' << s.frame << ',' << s.ke << '\n';
}

// Bins particles by y-position across the current scene's channel height and writes each bin's
// average x-velocity to a CSV next to the executable — e.g. the parabolic profile a Poiseuille
// scene should develop. Overwritten every time this is called (see the main loop's call site),
// so the file always reflects a recent snapshot rather than one moment early in the transient.
static void WriteVelocityProfileCsv() {
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

static App      app;
static Profiler profiler;
static bool     profilerOpen = false;

// Simulation only advances in Running/Rendering; Menu leaves `objects` untouched (cleared) and
// skips physics entirely so the menu screen never pays SPH cost. Rendering runs the same physics
// as Running, but crunched back-to-back at a fixed 1/fps dt and always drawn+captured straight to
// video — no real-time pacing, and no live playback afterward (see AdvanceRendering/
// DrawRenderingOverlay).
enum class AppState { Menu, Running, Rendering };
static AppState appState = AppState::Menu;

// Freezes physics (rendering, recording, and CSV data-saving all keep running) — toggled by
// Space or the Pause/Resume button. Independent of appState so a paused sim stays paused across
// UI redraws; only Restart/Menu/StartSimulation touch it.
static bool paused = false;

// Periodic-channel scenes (Poiseuille) drift the whole particle cloud sideways as the imposed
// force advects it, which makes the pressure-driven profile (fast center, slow walls) hard to
// read — it's buried under the bulk translation. When enabled, the drawn (not simulated) x
// position is offset by the accumulated mean flow velocity each substep (see Integrate), so the
// cloud appears stationary on average and only the relative motion between fast/slow bands shows.
static bool  cameraFollow  = true;
static float cameraOffsetX = 0.f;

constexpr float StepDt         = 1.f / 60.f; // fixed nominal frame length for manual stepping, so a
                                              // step is deterministic regardless of real elapsed time
constexpr int   BigStepFrames  = 10;         // frames advanced by Left Arrow / the "Step x10" button

// Recording + data-save configuration for one run — a manual "Start Simulation" click or one
// queued entry (see QueueEntry) — kept separate from the Scene itself so a queue entry can
// snapshot both independently.
struct RunSettings {
    bool  record = false;
    // Optional short label appended to the auto-generated filename (see RunNaming) — empty by
    // default, unlike the old flat "capture"/"data" titles this replaces, since the generated
    // name alone is already distinguishing.
    char  title[128] = "";
    int   fps = 60;
    float lengthSeconds = 10.f;
    // Only meaningful alongside `record`: renders+captures every frame back-to-back at 1/fps
    // instead of pacing to real time, so the video finishes in however long that takes to compute
    // rather than lengthSeconds of real time — see AdvanceRendering.
    bool  batchRender = false;

    bool  saveData = false;
    char  dataTitle[128] = "";
    int   dataRate = 30; // samples/sec, independent of render fps — see the "Save data to CSV" section
    float dataLengthSeconds = 10.f;
    bool  dataPosition = true;
    bool  dataVelocity = true;
    bool  dataSpeed = false;
    bool  dataDensity = true;
    bool  dataPressure = true;

    // Channel velocity profile (see ProfileLogger): per-bin averages across the channel height
    // instead of per-particle rows, so a long Poiseuille run stays well under a few MB.
    bool  saveProfile = false;
    char  profileTitle[128] = "";
    int   profileRate = 5;               // samples per SIM second
    float profileLengthSeconds = 75.f;   // sim seconds; 0 = unlimited
    int   profileBins = 0;               // 0 = one bin per particle row (spawn.gridCountY)
};

struct MenuConfig {
    // Index into the combined list of the 3 compiled ScenePresets followed by userPresets (see
    // CombinedX helpers below) — not just ScenePresets, so a saved custom preset shows up in the
    // same combo.
    int          sceneIndex = 0;
    // Full editable working copy — reseeded from BuildScene()/BuildSceneFromPreset() only when
    // sceneIndex or the grid count actually changes (see DrawMenu); every other field is free to
    // hand-edit and is left exactly as the user set it on every other frame. Assigned for real in
    // main(), not via a default member initializer — see the comment there for why.
    Scene        scene;
    RunSettings  run;
};
static MenuConfig menuConfig;

// Presets saved via the menu's "Presets" section, scanned from disk at startup and after each
// save. Combined with the 3 compiled ScenePresets (Scene.h) into one list for the scene combo —
// built-ins keep fixed indices [0, ScenePresets.size()), user presets follow.
static std::vector<ScenePresetEntry> userPresets;

static void RefreshUserPresets() {
    userPresets = ScanUserPresets(ExecutableDir() / "presets");
}

static int CombinedCount() {
    return static_cast<int>(ScenePresets.size() + userPresets.size());
}
static bool CombinedIsBuiltin(int i) {
    return i < static_cast<int>(ScenePresets.size());
}
static const std::string& CombinedName(int i) {
    return CombinedIsBuiltin(i) ? ScenePresets[i].name
                                 : userPresets[i - static_cast<int>(ScenePresets.size())].name;
}
static const std::string& CombinedDescription(int i) {
    return CombinedIsBuiltin(i) ? ScenePresets[i].description
                                 : userPresets[i - static_cast<int>(ScenePresets.size())].description;
}

// Owned directly by Engine.cpp (unlike Recorder, which lives behind App/VulkanContext because it
// needs the composited GPU frame) since the data it samples — `objects` — already lives here.
static DataRecorder dataRecorder;
static float        dataAccum{ 0.f };        // seconds since the last sample, gated at the active run's dataRate, like Recorder's fps gate
static float        dataElapsed{ 0.f };       // sim seconds since this data save started, for the CSV's time column
static uint32_t     dataSavedFrames{ 0 };
static int          dataRateActive{ 0 };
static float        dataLengthActive{ 0.f };  // 0 = unlimited, mirrors the active run's video length cap

// Defined further down; declared here for ProfileLogger.
float ScreenHalfWidth();
float ScreenHalfHeight();

// Simulated seconds since the last ResetSimulation() — advanced only by AdvanceSimulation, so it
// stops while paused and is exact in both live and batch modes (unlike the real-time dt the
// per-particle data save accumulates).
static double simTime = 0.0;

// Channel velocity-profile logger for Poiseuille-type runs. Every 1/rate simulated seconds it bins
// particles by y across the channel ([-H/2, H/2], wall to wall) and writes two small CSVs:
//   <name>_profile.csv : frame,time,bin,y_center,count,mean_vx,std_vx,mean_vy,mean_density
//   <name>_frames.csv  : frame,time,mean_vx,max_vx,rms_vy,mean_density,std_density,min_density,max_density
// Both start with '#'-prefixed lines recording the run's geometry and parameters (H and L as
// actually used, so the analysis never has to assume them) — read with pandas' comment='#'.
// Written synchronously on the sim thread: one sample is only nBins+1 rows.
struct ProfileLogger {
    std::ofstream profile;
    std::ofstream frames;
    int    nBins       = 0;
    int    rate        = 5;
    float  length      = 0.f;
    int    savedFrames = 0;

    bool IsActive() const { return profile.is_open(); }
    double NextSampleTime() const { return static_cast<double>(savedFrames) / rate; }
    float SavedSeconds() const { return savedFrames > 0 ? static_cast<float>(savedFrames - 1) / rate : 0.f; }

    void Stop() {
        if (profile.is_open()) profile.close();
        if (frames.is_open())  frames.close();
    }

    bool Start(const std::filesystem::path& base, int sampleRate, float lengthSeconds, int bins) {
        Stop();
        const auto& s = ActiveScene;
        nBins       = bins > 0 ? bins : std::max(s.spawn.gridCountY, 1);
        rate        = std::clamp(sampleRate, 1, 240);
        length      = std::max(lengthSeconds, 0.f);
        savedFrames = 0;

        profile.open(base.string() + "_profile.csv", std::ios::out | std::ios::trunc);
        frames.open(base.string() + "_frames.csv", std::ios::out | std::ios::trunc);
        if (!profile.is_open() || !frames.is_open()) {
            fprintf(stderr, "ProfileLogger: failed to open %s_*.csv\n", base.string().c_str());
            Stop();
            return false;
        }

        const auto& p = s.particles;
        const bool  morris = p.viscosityModel == ViscosityModel::Morris;
        for (std::ofstream* f : { &profile, &frames }) {
            *f << "# scene=" << s.name << '\n'
               << "# H=" << 2.f * ScreenHalfHeight() << '\n'
               << "# L=" << 2.f * ScreenHalfWidth() << '\n'
               << "# N=" << objects.size() << '\n'
               << "# gridCountX=" << s.spawn.gridCountX << '\n'
               << "# gridCountY=" << s.spawn.gridCountY << '\n'
               << "# spacing=" << p.circleRadius * 3.f * s.spawn.spacingScale << '\n'
               << "# h=" << p.smoothingRadius << '\n'
               << "# eos=" << (p.eos == EosModel::WCSPH ? "WCSPH" : "Polytropic") << '\n'
               << "# stiffness=" << p.stiffness << '\n'
               << "# c=" << std::sqrt(p.stiffness) << '\n' // WCSPH reference sound speed
               << "# targetDensity=" << p.targetDensity << '\n'
               << "# viscosityModel=" << (morris ? "Morris" : "Monaghan") << '\n'
               << "# nu=" << (morris ? p.kinematicViscosity : 0.f) << '\n'
               << "# alpha=" << (morris ? 0.f : p.viscosity) << '\n'
               << "# beta=" << (morris ? 0.f : p.viscosityQuadratic) << '\n'
               << "# forceX=" << s.physics.force.x << '\n'
               << "# forceY=" << s.physics.force.y << '\n'
               << "# noSlipWalls=" << (s.physics.noSlipWalls ? 1 : 0) << '\n'
               << "# periodicX=" << (s.boundary.periodicX ? 1 : 0) << '\n'
               << "# substeps=" << s.physics.substeps << '\n'
               << "# sampleRate=" << rate << '\n'
               << "# bins=" << nBins << '\n';
        }
        profile << "frame,time,bin,y_center,count,mean_vx,std_vx,mean_vy,mean_density\n";
        frames  << "frame,time,mean_vx,max_vx,rms_vy,mean_density,std_density,min_density,max_density\n";

        Sample(); // t = 0: the initial condition, before any step
        return true;
    }

    void Sample() {
        if (!IsActive()) return;
        const float hh   = ScreenHalfHeight();
        const float binH = 2.f * hh / static_cast<float>(nBins);

        std::vector<int>    count(nBins, 0);
        std::vector<double> sVx(nBins, 0.0), sVx2(nBins, 0.0), sVy(nBins, 0.0), sRho(nBins, 0.0);
        double gVx = 0.0, gVy2 = 0.0, gRho = 0.0, gRho2 = 0.0;
        double maxVx = -1e30, minRho = 1e30, maxRho = -1e30;

        for (const auto& obj : objects) {
            const int b = std::clamp(static_cast<int>((obj.pos.y + hh) / binH), 0, nBins - 1);
            const double vx = obj.vel.x, vy = obj.vel.y, rho = obj.density;
            count[b]++; sVx[b] += vx; sVx2[b] += vx * vx; sVy[b] += vy; sRho[b] += rho;
            gVx += vx; gVy2 += vy * vy; gRho += rho; gRho2 += rho * rho;
            maxVx  = std::max(maxVx, vx);
            minRho = std::min(minRho, rho);
            maxRho = std::max(maxRho, rho);
        }

        const int    frame = savedFrames;
        const double t     = simTime;
        for (int b = 0; b < nBins; b++) {
            const double yc = -hh + (b + 0.5) * binH;
            const int    c  = count[b];
            const double mVx  = c ? sVx[b] / c : 0.0;
            const double var  = c ? std::max(sVx2[b] / c - mVx * mVx, 0.0) : 0.0;
            profile << frame << ',' << t << ',' << b << ',' << yc << ',' << c << ','
                    << mVx << ',' << std::sqrt(var) << ','
                    << (c ? sVy[b] / c : 0.0) << ',' << (c ? sRho[b] / c : 0.0) << '\n';
        }

        const double n    = std::max<double>(static_cast<double>(objects.size()), 1.0);
        const double mRho = gRho / n;
        frames << frame << ',' << t << ',' << gVx / n << ',' << maxVx << ','
               << std::sqrt(gVy2 / n) << ',' << mRho << ','
               << std::sqrt(std::max(gRho2 / n - mRho * mRho, 0.0)) << ','
               << minRho << ',' << maxRho << '\n';

        savedFrames++;
        if (length > 0.f && t + 1e-6 >= length) Stop();
    }
};
static ProfileLogger profileLogger;

// Identifies which Start-Simulation/Restart session a KESample belongs to (see WriteKineticEnergyCsv).
static int runIndex = -1;

// Accumulated across the process's lifetime and written once at shutdown (see main()) — file
// scope (not a main()-local) so DrawAndSubmitFrame can push to it from both Running's per-real-
// frame call site and AdvanceRendering's per-video-frame one.
static std::vector<KESample> keByFrame;
static int lastRunIndexForKE = -1;
static int runFrameForKE     = 0;

// Live coloring: recolors every particle by a current simulation field instead of its scene
// color, so a fast flow (e.g. Poiseuille) is readable even when it's moving too fast to track
// individual particles. Each field gets its own low/high color pair, independently editable, so
// switching modes is visually obvious rather than "the same gradient, different numbers".
enum class ColorMode { Off, Speed, Pressure, Density };
static const char* ColorModeNames[] = { "Off (scene color)", "Speed", "Pressure", "Density" };

struct ColorRamp { Color low; Color high; };
static ColorMode colorMode = ColorMode::Off;
// Indexed by ColorMode; entry 0 (Off) is unused.
static ColorRamp colorRamps[] = {
    {},
    { {0.15f, 0.25f, 0.95f, 1.f}, {0.95f, 0.15f, 0.10f, 1.f} }, // Speed:    blue   -> red
    { {0.05f, 0.55f, 0.15f, 1.f}, {1.00f, 0.90f, 0.10f, 1.f} }, // Pressure: green  -> yellow
    { {0.55f, 0.05f, 0.65f, 1.f}, {1.00f, 0.55f, 0.05f, 1.f} }, // Density:  purple -> orange
};

static float FieldValue(const Object& o, ColorMode mode) {
    switch (mode) {
        case ColorMode::Speed:    return magnitude(o.vel);
        case ColorMode::Pressure: return o.pressure;
        case ColorMode::Density:  return o.density;
        default:                  return 0.f;
    }
}

static Color LerpColor(const Color& a, const Color& b, float t) {
    return { a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t, 1.f };
}

// The mode + gradient picker that decides how DrawAndSubmitFrame colors particles — shared by
// DrawRunningOverlay (the live corner panel) and DrawMenu's Record section, so a batch render
// (which never visits Running) still gets a chance to set this before the video starts, not just
// whatever colorMode happened to be left over from a previous live session.
static void DrawColorByControls(float contentWidth) {
    const float swatchSize = ImGui::GetFrameHeight();

    ImGui::TextUnformatted("Color by");
    ImGui::SetNextItemWidth(contentWidth);
    int modeIdx = static_cast<int>(colorMode);
    if (ImGui::Combo("##colorby", &modeIdx, ColorModeNames, IM_ARRAYSIZE(ColorModeNames)))
        colorMode = static_cast<ColorMode>(modeIdx);

    if (colorMode != ColorMode::Off) {
        ColorRamp& ramp = colorRamps[modeIdx];
        ImGui::Spacing();

        // Label on the left, swatch pinned to the content's right edge — NoInputs leaves just the
        // swatch button here, all sliders live in the popup it opens.
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Low");
        ImGui::SameLine(contentWidth - swatchSize);
        ImGui::ColorEdit3("##low", &ramp.low.r, ImGuiColorEditFlags_NoInputs);

        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("High");
        ImGui::SameLine(contentWidth - swatchSize);
        ImGui::ColorEdit3("##high", &ramp.high.r, ImGuiColorEditFlags_NoInputs);
    }
}

// Per-Scene-sub-struct editable sections for the menu's CollapsingHeaders (see DrawMenu) — split
// out the same way DrawColorByControls is, one function per struct rather than one giant flat
// block, so each preset's full parameter set is reachable without recompiling.

// One labelled input row: the label in a fixed left-hand column (wrapping onto a second line if
// it's long), the widget filling the right-hand column. Every labelled menu field goes through
// this. The old layout drew the label to the RIGHT of the widget and shrank the widget by the
// label's width , so a long label squeezed the input box and its +/-
// buttons until the value was cut off. Here the widget width never depends on the label.
// Call immediately before the widget, and give the widget a "##"-hidden label.
static void FormRow(float contentWidth, const char* label) {
    const float labelW  = std::floor(contentWidth * 0.52f);
    const float startX  = ImGui::GetCursorPosX();
    const float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
    ImGui::AlignTextToFramePadding();
    ImGui::PushTextWrapPos(startX + labelW - spacing);
    ImGui::TextUnformatted(label);
    ImGui::PopTextWrapPos();
    ImGui::SameLine();
    ImGui::SetCursorPosX(startX + labelW);
    ImGui::SetNextItemWidth(contentWidth - labelW);
}

static void DrawSpawnSection(SceneSpawn& spawn, float contentWidth) {
    FormRow(contentWidth, "Offset X (frac)");
    ImGui::SliderFloat("##Offset X (frac)", &spawn.offsetXFrac, -1.f, 1.f, "%.2f");
    FormRow(contentWidth, "Packing (spacing x)");
    ImGui::SliderFloat("##Packing (spacing x)", &spawn.spacingScale, 0.7f, 1.2f, "%.2f");
}

// Persistent-color patches (see SceneColorRegion): particles spawned inside keep the color all run.
// Edited in the setup view like events: right-panel tab + drag gizmo (see DrawColorRegionsPanel).
static SceneColorRegion placeRegionTemplate;
static int              selectedRegion = -1;
// True on frames where the "Persistent Color" window is the visible tab of the right dock node.
static bool             colorsTabActive = false;

static void DrawColorRegionFields(SceneColorRegion& r, float contentWidth) {
    static const char* ShapeNames[] = { "Circle", "Rect" };
    int shape = static_cast<int>(r.shape);
    FormRow(contentWidth, "Shape");
    if (ImGui::Combo("##Shape", &shape, ShapeNames, 2)) r.shape = static_cast<EventShape>(shape);
    FormRow(contentWidth, "Center X (px)");
    ImGui::InputFloat("##cx", &r.cx);
    FormRow(contentWidth, "Center Y (px)");
    ImGui::InputFloat("##cy", &r.cy);
    if (r.shape == EventShape::Circle) {
        FormRow(contentWidth, "Radius (px)");
        ImGui::InputFloat("##radius", &r.radius);
        r.radius = std::max(r.radius, 0.f);
    } else {
        FormRow(contentWidth, "Half-width (px)");
        ImGui::InputFloat("##hw", &r.halfW);
        FormRow(contentWidth, "Half-height (px)");
        ImGui::InputFloat("##hh", &r.halfH);
        r.halfW = std::max(r.halfW, 0.f);
        r.halfH = std::max(r.halfH, 0.f);
    }
    const float swatchSize = ImGui::GetFrameHeight();
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Color");
    ImGui::SameLine(contentWidth - swatchSize);
    ImGui::ColorEdit4("##color", &r.color.r, ImGuiColorEditFlags_NoInputs);
}

static void DrawBoundarySection(SceneBoundary& b, float contentWidth) {
    FormRow(contentWidth, "Width (frac)");
    ImGui::SliderFloat("##Width (frac)", &b.widthFrac, 0.05f, 1.f, "%.2f");
    FormRow(contentWidth, "Height (frac)");
    ImGui::SliderFloat("##Height (frac)", &b.heightFrac, 0.05f, 1.f, "%.2f");
    // > 0 overrides the matching fraction above with a fixed size (see SceneBoundary).
    FormRow(contentWidth, "Half-width (px, 0 = frac)");
    ImGui::InputFloat("##Half-width (px, 0 = frac)", &b.halfWidthPx);
    FormRow(contentWidth, "Half-height (px, 0 = frac)");
    ImGui::InputFloat("##Half-height (px, 0 = frac)", &b.halfHeightPx);
    ImGui::Checkbox("Periodic X", &b.periodicX);
}

static void DrawPhysicsSection(ScenePhysics& p, float contentWidth) {
    FormRow(contentWidth, "Force (x, y)");
    ImGui::InputFloat2("##Force (x, y)", &p.force.x);
    FormRow(contentWidth, "Restitution");
    ImGui::SliderFloat("##Restitution", &p.restitution, 0.f, 1.f, "%.2f");
    FormRow(contentWidth, "Friction");
    ImGui::SliderFloat("##Friction", &p.friction, 0.f, 1.f, "%.2f");
    ImGui::Checkbox("No-slip walls", &p.noSlipWalls);
    FormRow(contentWidth, "Substeps");
    ImGui::InputInt("##Substeps", &p.substeps);
    p.substeps = std::max(p.substeps, 1);
    FormRow(contentWidth, "Force interval");
    ImGui::InputInt("##Force interval", &p.forceInterval);
    p.forceInterval = std::max(p.forceInterval, 1);
}

static const char* GravityMethodNames[] = { "Barnes-Hut", "Genuine 2D" };

static void DrawGravitySection(SceneGravity& g, float contentWidth) {
    ImGui::Checkbox("Enabled", &g.enabled);
    if (!g.enabled) return;

    int methodIdx = static_cast<int>(g.method);
    FormRow(contentWidth, "Method");
    if (ImGui::Combo("##Method", &methodIdx, GravityMethodNames, IM_ARRAYSIZE(GravityMethodNames)))
        g.method = static_cast<GravityMethod>(methodIdx);

    FormRow(contentWidth, "G");
    ImGui::InputFloat("##G", &g.g);
    FormRow(contentWidth, "Particle mass");
    ImGui::InputFloat("##Particle mass", &g.particleMass);
    FormRow(contentWidth, "Softening");
    ImGui::InputFloat("##Softening", &g.softening);

    if (g.method == GravityMethod::BarnesHut) {
        FormRow(contentWidth, "MAC theta");
        ImGui::SliderFloat("##MAC theta", &g.macTheta, 0.1f, 1.5f, "%.2f");
        FormRow(contentWidth, "Max leaf particles");
        ImGui::InputInt("##Max leaf particles", &g.maxLeafParticles);
        g.maxLeafParticles = std::max(g.maxLeafParticles, 1);
    }
    ImGui::Checkbox("Validate vs. brute-force", &g.validate);
}

static const char* EosModelNames[] = { "WCSPH", "Polytropic" };
static const char* ViscosityModelNames[] = { "Monaghan (artificial)", "Morris (physical)" };

static void DrawParticlesSection(SceneParticles& pr, float contentWidth) {
    FormRow(contentWidth, "Circle radius");
    ImGui::InputFloat("##Circle radius", &pr.circleRadius);

    // Label on the left, swatch pinned to the content's right edge, same NoInputs-swatch pattern
    // DrawColorByControls uses for its Low/High rows — a full ColorEdit4 (4 numeric fields plus a
    // swatch) is wider than contentWidth allows room for a trailing label at all.
    {
        const float swatchSize = ImGui::GetFrameHeight();
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Circle color");
        ImGui::SameLine(contentWidth - swatchSize);
        ImGui::ColorEdit4("##circlecolor", &pr.circleColor.r, ImGuiColorEditFlags_NoInputs);
    }

    FormRow(contentWidth, "Smoothing radius");
    ImGui::InputFloat("##Smoothing radius", &pr.smoothingRadius);
    FormRow(contentWidth, "Target density");
    ImGui::InputFloat("##Target density", &pr.targetDensity, 0.f, 0.f, "%.6f");
    FormRow(contentWidth, "Stiffness");
    ImGui::InputFloat("##Stiffness", &pr.stiffness);

    int eosIdx = static_cast<int>(pr.eos);
    FormRow(contentWidth, "EOS");
    if (ImGui::Combo("##EOS", &eosIdx, EosModelNames, IM_ARRAYSIZE(EosModelNames)))
        pr.eos = static_cast<EosModel>(eosIdx);
    if (pr.eos == EosModel::WCSPH) {
        FormRow(contentWidth, "Exponent");
        ImGui::InputInt("##Exponent", &pr.exponent);
    } else {
        FormRow(contentWidth, "Polytropic index");
        ImGui::InputFloat("##Polytropic index", &pr.polytropicIndex);
    }

    int viscIdx = static_cast<int>(pr.viscosityModel);
    FormRow(contentWidth, "Viscosity model");
    if (ImGui::Combo("##Viscosity model", &viscIdx, ViscosityModelNames, IM_ARRAYSIZE(ViscosityModelNames)))
        pr.viscosityModel = static_cast<ViscosityModel>(viscIdx);
    if (pr.viscosityModel == ViscosityModel::Monaghan) {
        FormRow(contentWidth, "Viscosity");
        ImGui::InputFloat("##Viscosity", &pr.viscosity);
        FormRow(contentWidth, "Viscosity (quadratic)");
        ImGui::InputFloat("##Viscosity (quadratic)", &pr.viscosityQuadratic);
    } else {
        FormRow(contentWidth, "Kinematic viscosity (px^2/s)");
        ImGui::InputFloat("##Kinematic viscosity (px^2/s)", &pr.kinematicViscosity);
        pr.kinematicViscosity = std::max(pr.kinematicViscosity, 0.f);
    }
}

// Defined further down (needs ScreenHalfWidth/Height); forward-declared so AdvanceSimulation can
// call it despite living earlier in the file, next to the other pause/step/menu logic.
static void Integrate(float subDt);

// ---- Events (see SceneEvent) -------------------------------------------------------------------
static bool       showEventAreas     = true;  // draw event gizmos over the live view (ImGui only, never recorded)
static SceneEvent placeEventTemplate = [] { SceneEvent e; e.accel = Vec2(400.f, 0.f); return e; }();
// Whether each ActiveScene.events[i] of mode Once has already fired this run. Kept parallel to the
// live list (the live editor inserts/erases in step) and reset by ResetSimulation.
static std::vector<char> eventFired;

static bool InsideEvent(const SceneEvent& e, const Vec2& p) {
    const float dx = p.x - e.cx, dy = p.y - e.cy;
    if (e.shape == EventShape::Circle) return dx * dx + dy * dy <= e.radius * e.radius;
    return std::fabs(dx) <= e.halfW && std::fabs(dy) <= e.halfH;
}

// Adds each due Once event's velocity delta to every particle inside its area — once per run.
static void ApplyOnceEvents() {
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

// Applies every Continuous event active at sim time `t` as an acceleration over `subDt`. Goes
// straight onto vel (not acc) because acc is only zeroed/recomputed every forceInterval substeps.
static void ApplyContinuousEvents(double t, float subDt) {
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

static const char* EventShapeNames[] = { "Circle", "Rect" };
static const char* EventModeNames[]  = { "Continuous (acceleration)", "Once (velocity kick)", "Vector field" };
static const char* FieldKindNames[]  = { "Curl (swirl)", "Attract (to center)", "Repel (from center)" };

static void DrawEventFields(SceneEvent& e, float contentWidth) {
    int shape = static_cast<int>(e.shape), mode = static_cast<int>(e.mode);
    FormRow(contentWidth, "Shape");
    if (ImGui::Combo("##Shape", &shape, EventShapeNames, 2)) e.shape = static_cast<EventShape>(shape);
    FormRow(contentWidth, "Mode");
    if (ImGui::Combo("##Mode", &mode, EventModeNames, 3)) e.mode = static_cast<EventMode>(mode);

    FormRow(contentWidth, "Center X (px)");
    ImGui::InputFloat("##Center X (px)", &e.cx);
    FormRow(contentWidth, "Center Y (px)");
    ImGui::InputFloat("##Center Y (px)", &e.cy);
    if (e.shape == EventShape::Circle) {
        FormRow(contentWidth, "Radius (px)");
        ImGui::InputFloat("##Radius (px)", &e.radius);
        e.radius = std::max(e.radius, 0.f);
    } else {
        FormRow(contentWidth, "Half-width (px)");
        ImGui::InputFloat("##Half-width (px)", &e.halfW);
        FormRow(contentWidth, "Half-height (px)");
        ImGui::InputFloat("##Half-height (px)", &e.halfH);
        e.halfW = std::max(e.halfW, 0.f);
        e.halfH = std::max(e.halfH, 0.f);
    }

    const bool once = e.mode == EventMode::Once;
    if (e.mode == EventMode::Field) {
        int kind = static_cast<int>(e.fieldKind);
        FormRow(contentWidth, "Field");
        if (ImGui::Combo("##Field kind", &kind, FieldKindNames, 3)) e.fieldKind = static_cast<FieldKind>(kind);
        FormRow(contentWidth, e.fieldKind == FieldKind::Curl ? "Strength (px/s^2, - = clockwise)" : "Strength (px/s^2)");
        ImGui::InputFloat("##Field strength", &e.strength);
        ImGui::Checkbox("Fade to zero at the edge", &e.fade);
    } else {
        FormRow(contentWidth, once ? "Velocity kick X (px/s)" : "Accel X (px/s^2)");
        ImGui::InputFloat("##Event X", &e.accel.x);
        FormRow(contentWidth, once ? "Velocity kick Y (px/s)" : "Accel Y (px/s^2)");
        ImGui::InputFloat("##Event Y", &e.accel.y);
    }
    FormRow(contentWidth, "Start time (s)");
    ImGui::InputFloat("##Start time (s)", &e.startTime);
    e.startTime = std::max(e.startTime, 0.f);
    if (!once) {
        FormRow(contentWidth, "Duration (s, 0 = unlimited)");
        ImGui::InputFloat("##Duration (s, 0 = unlimited)", &e.duration);
        e.duration = std::max(e.duration, 0.f);
    }
}

// The container's half-extents (for scenes sized from the window rather than in absolute pixels)
// are measured once per run, in ResetSimulation, and held until the run ends -- so resizing the
// window or the docked panels mid-run can't move the walls under the particles (which would crush
// the fluid against a suddenly smaller box and blow the sim up). <= 0 = not frozen (menu).
static float frozenHalfW = 0.f, frozenHalfH = 0.f;

// `fullWindow`: measure against the whole window (recordings and batch renders, which are drawn
// full-frame) instead of the free area between the docked panels (a plain live run).
static void ResetSimulation(bool fullWindow) {
    objects.clear();
    {
        const auto& b = ActiveScene.boundary;
        const bool whole = fullWindow && !app.IsRecording();
        frozenHalfW = (whole ? app.HalfWidth()  : app.RecordingHalfWidth())  * b.widthFrac;
        frozenHalfH = (whole ? app.HalfHeight() : app.RecordingHalfHeight()) * b.heightFrac;
    }
    Init();
    eventFired.assign(ActiveScene.events.size(), 0);
    runIndex++;
    paused = false;
    cameraOffsetX = 0.f;
    // A restarted sim starts again from t = 0, so an in-progress profile would get a second,
    // overlapping time series appended to it — end it instead. (StartRun starts a fresh one after
    // its own ResetSimulation call.)
    profileLogger.Stop();
    simTime = 0.0;
}

// Advances one substep loop's worth of physics by dt — factored out of the main loop so manual
// stepping (paused, one keypress/button = one frame) runs the exact same code path as normal
// per-frame advancement, just with a fixed dt instead of the frame's real one.
static void AdvanceSimulation(float dt) {
    const auto& physics = ActiveScene.physics;
    // Force is recomputed every forceInterval substeps (not once per frame, not every substep) —
    // see Integrate()'s comment for why a stale force is a real energy-gain source, and why
    // recomputing every substep was too expensive at this particle count.
    const float subDt = dt / static_cast<float>(physics.substeps);
    ApplyOnceEvents();
    for (int step = 0; step < physics.substeps; step++) {
        if (step % physics.forceInterval == 0) {
            for (auto& obj : objects) obj.acc = Vec2(0.0f, 0.0f);
            Update(subDt);
        }
        ApplyContinuousEvents(simTime + static_cast<double>(step) * subDt, subDt);
        Integrate(subDt);
    }

    simTime += dt;
    // A loop, not an if: dt can span several sample periods when the profile rate exceeds the step
    // rate (e.g. 60 Hz profile at a 30 fps batch render), and one sample per call would let the
    // frame counter fall behind the time column.
    while (profileLogger.IsActive() && simTime + 1e-6 >= profileLogger.NextSampleTime())
        profileLogger.Sample();
}

static void TogglePause() { paused = !paused; }

// Stepping always pauses first (if not already) — pressing Step/the arrow keys while running
// freezes the sim on that frame rather than requiring a separate pause press first.
static void StepOnce() { paused = true; AdvanceSimulation(StepDt); }
static void StepBig()  { paused = true; for (int i = 0; i < BigStepFrames; i++) AdvanceSimulation(StepDt); }

// The recording/data-save configuration currently driving Running/Rendering — set by StartRun so
// AdvanceRendering's frame rate and DrawRenderingOverlay's progress display reflect whichever
// entry (manual or queued) is actually active, not always menuConfig.run.
static RunSettings activeRun;

// One queued run: a full Scene snapshot plus its own recording/data-save settings, captured by
// value at "Add to Queue" time so later menu edits never retroactively change an already-queued
// entry.
struct QueueEntry {
    Scene       scene;
    RunSettings run;
};
static std::vector<QueueEntry> runQueue;
static int  queueRunningIndex = -1; // -1 = no queue run in progress; runQueue itself is never
                                     // cleared by that, so "Run Queue" can be pressed again after.

// A queued entry needs some condition that will actually end its batch run on its own, or the
// queue would stall forever on it — checked at "Add to Queue" time (see DrawMenu's Run Queue
// section).
// Every enabled output must be finite (and at least one enabled): AdvanceRendering runs until
// *all* outputs have stopped, so a single unlimited one would keep the queue stuck on this entry.
static bool HasDeterministicEnd(const RunSettings& run) {
    if (!run.record && !run.saveData && !run.saveProfile) return false;
    return (!run.record      || run.lengthSeconds        > 0.f) &&
           (!run.saveData    || run.dataLengthSeconds    > 0.f) &&
           (!run.saveProfile || run.profileLengthSeconds > 0.f);
}

static void AdvanceQueue();

// Loads `scene`, resets the sim, and starts whatever recording/data-save `run` requests — the
// shared core behind a manual "Start Simulation" click and a queued entry's turn. `forceBatch` is
// true for every queue entry (so a queued run is always unattended/offscreen, never live Running,
// since only batch mode has a deterministic end) and false for a manual run, which uses its own
// `run.batchRender` checkbox instead.
static void StartRun(const Scene& scene, const RunSettings& run, bool forceBatch) {
    LoadScene(scene);
    activeRun = run;
    ResetSimulation(/*fullWindow=*/run.record || forceBatch || run.batchRender);

    // Shared by both outputs below, so a run's video and CSV carry the same run number and are
    // recognizable as belonging to the same run at a glance.
    const std::string runBase = RunNaming::BuildBaseName({
        .sceneName      = scene.name,
        .gridCountX     = scene.spawn.gridCountX,
        .gridCountY     = scene.spawn.gridCountY,
        .gravityEnabled = scene.gravity.enabled,
        .gravityG       = scene.gravity.g,
        .stiffness      = scene.particles.stiffness,
        .runNumber      = RunNaming::NextRunNumber(),
    });

    bool recording   = false;
    bool dataStarted = false;

    if (run.record) {
        std::string name = runBase;
        if (std::strlen(run.title) > 0) name += "_" + SanitizeFilename(run.title, "");
        // Sanitized here too (StartRecording does it again, idempotently) so the existence check
        // below looks at the exact path that will be written, e.g. when %g put a '+' in the name.
        name = SanitizeFilename(name, "capture");
        name = RunNaming::EnsureUniqueBaseName(name, { { ExecutableDir() / "recordings", ".mp4" } });
        recording = app.StartRecording(name, run.fps, run.lengthSeconds);
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
        std::string name = runBase;
        if (std::strlen(run.dataTitle) > 0) name += "_" + SanitizeFilename(run.dataTitle, "");
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
        // Viscosity model + its parameter in the name, since RunNaming's base name doesn't carry
        // them and they're what distinguishes one Poiseuille run from the next.
        const auto& p = scene.particles;
        char tag[96];
        if (p.viscosityModel == ViscosityModel::Morris)
            std::snprintf(tag, sizeof(tag), "_Morris_nu%g", p.kinematicViscosity);
        else
            std::snprintf(tag, sizeof(tag), "_Monaghan_a%g_b%g", p.viscosity, p.viscosityQuadratic);
        std::string name = runBase + tag;
        if (std::strlen(run.profileTitle) > 0) name += "_" + SanitizeFilename(run.profileTitle, "");
        name = RunNaming::EnsureUniqueBaseName(name, { { dir, "_profile.csv" } });
        if (!ec)
            profileStarted = profileLogger.Start(dir / name, run.profileRate,
                                                 run.profileLengthSeconds, run.profileBins);
    }

    // Entering Rendering only makes sense if something is actually active to drive it — if every
    // output failed to start (e.g. ffmpeg missing and a disk error), fall back to Running rather
    // than a Rendering state AdvanceRendering would find already finished.
    if ((forceBatch || run.batchRender) && (recording || dataStarted || profileStarted)) {
        // AdvanceRendering captures every video frame itself via RenderOffscreenFrame; the
        // per-tick RenderFrame() call (see main()) is only there to keep the progress overlay on
        // screen and must not also feed the recorder — see SetCaptureFromSwapchain.
        app.SetCaptureFromSwapchain(false);
        appState = AppState::Rendering;
    } else {
        appState = AppState::Running;
    }
}

static void StartSimulationFromMenu() {
    StartRun(menuConfig.scene, menuConfig.run, /*forceBatch=*/false);
}

// Starts queue entry `entry` in forced-batch mode; if it fails to actually start anything (see
// StartRun's fallback), immediately advances past it instead of stalling in Running with no
// auto-advance path (AdvanceQueue is only reachable from AdvanceRendering).
static void StartQueueEntry(const QueueEntry& entry) {
    StartRun(entry.scene, entry.run, /*forceBatch=*/true);
    if (appState != AppState::Rendering) AdvanceQueue();
}

static void StartQueue() {
    if (runQueue.empty()) return;
    queueRunningIndex = 0;
    StartQueueEntry(runQueue[0]);
}

// Resets the sim in place without leaving Running (and without touching any active recording or
// data save) — distinct from ReturnToMenu, which is the "stop and reconfigure" path.
static void RestartSimulation() {
    ResetSimulation(activeRun.record || activeRun.batchRender || queueRunningIndex >= 0);
}

static void ReturnToMenu() {
    queueRunningIndex = -1; // abort the rest of the queue, if any; runQueue itself stays intact
    frozenHalfW = frozenHalfH = 0.f;
    app.StopRecording();
    dataRecorder.Stop();
    profileLogger.Stop();
    objects.clear();
    // Restore the default so the next live recording (Running, not batchRender) captures normally
    // — only batch rendering ever turns this off.
    app.SetCaptureFromSwapchain(true);
    appState = AppState::Menu;
}

// Stops whatever the just-finished queue entry left active and starts the next one, or returns to
// Menu once the queue is exhausted — mirrors ReturnToMenu's stop calls but leaves runQueue itself
// intact, so "Run Queue" can be pressed again afterward.
static void AdvanceQueue() {
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

// ================================================================================================
// Docked-panel UI. One top bar, a left panel (Setup tabs / live view controls / render progress,
// depending on appState), a right panel (events list + inspector) and the sim view in the free
// center area, which also hosts the event gizmos (see ui/EventGizmo.h). All of it is drawn by
// DrawUi(), the single UI callback.
// ================================================================================================

// Index into UiEvents() of the event the right panel inspects and the gizmo highlights.
static int selectedEvent = -1;

// The event list the panels/gizmo edit: the menu's scene before a run, the live scene during one
// (so live edits take effect immediately and never touch the menu's copy).
static std::vector<SceneEvent>& UiEvents() {
    return appState == AppState::Menu ? menuConfig.scene.events : ActiveScene.events;
}

// A tinted, rounded "card" grouping a few related rows. Widths inside should come from
// GetContentRegionAvail() so they follow the panel's (user-resizable) width.
static void BeginCard(const char* id) {
    ImGui::PushStyleColor(ImGuiCol_ChildBg, Theme::PanelAlt);
    ImGui::BeginChild(id, ImVec2(0.f, 0.f),
        ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AlwaysUseWindowPadding);
}
static void EndCard() {
    ImGui::EndChild();
    ImGui::PopStyleColor();
}

// ---- Setup tabs (Menu state) ----------------------------------------------------------------

// Which scene index menuConfig.scene was last seeded from (-1 = never), so a scene change is told
// apart from ordinary frames; also re-pointed by "Save Current as Preset" so saving doesn't reseed.
static int lastSyncedSceneIndex = -1;

static void DrawSceneTab() {
    const float w = ImGui::GetContentRegionAvail().x;
    ImGui::PushTextWrapPos(0.f);

    Widgets::Caption("SCENE");
    {
    BeginCard("##sceneCard");
    const float w = ImGui::GetContentRegionAvail().x;
    ImGui::SetNextItemWidth(w);
    if (ImGui::BeginCombo("##scene", CombinedName(menuConfig.sceneIndex).c_str())) {
        for (int i = 0; i < CombinedCount(); i++) {
            const bool isSelected = (i == menuConfig.sceneIndex);
            if (ImGui::Selectable(CombinedName(i).c_str(), isSelected))
                menuConfig.sceneIndex = i;
            if (isSelected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }
    ImGui::TextUnformatted(CombinedDescription(menuConfig.sceneIndex).c_str());
    if (!CombinedIsBuiltin(menuConfig.sceneIndex))
        ImGui::TextDisabled("Custom preset: resizing the grid below won't re-tune its physics constants.");

    ImGui::Separator();
    ImGui::TextUnformatted("Particles");
    int gridX = menuConfig.scene.spawn.gridCountX;
    int gridY = menuConfig.scene.spawn.gridCountY;
    FormRow(w, "Grid X");
    const bool gridXEdited = ImGui::InputInt("##Grid X", &gridX);
    FormRow(w, "Grid Y");
    const bool gridYEdited = ImGui::InputInt("##Grid Y", &gridY);
    gridX = std::max(gridX, 1);
    gridY = std::max(gridY, 1);

    // Reseeds menuConfig.scene from scratch (the preset factory, or the preset file) whenever the
    // scene selection or the grid count actually changes this frame — including the very first
    // DrawMenu call, since lastSyncedSceneIndex starts at a value no real sceneIndex can equal.
    // Every other field is left exactly as the user last edited it on every other frame; see the
    // CollapsingHeader sections below.
    if (menuConfig.sceneIndex != lastSyncedSceneIndex) {
        bool loadedOk = true;
        if (CombinedIsBuiltin(menuConfig.sceneIndex)) {
            const Scene& preset = ScenePresets[menuConfig.sceneIndex];
            menuConfig.scene = BuildScene(menuConfig.sceneIndex, preset.spawn.gridCountX, preset.spawn.gridCountY);
        } else {
            Scene loaded;
            if (LoadScenePreset(userPresets[menuConfig.sceneIndex - static_cast<int>(ScenePresets.size())].path, loaded))
                menuConfig.scene = loaded;
            else
                loadedOk = false;
        }
        if (loadedOk) {
            lastSyncedSceneIndex = menuConfig.sceneIndex;
        } else {
            // The preset file is gone/unreadable: fall back to the first built-in and rescan so
            // the combo and the scene fields agree again instead of showing the dead selection.
            RefreshUserPresets();
            menuConfig.sceneIndex = 0;
            menuConfig.scene = BuildScene(0, ScenePresets[0].spawn.gridCountX, ScenePresets[0].spawn.gridCountY);
            lastSyncedSceneIndex = 0;
        }
    } else if (gridXEdited || gridYEdited) {
        // Only the fields that actually depend on the grid count are updated, so hand edits to
        // everything else (stiffness, gravity, boundary, name, ...) survive a grid resize. For a
        // built-in, "depends on the grid count" is found by diffing the factory scene at the old
        // count against the one at the new count; a user preset has no recalibration formula, so
        // only the count itself changes.
        Scene& cur = menuConfig.scene;
        if (CombinedIsBuiltin(menuConfig.sceneIndex)) {
            const Scene oldDefault = BuildScene(menuConfig.sceneIndex, cur.spawn.gridCountX, cur.spawn.gridCountY);
            const Scene newDefault = BuildScene(menuConfig.sceneIndex, gridX, gridY);
            auto follow = [](auto& dst, const auto& oldV, const auto& newV) { if (oldV != newV) dst = newV; };
            follow(cur.boundary.halfWidthPx,       oldDefault.boundary.halfWidthPx,       newDefault.boundary.halfWidthPx);
            follow(cur.boundary.halfHeightPx,      oldDefault.boundary.halfHeightPx,      newDefault.boundary.halfHeightPx);
            follow(cur.physics.force.x,            oldDefault.physics.force.x,            newDefault.physics.force.x);
            follow(cur.physics.force.y,            oldDefault.physics.force.y,            newDefault.physics.force.y);
            follow(cur.physics.substeps,           oldDefault.physics.substeps,           newDefault.physics.substeps);
            follow(cur.gravity.particleMass,       oldDefault.gravity.particleMass,       newDefault.gravity.particleMass);
            follow(cur.gravity.softening,          oldDefault.gravity.softening,          newDefault.gravity.softening);
            follow(cur.particles.circleRadius,     oldDefault.particles.circleRadius,     newDefault.particles.circleRadius);
            follow(cur.particles.smoothingRadius,  oldDefault.particles.smoothingRadius,  newDefault.particles.smoothingRadius);
            follow(cur.particles.targetDensity,    oldDefault.particles.targetDensity,    newDefault.particles.targetDensity);
            follow(cur.particles.stiffness,        oldDefault.particles.stiffness,        newDefault.particles.stiffness);
        }
        cur.spawn.gridCountX = gridX;
        cur.spawn.gridCountY = gridY;
    }

    ImGui::Checkbox("Circular spawn", &menuConfig.scene.spawn.circular);
    ImGui::TextDisabled("%d particles", menuConfig.scene.spawn.gridCountX * menuConfig.scene.spawn.gridCountY);
    ImGui::TextDisabled("Real-Time Limit: %d particles",
                         menuConfig.scene.realTimeLimitX * menuConfig.scene.realTimeLimitY);


    EndCard();
    }

    Widgets::Caption("LAYOUT");
    if (ImGui::CollapsingHeader("Spawn"))    DrawSpawnSection(menuConfig.scene.spawn, w);
    if (ImGui::CollapsingHeader("Boundary")) DrawBoundarySection(menuConfig.scene.boundary, w);

    if (ImGui::CollapsingHeader("Presets")) {
        static char presetSaveName[128] = "";
        FormRow(w, "Name");
        ImGui::InputText("##Name", presetSaveName, sizeof(presetSaveName));
        if (ImGui::Button("Save Current as Preset", ImVec2(w, 0.f)) && std::strlen(presetSaveName) > 0) {
            const auto dir = ExecutableDir() / "presets";
            std::error_code ec;
            std::filesystem::create_directories(dir, ec);
            if (!ec) {
                std::string safeName = SanitizeFilename(presetSaveName, "preset");
                std::string displayName = presetSaveName;
                // Re-saving over the preset currently selected is an intentional in-place update;
                // anything else that would land on an existing file gets a numeric suffix instead
                // of silently clobbering it.
                const bool selectedIsUser = !CombinedIsBuiltin(menuConfig.sceneIndex) &&
                    menuConfig.sceneIndex - static_cast<int>(ScenePresets.size()) < static_cast<int>(userPresets.size());
                const bool overwritingSelected = selectedIsUser &&
                    userPresets[menuConfig.sceneIndex - static_cast<int>(ScenePresets.size())].path ==
                        dir / (safeName + ".scenepreset");
                if (!overwritingSelected) {
                    const std::string unique = RunNaming::EnsureUniqueBaseName(safeName, { { dir, ".scenepreset" } });
                    if (unique != safeName) {
                        displayName += unique.substr(safeName.size()); // the "_2", "_3" ... suffix
                        safeName = unique;
                    }
                }
                const auto presetPath = dir / (safeName + ".scenepreset");
                menuConfig.scene.name = displayName;
                SaveScenePreset(presetPath, menuConfig.scene);
                RefreshUserPresets();

                // ScanUserPresets sorts by name, so re-scanning can shift every user preset's
                // index — re-point sceneIndex (and lastSyncedSceneIndex, to match, so this
                // doesn't itself trigger a reseed) at the just-saved file instead of leaving it
                // aimed at whatever now sits at the old index.
                for (int i = 0; i < static_cast<int>(userPresets.size()); i++) {
                    if (userPresets[i].path == presetPath) {
                        menuConfig.sceneIndex = static_cast<int>(ScenePresets.size()) + i;
                        lastSyncedSceneIndex  = menuConfig.sceneIndex;
                        break;
                    }
                }
            }
        }
        ImGui::TextDisabled("Saved presets appear in the Scene list above.");
    }


    ImGui::PopTextWrapPos();
}

static void DrawPhysicsTab() {
    const float w = ImGui::GetContentRegionAvail().x;
    ImGui::PushTextWrapPos(0.f);
    if (ImGui::CollapsingHeader("Physics", ImGuiTreeNodeFlags_DefaultOpen))         DrawPhysicsSection(menuConfig.scene.physics, w);
    if (ImGui::CollapsingHeader("Gravity"))                                         DrawGravitySection(menuConfig.scene.gravity, w);
    if (ImGui::CollapsingHeader("Particles / EOS", ImGuiTreeNodeFlags_DefaultOpen)) DrawParticlesSection(menuConfig.scene.particles, w);
    // Always available (not tied to recording) so velocity/density gradient coloring can be set up
    // before a run starts, including for batch renders that never visit the live view.
    if (ImGui::CollapsingHeader("Color By"))                                        DrawColorByControls(w);
    ImGui::PopTextWrapPos();
}

static void DrawOutputTab() {
    const float w = ImGui::GetContentRegionAvail().x;
    ImGui::PushTextWrapPos(0.f);
    ImGui::Checkbox("Record to video", &menuConfig.run.record);
    if (menuConfig.run.record) {
        FormRow(w, "Title (optional label)");
        ImGui::InputText("##Title (optional label)", menuConfig.run.title, sizeof(menuConfig.run.title));
        FormRow(w, "FPS");
        ImGui::InputInt("##FPS", &menuConfig.run.fps);
        menuConfig.run.fps = std::clamp(menuConfig.run.fps, 1, 240);
        FormRow(w, "Length (s, 0 = unlimited)");
        ImGui::InputFloat("##Length (s, 0 = unlimited)", &menuConfig.run.lengthSeconds, 1.0f, 10.0f, "%.0f");
        menuConfig.run.lengthSeconds = std::max(menuConfig.run.lengthSeconds, 0.f);
        ImGui::TextDisabled("Saved to recordings/<generated name>.mp4 (requires ffmpeg on PATH)");

        ImGui::Checkbox("Render as fast as possible (no live playback)", &menuConfig.run.batchRender);
        if (menuConfig.run.batchRender)
            ImGui::TextDisabled("Computes and saves the video directly at FPS/Length above, taking\n"
                                 "however long that takes to compute instead of that many real\n"
                                 "seconds. Cancel button (or Space) stops it early.");
    }

    ImGui::Separator();
    ImGui::Checkbox("Save data to CSV", &menuConfig.run.saveData);
    if (menuConfig.run.saveData) {
        FormRow(w, "Data title (optional label)");
        ImGui::InputText("##Data title (optional label)", menuConfig.run.dataTitle, sizeof(menuConfig.run.dataTitle));
        FormRow(w, "Sample rate (Hz)");
        ImGui::InputInt("##Sample rate (Hz)", &menuConfig.run.dataRate);
        menuConfig.run.dataRate = std::clamp(menuConfig.run.dataRate, 1, 240);
        FormRow(w, "Save length (s, 0 = unlimited)");
        ImGui::InputFloat("##Save length (s, 0 = unlimited)", &menuConfig.run.dataLengthSeconds, 1.0f, 10.0f, "%.0f");
        menuConfig.run.dataLengthSeconds = std::max(menuConfig.run.dataLengthSeconds, 0.f);

        ImGui::TextUnformatted("Fields");
        ImGui::Checkbox("Position", &menuConfig.run.dataPosition);
        ImGui::SameLine();
        ImGui::Checkbox("Velocity", &menuConfig.run.dataVelocity);
        ImGui::SameLine();
        ImGui::Checkbox("Speed", &menuConfig.run.dataSpeed);
        ImGui::Checkbox("Density", &menuConfig.run.dataDensity);
        ImGui::SameLine();
        ImGui::Checkbox("Pressure", &menuConfig.run.dataPressure);

        ImGui::TextDisabled("Saved to data/<generated name>.csv (one row per particle per sample)");
    }

    ImGui::Separator();
    ImGui::Checkbox("Save channel velocity profile", &menuConfig.run.saveProfile);
    if (menuConfig.run.saveProfile) {
        FormRow(w, "Profile title (optional label)");
        ImGui::InputText("##Profile title (optional label)", menuConfig.run.profileTitle, sizeof(menuConfig.run.profileTitle));
        FormRow(w, "Profile rate (Hz, sim time)");
        ImGui::InputInt("##Profile rate (Hz, sim time)", &menuConfig.run.profileRate);
        menuConfig.run.profileRate = std::clamp(menuConfig.run.profileRate, 1, 60);
        FormRow(w, "Profile length (sim s, 0 = unlimited)");
        ImGui::InputFloat("##Profile length (sim s, 0 = unlimited)", &menuConfig.run.profileLengthSeconds, 1.0f, 10.0f, "%.0f");
        menuConfig.run.profileLengthSeconds = std::max(menuConfig.run.profileLengthSeconds, 0.f);
        FormRow(w, "Bins across channel (0 = one per row)");
        ImGui::InputInt("##Bins across channel (0 = one per row)", &menuConfig.run.profileBins);
        menuConfig.run.profileBins = std::max(menuConfig.run.profileBins, 0);
        ImGui::TextDisabled("Saved to data/<name>_profile.csv + _frames.csv: per-bin v_x across the "
                             "channel height, plus per-sample density/velocity stats.");
    }


    ImGui::PopTextWrapPos();
}

static void DrawQueueTab() {
    const float w = ImGui::GetContentRegionAvail().x;
    ImGui::PushTextWrapPos(0.f);
    ImGui::TextUnformatted("Run Queue");
    if (ImGui::Button("Add to Queue", ImVec2(w, 0.f))) {
        if (HasDeterministicEnd(menuConfig.run))
            runQueue.push_back({ menuConfig.scene, menuConfig.run });
    }
    if (!HasDeterministicEnd(menuConfig.run))
        ImGui::TextColored(ImVec4(1.f, 0.4f, 0.4f, 1.f),
                            "Enable recording, data-save or profile with a nonzero length first.");

    int removeIndex = -1;
    for (int i = 0; i < static_cast<int>(runQueue.size()); i++) {
        ImGui::PushID(i);
        ImGui::TextDisabled("%d.", i + 1);
        ImGui::SameLine();
        ImGui::Text("%s (%dx%d)%s%s%s", runQueue[i].scene.name.c_str(),
                    runQueue[i].scene.spawn.gridCountX, runQueue[i].scene.spawn.gridCountY,
                    runQueue[i].run.record      ? " [video]"   : "",
                    runQueue[i].run.saveData    ? " [data]"    : "",
                    runQueue[i].run.saveProfile ? " [profile]" : "");
        ImGui::SameLine();
        if (ImGui::SmallButton("Remove")) removeIndex = i;
        ImGui::PopID();
    }
    if (removeIndex >= 0) runQueue.erase(runQueue.begin() + removeIndex);

    ImGui::BeginDisabled(runQueue.empty());
    if (ImGui::Button("Run Queue", ImVec2(w, 0.f))) StartQueue();
    ImGui::EndDisabled();


    ImGui::PopTextWrapPos();
}

static void DrawSetupPanel() {
    const ImGuiStyle& style = ImGui::GetStyle();
    // Sticky footer: the tab content above scrolls, the particle readout and Start button never do.
    const float buttonH = ImGui::GetFrameHeight() * 1.5f;
    const float footerH = buttonH + ImGui::GetTextLineHeight() + style.ItemSpacing.y * 3.f;

    if (ImGui::BeginTabBar("##setupTabs")) {
        struct Tab { const char* name; void (*draw)(); };
        const Tab tabs[] = { { "Scene", DrawSceneTab }, { "Physics", DrawPhysicsTab },
                             { "Output", DrawOutputTab }, { "Queue", DrawQueueTab } };
        for (const Tab& t : tabs) {
            if (!ImGui::BeginTabItem(t.name)) continue;
            ImGui::BeginChild("##tabScroll", ImVec2(0.f, -footerH));
            ImGui::Spacing();
            t.draw();
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::Separator();
    const int count = menuConfig.scene.spawn.gridCountX * menuConfig.scene.spawn.gridCountY;
    ImGui::TextDisabled("%d particles  |  real-time limit %d", count,
                         menuConfig.scene.realTimeLimitX * menuConfig.scene.realTimeLimitY);
    if (Widgets::PrimaryButton("Start Simulation", ImVec2(-FLT_MIN, buttonH)))
        StartSimulationFromMenu();
}

// ---- Live view controls (Running state) -----------------------------------------------------

static void DrawLivePanel() {
    ImGui::PushTextWrapPos(0.f);

    Widgets::Caption("RUN");
    BeginCard("##runCard");
    ImGui::Text("Simulated time   %.2f s", simTime);
    ImGui::Text("Particles        %d", static_cast<int>(objects.size()));
    ImGui::TextDisabled("Space: pause  |  Right: step  |  Left: step x10");
    EndCard();

    Widgets::Caption("VIEW");
    BeginCard("##viewCard");
    ImGui::Checkbox("Show event areas", &showEventAreas);
    if (ActiveScene.boundary.periodicX) {
        ImGui::Checkbox("Follow flow (camera)", &cameraFollow);
        ImGui::TextDisabled("Cancels bulk drift so the profile's relative motion reads as motion.");
    }
    EndCard();

    Widgets::Caption("COLOR BY");
    BeginCard("##colorCard");
    DrawColorByControls(ImGui::GetContentRegionAvail().x);
    EndCard();

    ImGui::PopTextWrapPos();
}

// ---- Render progress (Rendering state) ------------------------------------------------------

// Safe to draw now that VulkanContext::RenderFrame captures the video frame in a first pass and
// only draws ImGui in a second one on top — this never reaches the recorded video.
static void DrawRenderingPanel() {
    const float w = ImGui::GetContentRegionAvail().x;
    ImGui::PushTextWrapPos(0.f);

    if (queueRunningIndex >= 0)
        ImGui::Text("Queue: %d / %d", queueRunningIndex + 1, static_cast<int>(runQueue.size()));
    ImGui::TextUnformatted(ActiveScene.name.c_str());
    ImGui::Spacing();

    // A data-only queue entry (no video) never sets app.IsRecording() — track its own elapsed
    // time via the data-save sample count instead, same source the "DATA" chip uses.
    if (app.IsRecording()) {
        const float recorded = app.RecordedSeconds();
        if (activeRun.lengthSeconds > 0.f) {
            ImGui::Text("%.1f / %.1f seconds rendered", recorded, activeRun.lengthSeconds);
            ImGui::ProgressBar(std::clamp(recorded / activeRun.lengthSeconds, 0.f, 1.f), ImVec2(w, 0.f));
        } else {
            ImGui::Text("%.1f seconds rendered (unlimited length)", recorded);
        }
    } else if (dataRecorder.IsActive()) {
        const float dataSeconds = dataRateActive > 0 ? static_cast<float>(dataSavedFrames) / dataRateActive : 0.f;
        if (activeRun.dataLengthSeconds > 0.f) {
            ImGui::Text("%.1f / %.1f seconds sampled", dataSeconds, activeRun.dataLengthSeconds);
            ImGui::ProgressBar(std::clamp(dataSeconds / activeRun.dataLengthSeconds, 0.f, 1.f), ImVec2(w, 0.f));
        } else {
            ImGui::Text("%.1f seconds sampled (unlimited length)", dataSeconds);
        }
    } else if (profileLogger.IsActive()) {
        const float t = static_cast<float>(simTime);
        if (activeRun.profileLengthSeconds > 0.f) {
            ImGui::Text("%.1f / %.1f sim seconds profiled", t, activeRun.profileLengthSeconds);
            ImGui::ProgressBar(std::clamp(t / activeRun.profileLengthSeconds, 0.f, 1.f), ImVec2(w, 0.f));
        } else {
            ImGui::Text("%.1f sim seconds profiled (unlimited length)", t);
        }
    }
    ImGui::Spacing();
    ImGui::TextDisabled("Rendering as fast as possible, straight to video - no live playback follows.");
    ImGui::Spacing();
    if (Widgets::DangerButton("Cancel", ImVec2(w, 0.f))) ReturnToMenu();

    ImGui::PopTextWrapPos();
}

static void DrawLeftPanel() {
    const char* title = appState == AppState::Menu    ? "Setup###LeftPanel"
                      : appState == AppState::Running ? "Simulation###LeftPanel"
                                                      : "Rendering###LeftPanel";
    if (ImGui::Begin(title)) {
        switch (appState) {
            case AppState::Menu:      DrawSetupPanel();     break;
            case AppState::Running:   DrawLivePanel();      break;
            case AppState::Rendering: DrawRenderingPanel(); break;
        }
    }
    ImGui::End();
}

// ---- Events panel (right) -------------------------------------------------------------------

// Shape/mode/push/duration used for areas created by dragging in the sim view or "+ New event".
static void DrawEventDefaults(float w) {
    int shape = static_cast<int>(placeEventTemplate.shape), mode = static_cast<int>(placeEventTemplate.mode);
    FormRow(w, "Shape");
    if (ImGui::Combo("##defShape", &shape, EventShapeNames, 2)) placeEventTemplate.shape = static_cast<EventShape>(shape);
    FormRow(w, "Mode");
    if (ImGui::Combo("##defMode", &mode, EventModeNames, 3)) placeEventTemplate.mode = static_cast<EventMode>(mode);
    const bool once = placeEventTemplate.mode == EventMode::Once;
    if (placeEventTemplate.mode == EventMode::Field) {
        int kind = static_cast<int>(placeEventTemplate.fieldKind);
        FormRow(w, "Field");
        if (ImGui::Combo("##defFieldKind", &kind, FieldKindNames, 3)) placeEventTemplate.fieldKind = static_cast<FieldKind>(kind);
        FormRow(w, "Strength (px/s^2)");
        ImGui::InputFloat("##defStrength", &placeEventTemplate.strength);
        ImGui::Checkbox("Fade to zero at the edge##def", &placeEventTemplate.fade);
    } else {
        FormRow(w, once ? "Kick X (px/s)" : "Accel X (px/s^2)");
        ImGui::InputFloat("##defX", &placeEventTemplate.accel.x);
        FormRow(w, once ? "Kick Y (px/s)" : "Accel Y (px/s^2)");
        ImGui::InputFloat("##defY", &placeEventTemplate.accel.y);
    }
    if (!once) {
        FormRow(w, "Duration (s, 0 = unlimited)");
        ImGui::InputFloat("##defDur", &placeEventTemplate.duration);
        placeEventTemplate.duration = std::max(placeEventTemplate.duration, 0.f);
    }
}

static void RemoveEventAt(std::vector<SceneEvent>& events, int index) {
    if (index < 0 || index >= static_cast<int>(events.size())) return;
    events.erase(events.begin() + index);
    if (appState != AppState::Menu && index < static_cast<int>(eventFired.size()))
        eventFired.erase(eventFired.begin() + index);
    selectedEvent = -1;
}

// Result of the async "Browse..." dialog; SDL may call back from another thread, so hand it over
// through a mutex and let the UI pick it up next frame.
static std::mutex  pickedImageMutex;
static std::string pickedImagePath;

static void SDLCALL OnImagePicked(void*, const char* const* files, int) {
    if (files && files[0]) {
        std::lock_guard<std::mutex> lock(pickedImageMutex);
        pickedImagePath = files[0];
    }
}

static void DrawColorPatternSection(SceneColorPattern& p, float w) {
    {
        std::lock_guard<std::mutex> lock(pickedImageMutex);
        if (!pickedImagePath.empty()) {
            p.imagePath = pickedImagePath;
            p.kind      = ColorPatternKind::Image;
            pickedImagePath.clear();
        }
    }

    static const char* KindNames[] = { "None", "Vertical stripes", "Horizontal stripes", "Checkerboard",
                                       "Rings", "Gradient (left to right)", "Image" };
    int kind = static_cast<int>(p.kind);
    FormRow(w, "Pattern");
    if (ImGui::Combo("##patternKind", &kind, KindNames, IM_ARRAYSIZE(KindNames)))
        p.kind = static_cast<ColorPatternKind>(kind);
    if (p.kind == ColorPatternKind::None) return;

    const float swatchSize = ImGui::GetFrameHeight();
    if (p.kind != ColorPatternKind::Image) {
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Color A");
        ImGui::SameLine(w - swatchSize);
        ImGui::ColorEdit4("##patternA", &p.colorA.r, ImGuiColorEditFlags_NoInputs);
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Color B");
        ImGui::SameLine(w - swatchSize);
        ImGui::ColorEdit4("##patternB", &p.colorB.r, ImGuiColorEditFlags_NoInputs);
    }
    if (p.kind != ColorPatternKind::Gradient && p.kind != ColorPatternKind::Image) {
        FormRow(w, "Cell size (px)");
        ImGui::InputFloat("##patternCell", &p.cellSize);
        p.cellSize = std::max(p.cellSize, 4.f);
    } else if (p.kind == ColorPatternKind::Image) {
        char buf[512];
        std::snprintf(buf, sizeof(buf), "%s", p.imagePath.c_str());
        FormRow(w, "Image file");
        if (ImGui::InputText("##patternImage", buf, sizeof(buf))) p.imagePath = buf;
        if (ImGui::Button("Browse...")) {
            static const SDL_DialogFileFilter filters[] = { { "Images (PNG, BMP)", "png;bmp" } };
            SDL_ShowOpenFileDialog(OnImagePicked, nullptr, nullptr, filters, 1, nullptr, false);
        }
        ImGui::TextDisabled("%s", PatternImageStatus(p).c_str());
    }
}

// Setup-only counterpart of the events list: persistent-color areas (see SceneColorRegion).
static void DrawColorRegionsPanel() {
    {
        const float pw = ImGui::GetContentRegionAvail().x;
        Widgets::Caption("PATTERN");
        DrawColorPatternSection(menuConfig.scene.colorPattern, pw);
        ImGui::Spacing();
        Widgets::Caption("AREAS");
    }
    auto& regions = menuConfig.scene.colorRegions;
    if (selectedRegion >= static_cast<int>(regions.size())) selectedRegion = -1;
    const float w = ImGui::GetContentRegionAvail().x;

    ImGui::TextDisabled("Drag in the view to draw an area. Drag an area to move it, its handles to\n"
                         "resize it (hold Alt to resize from the center). Del removes the selected\n"
                         "area. Particles spawned inside keep its color for the whole run.");
    ImGui::Spacing();

    if (Widgets::PrimaryButton("+ New area at center", ImVec2(w, 0.f))) {
        SceneColorRegion r = placeRegionTemplate;
        r.cx = 0.f; r.cy = 0.f;
        regions.push_back(r);
        selectedRegion = static_cast<int>(regions.size()) - 1;
    }

    if (ImGui::CollapsingHeader("New area defaults")) {
        static const char* ShapeNames[] = { "Circle", "Rect" };
        int shape = static_cast<int>(placeRegionTemplate.shape);
        FormRow(w, "Shape");
        if (ImGui::Combo("##defRegionShape", &shape, ShapeNames, 2)) placeRegionTemplate.shape = static_cast<EventShape>(shape);
        const float swatchSize = ImGui::GetFrameHeight();
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Color");
        ImGui::SameLine(w - swatchSize);
        ImGui::ColorEdit4("##defRegionColor", &placeRegionTemplate.color.r, ImGuiColorEditFlags_NoInputs);
    }

    Widgets::Caption("PERSISTENT COLOR");
    if (regions.empty()) ImGui::TextDisabled("None yet.");
    for (int i = 0; i < static_cast<int>(regions.size()); i++) {
        const SceneColorRegion& r = regions[i];
        ImGui::PushID(i);
        char label[96];
        std::snprintf(label, sizeof(label), "      %d   %s", i + 1, r.shape == EventShape::Circle ? "Circle" : "Rect");
        const ImVec2 rowPos = ImGui::GetCursorScreenPos();
        if (ImGui::Selectable(label, i == selectedRegion)) selectedRegion = i;
        ImGui::GetWindowDrawList()->AddCircleFilled(
            ImVec2(rowPos.x + ImGui::GetFontSize() * 0.7f, rowPos.y + ImGui::GetFrameHeight() * 0.5f - 2.f),
            ImGui::GetFontSize() * 0.28f, ImGui::ColorConvertFloat4ToU32(ImVec4(r.color.r, r.color.g, r.color.b, 1.f)));
        ImGui::PopID();
    }

    if (selectedRegion >= 0) {
        SceneColorRegion& r = regions[selectedRegion];
        char title[32];
        std::snprintf(title, sizeof(title), "AREA %d", selectedRegion + 1);
        Widgets::Caption(title);
        BeginCard("##regionInspector");
        DrawColorRegionFields(r, ImGui::GetContentRegionAvail().x);
        ImGui::Spacing();
        const float bw = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) / 2.f;
        if (ImGui::Button("Duplicate", ImVec2(bw, 0.f))) {
            SceneColorRegion copy = r;
            copy.cx += 30.f; copy.cy -= 30.f;
            regions.push_back(copy);
            selectedRegion = static_cast<int>(regions.size()) - 1;
        } else {
            ImGui::SameLine();
            if (Widgets::DangerButton("Delete", ImVec2(bw, 0.f))) {
                regions.erase(regions.begin() + selectedRegion);
                selectedRegion = -1;
            }
        }
        EndCard();
    }
}

// Own window, docked as a tab beside "Events" (first appearance only; afterwards the user's layout wins).
static void DrawColorsWindow() {
    colorsTabActive = false;
    if (appState != AppState::Menu) return;
    if (const ImGuiWindow* rw = ImGui::FindWindowByName(Workspace::RightPanelId); rw && rw->DockId)
        ImGui::SetNextWindowDockID(rw->DockId, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Persistent Color###ColorsPanel")) {
        colorsTabActive = true;
        ImGui::PushTextWrapPos(0.f);
        DrawColorRegionsPanel();
        ImGui::PopTextWrapPos();
    }
    ImGui::End();
}

static void DrawEventsPanel() {
    if (ImGui::Begin("Events###RightPanel")) {
        const bool locked = appState == AppState::Rendering;
        const bool live   = appState != AppState::Menu;
        ImGui::BeginDisabled(locked);
        ImGui::PushTextWrapPos(0.f);

        auto& events = UiEvents();
        if (selectedEvent >= static_cast<int>(events.size())) selectedEvent = -1;
        const float w = ImGui::GetContentRegionAvail().x;

        ImGui::TextDisabled("Drag in the view to draw an area. Drag an area to move it, its handles to\n"
                             "resize it (each side/corner; hold Alt to resize from the center) or\n"
                             "aim the push. Del removes the selected event.");
        ImGui::Spacing();

        if (Widgets::PrimaryButton("+ New event at center", ImVec2(w, 0.f))) {
            SceneEvent e = placeEventTemplate;
            e.cx = 0.f; e.cy = 0.f;
            if (live) e.startTime = static_cast<float>(simTime);
            events.push_back(e);
            if (live) eventFired.resize(events.size(), 0);
            selectedEvent = static_cast<int>(events.size()) - 1;
        }

        if (ImGui::CollapsingHeader("New event defaults")) DrawEventDefaults(w);

        Widgets::Caption("EVENTS");
        if (events.empty()) ImGui::TextDisabled("None yet.");
        for (int i = 0; i < static_cast<int>(events.size()); i++) {
            const SceneEvent& e = events[i];
            ImGui::PushID(i);
            char label[96];
            std::snprintf(label, sizeof(label), "      %d   %s | %s", i + 1,
                          e.shape == EventShape::Circle ? "Circle" : "Rect",
                          e.mode == EventMode::Once ? "one-shot kick" : e.mode == EventMode::Field ? "vector field" : "continuous push");
            const ImVec2 rowPos = ImGui::GetCursorScreenPos();
            if (ImGui::Selectable(label, i == selectedEvent)) selectedEvent = i;
            const ImVec4 col = e.mode == EventMode::Once ? Theme::Amber : e.mode == EventMode::Field ? Theme::Teal : Theme::Violet;
            ImGui::GetWindowDrawList()->AddCircleFilled(
                ImVec2(rowPos.x + ImGui::GetFontSize() * 0.7f, rowPos.y + ImGui::GetFrameHeight() * 0.5f - 2.f),
                ImGui::GetFontSize() * 0.28f, Theme::U32(col));
            ImGui::PopID();
        }

        if (selectedEvent >= 0) {
            SceneEvent& e = events[selectedEvent];
            char title[32];
            std::snprintf(title, sizeof(title), "EVENT %d", selectedEvent + 1);
            Widgets::Caption(title);
            BeginCard("##eventInspector");
            DrawEventFields(e, ImGui::GetContentRegionAvail().x);
            ImGui::Spacing();
            const float bw = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x * 2.f) / 3.f;
            ImGui::BeginDisabled(!live);
            if (ImGui::Button("Fire now", ImVec2(bw, 0.f))) {
                e.startTime = static_cast<float>(simTime);
                if (selectedEvent < static_cast<int>(eventFired.size())) eventFired[selectedEvent] = 0;
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button("Duplicate", ImVec2(bw, 0.f))) {
                SceneEvent copy = e;
                copy.cx += 30.f; copy.cy -= 30.f;
                events.push_back(copy);
                if (live) eventFired.resize(events.size(), 0);
                selectedEvent = static_cast<int>(events.size()) - 1;
            } else {
                ImGui::SameLine();
                if (Widgets::DangerButton("Delete", ImVec2(bw, 0.f))) RemoveEventAt(events, selectedEvent);
            }
            EndCard();
        }

        ImGui::PopTextWrapPos();
        ImGui::EndDisabled();
    }
    ImGui::End();
}

// ---- Sim view: scene preview (setup) + event gizmos -----------------------------------------

static void DrawSimOverlay(const Workspace::Frame& frame) {
    const ImGuiIO& io = ImGui::GetIO();
    const float fb = std::max(io.DisplayFramebufferScale.x, 0.01f); // framebuffer pixels per ImGui point

    // The sim is rendered into the free center area (see VulkanContext::SetSceneViewport); the
    // renderer ignores this while a recording is active and reports the window center instead.
    app.SetSceneViewport(frame.centerMin.x * fb, frame.centerMin.y * fb,
                         (frame.centerMax.x - frame.centerMin.x) * fb,
                         (frame.centerMax.y - frame.centerMin.y) * fb);
    float cxPx = 0.f, cyPx = 0.f;
    app.SceneCenterPx(cxPx, cyPx);

    EventGizmo::View view;
    view.origin  = ImVec2(cxPx / fb, cyPx / fb);
    view.scale   = 1.f / fb;
    view.clipMin = frame.centerMin;
    view.clipMax = frame.centerMax;

    if (appState == AppState::Menu) {
        // Pre-run preview: the container and the spawn block, so events can be placed visually.
        const Scene& s = menuConfig.scene;
        const float halfW = s.boundary.halfWidthPx  > 0.f ? s.boundary.halfWidthPx  : app.RecordingHalfWidth()  * s.boundary.widthFrac;
        const float halfH = s.boundary.halfHeightPx > 0.f ? s.boundary.halfHeightPx : app.RecordingHalfHeight() * s.boundary.heightFrac;
        ImDrawList* dl = ImGui::GetBackgroundDrawList();
        dl->PushClipRect(frame.centerMin, frame.centerMax, true);
        const ImVec2 a(view.origin.x - halfW * view.scale, view.origin.y - halfH * view.scale);
        const ImVec2 b(view.origin.x + halfW * view.scale, view.origin.y + halfH * view.scale);
        dl->AddRectFilled(a, b, Theme::U32(Theme::WithAlpha(Theme::Panel, 0.45f)), 14.f);
        dl->AddRect(a, b, Theme::U32(Theme::WithAlpha(Theme::TextDim, 0.8f)), 14.f, 0, 3.f);

        const float spacing = s.particles.circleRadius * 3.f * s.spawn.spacingScale;
        const float cx = s.spawn.offsetXFrac * halfW;
        const ImU32 fill = Theme::U32(Theme::WithAlpha(Theme::Accent, 0.22f));
        const ImU32 line = Theme::U32(Theme::WithAlpha(Theme::Accent, 0.7f));
        if (s.spawn.circular) {
            const float r = spacing * std::sqrt(static_cast<float>(s.spawn.gridCountX * s.spawn.gridCountY) / 3.14159265f);
            const ImVec2 c(view.origin.x + cx * view.scale, view.origin.y);
            dl->AddCircleFilled(c, r * view.scale, fill, 96);
            dl->AddCircle(c, r * view.scale, line, 96, 2.f);
        } else {
            const float bw = s.spawn.gridCountX * spacing * 0.5f, bh = s.spawn.gridCountY * spacing * 0.5f;
            const ImVec2 pa(view.origin.x + (cx - bw) * view.scale, view.origin.y - bh * view.scale);
            const ImVec2 pb(view.origin.x + (cx + bw) * view.scale, view.origin.y + bh * view.scale);
            dl->AddRectFilled(pa, pb, fill, 8.f);
            dl->AddRect(pa, pb, line, 8.f, 0, 2.f);
        }
        if (s.colorPattern.kind != ColorPatternKind::None) {
            // Coarse sample grid over the spawn block, so the pattern shows before the run starts.
            const float bw = s.spawn.circular ? spacing * std::sqrt(static_cast<float>(s.spawn.gridCountX * s.spawn.gridCountY) / 3.14159265f)
                                              : s.spawn.gridCountX * spacing * 0.5f;
            const float bh = s.spawn.circular ? bw : s.spawn.gridCountY * spacing * 0.5f;
            const PatternSampler sampler(s.colorPattern, cx - bw, -bh, cx + bw, bh);
            const float step = std::max(std::max(bw, bh) / 70.f, 2.f);
            for (float wy = -bh + step * 0.5f; wy < bh; wy += step)
                for (float wx = cx - bw + step * 0.5f; wx < cx + bw; wx += step) {
                    if (s.spawn.circular && std::hypot(wx - cx, wy) > bw) continue;
                    Color pc;
                    if (!sampler.Sample(wx, wy, pc)) continue;
                    const ImVec2 q(view.origin.x + wx * view.scale, view.origin.y - wy * view.scale);
                    const float hs = step * 0.5f * view.scale + 0.5f;
                    dl->AddRectFilled(ImVec2(q.x - hs, q.y - hs), ImVec2(q.x + hs, q.y + hs),
                                      ImGui::ColorConvertFloat4ToU32(ImVec4(pc.r, pc.g, pc.b, 0.85f)));
                }
        }
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%d particles", s.spawn.gridCountX * s.spawn.gridCountY);
        dl->AddText(ImVec2(a.x + 14.f, a.y + 10.f), Theme::U32(Theme::TextDim), buf);
        dl->PopClipRect();
    }

    if (appState == AppState::Menu || (appState == AppState::Running && showEventAreas)) {
        if (appState == AppState::Running && ActiveScene.boundary.periodicX && cameraFollow) {
            view.cameraOffsetX = cameraOffsetX;
            view.wrapHalfWidth = ScreenHalfWidth();
        }
        // Setup only: persistent-color areas, edited through SceneEvent proxies so the same gizmo can
        // draw/drag them. Only interactive while the "Persistent Color" tab is open (the events gizmo
        // below is then draw-only), and always drawn so they stay visible.
        const bool editColors = appState == AppState::Menu && colorsTabActive;
        if (appState == AppState::Menu) {
            auto& regions = menuConfig.scene.colorRegions;
            std::vector<SceneEvent> proxy(regions.size());
            std::vector<ImVec4>     tints(regions.size());
            for (size_t i = 0; i < regions.size(); i++) {
                const SceneColorRegion& rg = regions[i];
                proxy[i].shape = rg.shape;
                proxy[i].cx = rg.cx; proxy[i].cy = rg.cy;
                proxy[i].radius = rg.radius; proxy[i].halfW = rg.halfW; proxy[i].halfH = rg.halfH;
                tints[i] = ImVec4(rg.color.r, rg.color.g, rg.color.b, 1.f);
            }
            SceneEvent proxyDefaults;
            proxyDefaults.shape = placeRegionTemplate.shape;
            int dummySel = -1;
            EventGizmo::Context rc;
            rc.view        = view;
            rc.events      = &proxy;
            rc.selected    = editColors ? &selectedRegion : &dummySel;
            rc.defaults    = &proxyDefaults;
            rc.interactive = editColors;
            rc.showArrows  = false;
            rc.colors      = &tints;
            const EventGizmo::Result rr = EventGizmo::Update(rc);
            if (rr.deleted >= 0 && rr.deleted < static_cast<int>(regions.size()))
                regions.erase(regions.begin() + rr.deleted);
            for (size_t i = 0; i < regions.size() && i < proxy.size(); i++) {
                regions[i].cx = proxy[i].cx; regions[i].cy = proxy[i].cy;
                regions[i].radius = proxy[i].radius;
                regions[i].halfW = proxy[i].halfW; regions[i].halfH = proxy[i].halfH;
            }
            if (rr.created && proxy.size() > regions.size()) {
                const SceneEvent& p = proxy.back();
                SceneColorRegion n = placeRegionTemplate;
                n.shape = p.shape; n.cx = p.cx; n.cy = p.cy;
                n.radius = p.radius; n.halfW = p.halfW; n.halfH = p.halfH;
                regions.push_back(n);
            }
        }

        EventGizmo::Context ctx;
        ctx.view     = view;
        ctx.events   = &UiEvents();
        ctx.selected = &selectedEvent;
        ctx.defaults = &placeEventTemplate;
        ctx.fired    = &eventFired;
        ctx.simTime  = simTime;
        ctx.live     = appState == AppState::Running;
        ctx.interactive = !editColors;
        const EventGizmo::Result r = EventGizmo::Update(ctx);
        if (ctx.live) {
            eventFired.resize(ctx.events->size(), 0);
            if (r.deleted >= 0 && r.deleted < static_cast<int>(eventFired.size()))
                eventFired.erase(eventFired.begin() + r.deleted);
        }
    }
}

// ---- Top bar --------------------------------------------------------------------------------

static void DrawTopBarContents() {
    const ImGuiStyle& style = ImGui::GetStyle();
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(Theme::Accent, "STJARNA");
    ImGui::SameLine();

    switch (appState) {
        case AppState::Menu:      Widgets::Chip("SETUP", Theme::Accent); break;
        case AppState::Running:   paused ? Widgets::Chip("PAUSED", Theme::Amber) : Widgets::Chip("RUNNING", Theme::Teal); break;
        case AppState::Rendering: Widgets::Chip("RENDERING", Theme::Coral); break;
    }
    ImGui::SameLine();
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("%s", (appState == AppState::Menu ? menuConfig.scene.name : ActiveScene.name).c_str());

    char buf[48];
    if (app.IsRecording()) {
        std::snprintf(buf, sizeof(buf), "REC %.1fs", app.RecordedSeconds());
        ImGui::SameLine(); Widgets::Chip(buf, Theme::Coral);
    }
    if (dataRecorder.IsActive()) {
        const float dataSeconds = dataRateActive > 0 ? static_cast<float>(dataSavedFrames) / dataRateActive : 0.f;
        std::snprintf(buf, sizeof(buf), "DATA %.1fs", dataSeconds);
        ImGui::SameLine(); Widgets::Chip(buf, Theme::Accent);
    }
    if (profileLogger.IsActive()) {
        std::snprintf(buf, sizeof(buf), "PROFILE %.1fs", static_cast<float>(simTime));
        ImGui::SameLine(); Widgets::Chip(buf, Theme::Teal);
    }

    // Right-aligned action buttons.
    struct Btn { const char* label; int id; };
    std::vector<Btn> btns;
    if (appState == AppState::Running) {
        btns = { { paused ? "Resume" : "Pause", 0 }, { "Step", 1 }, { "Step x10", 2 }, { "Restart", 3 }, { "Menu", 4 } };
    } else if (appState == AppState::Rendering) {
        btns = { { "Cancel", 4 } };
    }
    btns.push_back({ "Reset layout", 5 });

    float total = style.ItemSpacing.x * static_cast<float>(btns.size() - 1);
    for (const Btn& b : btns) total += ImGui::CalcTextSize(b.label).x + style.FramePadding.x * 2.f;
    ImGui::SameLine(ImGui::GetWindowWidth() - total - style.WindowPadding.x);

    for (size_t i = 0; i < btns.size(); i++) {
        if (i > 0) ImGui::SameLine();
        if (ImGui::Button(btns[i].label)) {
            switch (btns[i].id) {
                case 0: TogglePause();       break;
                case 1: StepOnce();          break;
                case 2: StepBig();           break;
                case 3: RestartSimulation(); break;
                case 4: ReturnToMenu();      break;
                case 5: Workspace::RequestLayoutReset(); break;
            }
        }
    }
}

// The single UI callback (see main()): top bar, dockspace, the two panels, then the sim-view
// overlay — in that order, so the gizmo's io.WantCaptureMouse check already knows about this
// frame's panels.
static void DrawUi() {
    // A different run state or scene means the old selection no longer points at the same event.
    static AppState lastState = AppState::Menu;
    static int      lastScene = -2;
    if (appState != lastState || menuConfig.sceneIndex != lastScene) {
        selectedEvent = -1;
        lastState = appState;
        lastScene = menuConfig.sceneIndex;
    }

    if (Workspace::BeginTopBar()) DrawTopBarContents();
    Workspace::EndTopBar();
    const Workspace::Frame frame = Workspace::DockSpace();

    DrawLeftPanel();
    DrawEventsPanel();
    DrawColorsWindow();
    DrawSimOverlay(frame);
}

Object& CreateObject(float x, float y, Renderable r) {
    objects.push_back({ .pos = Vec2(x, y), .renderable = r });
    return objects.back();
}

void RemoveObject(size_t i) {
    objects[i] = objects.back();
    objects.pop_back();
}

// The simulation container's half-extents — the active scene's boundary fractions applied to the
// recording's own resolution while one is active (so the boundary/wall-bounce geometry can't
// drift out from under a captured video frame that's pinned to a fixed size — see
// VulkanContext::RecordingHalfWidth/Height), or the live window size otherwise. A scene that sets
// SceneBoundary::halfWidthPx/halfHeightPx gets that exact size instead, independent of both.
float ScreenHalfWidth() {
    const auto& b = ActiveScene.boundary;
    if (b.halfWidthPx > 0.f) return b.halfWidthPx;
    return frozenHalfW > 0.f ? frozenHalfW : app.RecordingHalfWidth() * b.widthFrac;
}
float ScreenHalfHeight() {
    const auto& b = ActiveScene.boundary;
    if (b.halfHeightPx > 0.f) return b.halfHeightPx;
    return frozenHalfH > 0.f ? frozenHalfH : app.RecordingHalfHeight() * b.heightFrac;
}

// Draws a black frame at the active scene's boundary, so a container narrower than the window
// (see SceneBoundary) reads as a wall instead of an invisible line partway across the screen.
// AddRectOutline's border band is drawn inward from the half-extent passed in, so the rect is
// inflated by BorderThickness here — that puts the band entirely outside the wall (half-extent
// stays exactly where particles clamp) instead of eating into the fluid's play area.
static void DrawBoundary(App& app) {
    constexpr float BorderThickness = 6.0f;
    constexpr Color BorderColor     = { 0.f, 0.f, 0.f, 1.f };
    const float hw = ScreenHalfWidth(), hh = ScreenHalfHeight();

    if (ActiveScene.boundary.periodicX) {
        // No side walls to draw when X wraps — just solid top/bottom bars marking the channel's
        // Y confinement. A border thickness bigger than the bar itself fills it solid rather than
        // drawing it as a hollow frame (see AddRectOutline).
        constexpr float FillSolid = 1e6f;
        const float barHalfHeight = BorderThickness * 0.5f;
        app.AddRectOutline(0.f,  hh + barHalfHeight, hw + BorderThickness, barHalfHeight, FillSolid, BorderColor);
        app.AddRectOutline(0.f, -hh - barHalfHeight, hw + BorderThickness, barHalfHeight, FillSolid, BorderColor);
    } else {
        app.AddRectOutline(0.f, 0.f, hw + BorderThickness, hh + BorderThickness, BorderThickness, BorderColor);
    }
}

// vt is the velocity component tangential to this axis's wall (e.g. vel.y when resolving the
// x-axis walls); when noSlipWalls is enabled it gets damped on contact instead of passing
// through untouched, so wall-adjacent particles drag against the boundary rather than free-slip.
static void resolveAxis(float& p, float& v, float& vt, float half, float r) {
    const auto& physics = ActiveScene.physics;
    bool hit = false;
    if (p - r < -half) { p = -half + r; if (v < 0.0f) { v *= -physics.restitution; hit = true; } }
    if (p + r >  half) { p =  half - r; if (v > 0.0f) { v *= -physics.restitution; hit = true; } }
    if (hit && physics.noSlipWalls) vt *= 1.0f - physics.friction;
}

// Wraps x into (-hw, hw], the same range periodicX teleporting keeps simulated positions in —
// used to fold the camera-follow display offset back into range regardless of how far it's
// accumulated, rather than only handling a single wrap like the simulated-position teleport does.
static float WrapX(float x, float hw) {
    const float span = 2.f * hw;
    if (span <= 0.f) return x;
    x = std::fmod(x + hw, span);
    if (x < 0.f) x += span;
    return x - hw;
}

// Integrates a single substep, reusing whatever force (particle.acc) the last Update() call
// computed — held constant across ActiveScene.physics.forceInterval substeps at a time to trade
// some staleness back for compute cost. acc is reset only when it's about to be recomputed
// (see the main loop), not here.
static void Integrate(float subDt) {
    const float hw = ScreenHalfWidth(), hh = ScreenHalfHeight();
    const bool  periodicX = ActiveScene.boundary.periodicX;
    double sumVx = 0.0; // mean flow velocity this substep, for the camera-follow display offset
    for (auto& obj : objects) {
        const float r = obj.Radius();
        obj.vel += obj.acc * subDt;
        obj.pos += obj.vel * subDt;
        if (periodicX) {
            // Teleport across the seam rather than bounce — velocity is left untouched, so the
            // channel behaves like an infinite domain rather than a wall.
            if      (obj.pos.x >  hw) obj.pos.x -= 2.f * hw;
            else if (obj.pos.x < -hw) obj.pos.x += 2.f * hw;
            sumVx += obj.vel.x;
        } else {
            resolveAxis(obj.pos.x, obj.vel.x, obj.vel.y, hw, r);
        }
        resolveAxis(obj.pos.y, obj.vel.y, obj.vel.x, hh, r);
    }
    if (periodicX && cameraFollow && !objects.empty()) {
        cameraOffsetX += static_cast<float>(sumVx / objects.size()) * subDt;
        cameraOffsetX  = WrapX(cameraOffsetX, hw);
    }
}

// Draws the current particles/boundary and submits the frame (capturing it if a video recording
// is active), then records the per-frame bookkeeping (kinetic-energy history, CSV data sample) —
// one call is exactly one simulated+rendered(+captured) frame, at whatever dt physics was just
// advanced by. Running calls this once per real frame; AdvanceRendering calls it back-to-back at
// a fixed 1/fps dt instead, with no pacing to real time and no ImGui overlay drawn over it (an
// overlay would get baked straight into the captured video — see VulkanContext::RenderFrame).
static void DrawAndSubmitFrame(float dt, bool offscreen) {
    const bool coloring = colorMode != ColorMode::Off && !objects.empty();
    float kineticEnergy = 0.f;
    float colorLo = 0.f, colorHi = 0.f;
    if (coloring) colorLo = colorHi = FieldValue(objects[0], colorMode);
    for (auto& obj : objects) {
        kineticEnergy += 0.5f * (obj.vel.x * obj.vel.x + obj.vel.y * obj.vel.y);
        if (coloring) {
            const float v = FieldValue(obj, colorMode);
            colorLo = std::min(colorLo, v);
            colorHi = std::max(colorHi, v);
        }
    }

    // Display-only x shift (see Integrate's cameraOffsetX accumulation) — never touches obj.pos,
    // so physics/CSV/data-save all still see true simulated positions.
    const bool  followCam = ActiveScene.boundary.periodicX && cameraFollow;
    const float hw        = ScreenHalfWidth();
    if (coloring) {
        // Auto-scaled to the current frame's range rather than a fixed scale — the field's
        // magnitude varies wildly across scenes (and over a Poiseuille run's own ramp-up), so a
        // fixed range would either clip everything to one end or wash out early on.
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

    // Batch rendering (see AdvanceRendering) never touches the swapchain/window at all — offscreen
    // draws+captures every frame unconditionally instead, so it can't be throttled by vsync or by
    // the OS deprioritizing an occluded/backgrounded window.
    if (offscreen) app.RenderOffscreenFrame();
    else           app.RenderFrame(dt);
    app.SetObjectCount(static_cast<int>(objects.size()));
    app.SetKineticEnergy(kineticEnergy);

    if (runIndex != lastRunIndexForKE) { lastRunIndexForKE = runIndex; runFrameForKE = 0; }
    keByFrame.push_back({ runIndex, runFrameForKE++, kineticEnergy });

    if (dataRecorder.IsActive()) {
        dataAccum += dt;
        const float interval = 1.f / static_cast<float>(dataRateActive);
        // Sampled on its own accumulator, independent of the render/physics rate, same as
        // Recorder's fps gate for video — dataRate is usually far below the sim's own rate.
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

// Crunches physics AND rendering/capture back-to-back at a fixed 1/fps dt instead of pacing to
// real time, so a video finishes as fast as the CPU/GPU can produce it rather than taking
// lengthSeconds of real time. Draws+captures offscreen (see DrawAndSubmitFrame/
// RenderOffscreenFrame) — no swapchain/window involvement, so nothing here can be throttled by
// vsync or by the OS deprioritizing an occluded/backgrounded window. Relies on the recorder's own
// length cap to know when it's done — once app.IsRecording() goes false (length reached, or
// Cancel's StopRecording()), there's no live playback to fall into, so this goes straight back to
// Menu. Still budgeted to a wall-clock slice per call so the main loop's PollEvents() keeps the
// window responsive during a long render instead of hanging; the caller separately refreshes the
// on-screen progress overlay once per real tick (a plain, cheap swapchain present, decoupled from
// this loop's own pace) rather than this loop touching the swapchain itself.
// True while the active run still has an output driving the batch loop — a data-only queue entry
// (no video) never sets app.IsRecording(), so the loop needs both checked to know it's still going.
static bool BatchRunActive() { return app.IsRecording() || dataRecorder.IsActive() || profileLogger.IsActive(); }

static void AdvanceRendering() {
    using Clock = std::chrono::steady_clock;
    const auto  budgetStart            = Clock::now();
    constexpr float FrameBudgetSeconds = 1.f / 30.f;
    const float frameDt = 1.f / static_cast<float>(std::max(activeRun.fps, 1));

    while (BatchRunActive()) {
        AdvanceSimulation(frameDt);
        DrawAndSubmitFrame(frameDt, /*offscreen=*/true);
        if (std::chrono::duration<float>(Clock::now() - budgetStart).count() >= FrameBudgetSeconds)
            break;
    }

    // StopRecording()/dataRecorder.Stop() here are no-ops if the output already stopped itself on
    // reaching its own length cap — this just handles whichever one (or both) just finished.
    if (!BatchRunActive()) {
        if (queueRunningIndex >= 0) AdvanceQueue();
        else                        ReturnToMenu();
    }
}

int main(int, char**) {
    // Assigned here, not via a default member initializer on MenuConfig — menuConfig (this TU)
    // and ScenePresets (Scene.cpp) are globals in different translation units with unspecified
    // relative init order, so reading ScenePresets[0] from a static initializer would be a real
    // hazard. By main()'s first line every global constructor has already run.
    menuConfig.scene = ScenePresets[0];
    RefreshUserPresets();

    app.Init(Config::WindowTitle, Config::WindowWidth, Config::WindowHeight);
    app.SetUICallback([]() { DrawUi(); });

    objects.reserve(100000);
    keByFrame.reserve(1u << 16); // avoid mid-run reallocations from skewing frame timing

    using Clock = std::chrono::steady_clock;
    auto last = Clock::now();

    profiler.MarkFrameStart();
    while (app.PollEvents()) {
        if (app.TakeF1Toggle()) {
            profilerOpen = !profilerOpen;
            app.SetProfilerOpen(profilerOpen);
        }
        // Drained every frame regardless of appState so a press queued during the menu (or while
        // a text field had focus) never leaks into the next Running session.
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
            if (!paused) AdvanceSimulation(dt);
            profiler.MarkComputeEnd();
            DrawAndSubmitFrame(dt, /*offscreen=*/false);
            profiler.MarkRenderEnd();
        } else if (appState == AppState::Rendering) {
            // Same key Running uses for pause/resume; the Cancel button in DrawRenderingOverlay
            // does the same thing.
            if (spacePressed) {
                ReturnToMenu();
            } else {
                profiler.MarkComputeEnd();
                AdvanceRendering();
                profiler.MarkRenderEnd();
            }
            // Refreshes the progress overlay (or, if AdvanceRendering just finished, the menu) —
            // deliberately separate from AdvanceRendering's own offscreen capture loop above, so
            // showing it only costs one swapchain acquire/present per real tick, not one per video
            // frame captured within that tick's budget.
            app.RenderFrame(dt);
        } else {
            app.RenderFrame(dt); // Menu: just the UI, no particles/bookkeeping
            profiler.MarkComputeEnd();
            profiler.MarkRenderEnd();
        }

        if (profiler.Tick()) {
            app.SetProfilerStats(profiler.GetStats());
            if (appState == AppState::Running) WriteVelocityProfileCsv();
        }

        profiler.MarkFrameStart();
    }
    WriteKineticEnergyCsv(keByFrame);
    app.Shutdown();
    return 0;
}
