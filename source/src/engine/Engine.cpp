#include <imgui_internal.h>
#include <SDL3/SDL.h>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
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

// User-defined entry points
void Init();
void Update(float dt);

std::vector<Object> objects;

// Directory of the running executable, so output files land next to it regardless of the working directory.
static std::filesystem::path ExecutableDir() {
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
static Profiler profiler; // frame/render timing, shown in the live panel's Run card

// The simulation only advances in Running/Rendering; Menu skips physics entirely. Rendering runs
// the same physics back-to-back at a fixed 1/fps dt, drawn and captured straight to video with
// no real-time pacing (see AdvanceRendering/DrawRenderingOverlay).
enum class AppState { Menu, Running, Rendering };
static AppState appState = AppState::Menu;

// Freezes physics (rendering, recording and data-saving keep running). Toggled by Space or the
// Pause/Resume button; only Restart/Menu/StartSimulation reset it.
static bool paused = false;

// Periodic-channel scenes drift sideways as the force advects the cloud, burying the profile
// under bulk translation. When enabled, the drawn (not simulated) x position is offset by the
// accumulated mean flow velocity each substep (see Integrate), so only relative motion shows.
static bool  cameraFollow  = true;
static float cameraOffsetX = 0.f;

constexpr float StepDt         = 1.f / 60.f; // fixed frame length for manual stepping
constexpr int   BigStepFrames  = 10;         // frames advanced by Left Arrow / the "Step x10" button

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
    // Index into the built-in ScenePresets followed by userPresets (see the Combined helpers below).
    int          sceneIndex = 0;
    // Editable working copy, reseeded from BuildScene()/LoadScenePreset() only when sceneIndex or
    // the grid count changes. Assigned in main(), not via a default member initializer.
    Scene        scene;
    RunSettings  run;
};
static MenuConfig menuConfig;

// Presets saved via the menu, scanned at startup and after each save. The scene combo lists the
// built-ins at [0, ScenePresets.size()), then these.
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

// Owned here (unlike Recorder) because the data it samples, `objects`, lives here.
static DataRecorder dataRecorder;
static float        dataAccum{ 0.f };        // seconds since the last sample, gated at the active run's dataRate
static float        dataElapsed{ 0.f };       // sim seconds since this data save started (CSV time column)
static uint32_t     dataSavedFrames{ 0 };
static int          dataRateActive{ 0 };
static float        dataLengthActive{ 0.f };  // 0 = unlimited

// Defined further down; declared here for ProfileLogger.
float ScreenHalfWidth();
float ScreenHalfHeight();

// Simulated seconds since the last ResetSimulation(); advanced only by AdvanceSimulation, so it stops while paused.
static double simTime = 0.0;

// Channel velocity-profile logger. Every 1/rate simulated seconds it bins particles by y across the
// channel and writes two small CSVs:
//   <name>_profile.csv : frame,time,bin,y_center,count,mean_vx,std_vx,mean_vy,mean_density
//   <name>_frames.csv  : frame,time,mean_vx,max_vx,rms_vy,mean_density,std_density,min_density,max_density
// Both start with '#'-prefixed lines recording the run's geometry and parameters (read with
// pandas' comment='#'). Written synchronously on the sim thread; a sample is only nBins+1 rows.
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


// Live coloring: recolors every particle by a simulation field instead of its scene color. Each
// field has its own editable low/high color pair.
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

// The mode + gradient picker used by DrawAndSubmitFrame, shared by the live panel and the setup
// view so a batch render can set it before starting.
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

        // Label on the left, swatch on the right; NoInputs leaves just the button, with sliders in its popup.
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

// Editable sections for the menu's CollapsingHeaders, one function per Scene sub-struct.

// One labelled input row: the label in a fixed left column (wrapping if long), the widget filling
// the right. Call immediately before the widget and give it a "##"-hidden label.
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


// Persistent-color patches (see SceneColorRegion), edited like events: panel + drag gizmo.
static SceneColorRegion placeRegionTemplate;
static int              selectedRegion = -1;
// True on frames where the "Persistent Color" window is the visible tab of the right dock node.
static bool             colorsTabActive = false;

// Solid obstacles (see SceneObstacle): same editing model as the color areas.
static SceneObstacle    placeObstacleTemplate;
static int              selectedObstacle = -1;
static bool             obstaclesTabActive = false;

// Particle groups (see SceneGroup): the spawn blocks, edited like obstacles.
static SceneGroup       placeGroupTemplate;
static int              selectedGroup = -1;
static bool             groupsTabActive = false;

// Shared by SceneColorRegion, SceneObstacle and SceneGroup (groups draw their own optional-color row).
template <typename Shape>
static void DrawColorRegionFields(Shape& r, float contentWidth, bool withColor = true) {
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
    if (!withColor) return;
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

// ---- Physics tab sections, one per concern: fluid, flow, forcing, walls and time stepping.

static void DrawExternalForceSection(ScenePhysics& p, float contentWidth) {
    FormRow(contentWidth, "Force (x, y)");
    ImGui::InputFloat2("##Force (x, y)", &p.force.x);
    ImGui::TextDisabled("Uniform acceleration on every particle (e.g. gravity), px/s^2.");
}

static void DrawWallsSection(ScenePhysics& p, float contentWidth) {
    FormRow(contentWidth, "Restitution");
    ImGui::SliderFloat("##Restitution", &p.restitution, 0.f, 1.f, "%.2f");
    FormRow(contentWidth, "Friction");
    ImGui::SliderFloat("##Friction", &p.friction, 0.f, 1.f, "%.2f");
    ImGui::Checkbox("No-slip walls", &p.noSlipWalls);
    ImGui::TextDisabled("Applies to the container and to obstacles.");
}

static void DrawIntegrationSection(ScenePhysics& p, float contentWidth) {
    FormRow(contentWidth, "Substeps");
    ImGui::InputInt("##Substeps", &p.substeps);
    p.substeps = std::max(p.substeps, 1);
    FormRow(contentWidth, "Force interval");
    ImGui::InputInt("##Force interval", &p.forceInterval);
    p.forceInterval = std::max(p.forceInterval, 1);
    ImGui::TextDisabled("Substeps per frame; forces are recomputed every <interval> substeps.");
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

// The fluid itself: equation of state, stiffness, rest density and smoothing radius.
static void DrawFluidSection(SceneParticles& pr, float contentWidth) {
    int eosIdx = static_cast<int>(pr.eos);
    FormRow(contentWidth, "Equation of state");
    if (ImGui::Combo("##EOS", &eosIdx, EosModelNames, IM_ARRAYSIZE(EosModelNames)))
        pr.eos = static_cast<EosModel>(eosIdx);
    if (pr.eos == EosModel::WCSPH) {
        FormRow(contentWidth, "Exponent");
        ImGui::InputInt("##Exponent", &pr.exponent);
    } else {
        FormRow(contentWidth, "Polytropic index");
        ImGui::InputFloat("##Polytropic index", &pr.polytropicIndex);
    }
    FormRow(contentWidth, "Stiffness");
    ImGui::InputFloat("##Stiffness", &pr.stiffness);
    FormRow(contentWidth, "Target density");
    ImGui::InputFloat("##Target density", &pr.targetDensity, 0.f, 0.f, "%.6f");
    FormRow(contentWidth, "Smoothing radius");
    ImGui::InputFloat("##Smoothing radius", &pr.smoothingRadius);
}

static void DrawViscositySection(SceneParticles& pr, float contentWidth) {
    int viscIdx = static_cast<int>(pr.viscosityModel);
    FormRow(contentWidth, "Model");
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

// Defined further down (needs ScreenHalfWidth/Height); forward-declared for AdvanceSimulation.
static bool Integrate(float subDt);

// ---- Events (see SceneEvent) -------------------------------------------------------------------
static bool       showEventAreas     = true;  // draw event gizmos over the live view (never recorded)
static SceneEvent placeEventTemplate = [] { SceneEvent e; e.accel = Vec2(400.f, 0.f); return e; }();
// Whether each Once event has fired this run. Parallel to ActiveScene.events; reset by ResetSimulation.
static std::vector<char> eventFired;

static bool InsideEvent(const SceneEvent& e, const Vec2& p) {
    const float dx = p.x - e.cx, dy = p.y - e.cy;
    if (e.shape == EventShape::Circle) return dx * dx + dy * dy <= e.radius * e.radius;
    return std::fabs(dx) <= e.halfW && std::fabs(dy) <= e.halfH;
}

// Adds each due Once event's velocity delta to every particle inside its area, once per run.
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

// Applies every Continuous event active at sim time `t` as an acceleration over `subDt`. Goes onto
// vel, not acc, because acc is only recomputed every forceInterval substeps.
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

// Window-sized container half-extents are measured once per run in ResetSimulation, so resizing
// mid-run can't move the walls under the particles. <= 0 = not frozen (menu).
static float frozenHalfW = 0.f, frozenHalfH = 0.f;

// `fullWindow`: measure against the whole window (recordings and batch renders) instead of the
// free area between the docked panels.
// ---- Sim thread ---------------------------------------------------------------------------------
// Live (Running) physics runs on a worker thread so a slow or diverging sim can't freeze the
// window. The UI thread asks for time to be simulated; the worker advances it.
//
// simMutex guards all simulation state (objects, ActiveScene, eventFired, simTime, cameraOffsetX,
// profileLogger sampling, the fault message). The worker holds it during a substep and releases it
// between substeps; the UI thread asks the worker to yield (uiWaiting) so it waits at most one
// substep. If the lock can't be taken in time the UI shows a "not responding" notice instead.
// Batch rendering calls AdvanceSimulation from the main thread, where the lock is uncontended.
static std::timed_mutex          simMutex;
static std::atomic<int>          uiWaiting{ 0 };
static std::atomic<uint32_t>     simGeneration{ 0 }; // bumped by ResetSimulation; a step in flight abandons itself
static std::atomic<bool>         simAbort{ false };  // set at shutdown; the worker stops at the next substep
static std::atomic<bool>         simFaulted{ false };
static std::string               simFaultMessage;    // guarded by simMutex

static std::mutex                workMutex;
static std::condition_variable   workCv;
static double                    workPendingDt      = 0.0; // simulated seconds the UI thread wants advanced
static int                       workManualSteps    = 0;   // StepDt-sized steps requested by Step / Step x10
static bool                      workQuit           = false;
static std::thread               simThread;
static std::atomic<bool>         simThreadExited{ false };
static std::atomic<float>        simStepMs{ 0.f }; // smoothed wall time of one worker step ("Compute")
static std::atomic<float>        simSpeed{ 1.f };  // smoothed fraction of real time the sim keeps up with
static float                     renderCpuMs  = 0.f; // UI thread: smoothed draw/submit time, excluding waits
static float                     lockWaitMs   = 0.f; // UI thread: time LockSim waited this frame

using SimLock = std::unique_lock<std::timed_mutex>;

// Takes simMutex for the UI thread, making the worker yield. False if the worker is stuck in a
// single substep longer than `timeoutMs`; the caller must then not touch sim state.
static bool LockSim(SimLock& lk, int timeoutMs = 100) {
    const auto start = std::chrono::steady_clock::now();
    uiWaiting++;
    lk = SimLock(simMutex, std::defer_lock);
    const bool ok = lk.try_lock_for(std::chrono::milliseconds(timeoutMs));
    uiWaiting--;
    lockWaitMs += std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start).count();
    return ok;
}

// Marks the sim diverged and stops it advancing until the next reset. Caller holds simMutex.
static void SetFaultLocked(const char* message) {
    if (simFaulted) return;
    simFaultMessage = message;
    simFaulted      = true;
    std::fprintf(stderr, "[sim] fault: %s\n", message);
}

static void ClearPendingWork() {
    std::lock_guard<std::mutex> lk(workMutex);
    workPendingDt   = 0.0;
    workManualSteps = 0;
}

static void ResetSimulation(bool fullWindow) {
    simGeneration++;
    ClearPendingWork();
    simFaulted = false;
    simFaultMessage.clear();
    objects.clear();
    {
        const auto& b = ActiveScene.boundary;
        const bool whole = fullWindow && !app.IsRecording();
        frozenHalfW = (whole ? app.HalfWidth()  : app.RecordingHalfWidth())  * b.widthFrac;
        frozenHalfH = (whole ? app.HalfHeight() : app.RecordingHalfHeight()) * b.heightFrac;
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
static void AdvanceSimulationLocked(float dt) {
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
static void AdvanceSimulation(float dt) {
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

static void SimWorkerMain() {
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

static void StartSimThread() { simThread = std::thread(SimWorkerMain); }

// Stops the worker. If it's wedged in a substep it is detached instead, and the caller must exit
// without running static destructors (returns false).
static bool StopSimThread() {
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
static void QueueSimTime(float dt) {
    {
        std::lock_guard<std::mutex> lk(workMutex);
        workPendingDt = std::min(workPendingDt + static_cast<double>(dt), 1.0 / 30.0);
    }
    workCv.notify_one();
}

static void QueueManualSteps(int count) {
    {
        std::lock_guard<std::mutex> lk(workMutex);
        workManualSteps += count;
    }
    workCv.notify_one();
}

static void TogglePause() { paused = !paused; }

// Stepping pauses first, so Step/arrow keys freeze the sim on that frame.
static void StepOnce() { paused = true; QueueManualSteps(1); }
static void StepBig()  { paused = true; QueueManualSteps(BigStepFrames); }

// The run configuration currently driving Running/Rendering (manual or queued), set by StartRun.
static RunSettings activeRun;

// One queued run: a Scene snapshot plus its own output settings, captured by value so later menu
// edits don't change it.
struct QueueEntry {
    Scene       scene;
    RunSettings run;
};
static std::vector<QueueEntry> runQueue;
static int  queueRunningIndex = -1; // -1 = no queue run in progress; runQueue itself is kept for re-running

// A queued entry needs a finite end or it would stall the queue. Every enabled output must be
// finite (and at least one enabled), since AdvanceRendering runs until all outputs stop.
static bool HasDeterministicEnd(const RunSettings& run) {
    if (!run.record && !run.saveData && !run.saveProfile) return false;
    return (!run.record      || run.lengthSeconds        > 0.f) &&
           (!run.saveData    || run.dataLengthSeconds    > 0.f) &&
           (!run.saveProfile || run.profileLengthSeconds > 0.f);
}

static void AdvanceQueue();

// Loads `scene`, resets the sim, and starts the outputs `run` requests. Shared by a manual start
// and queued entries. `forceBatch` is true for queue entries (only batch mode has a deterministic
// end); a manual run uses `run.batchRender`.
static void StartRun(const Scene& scene, const RunSettings& run, bool forceBatch) {
    LoadScene(scene);
    activeRun = run;
    ResetSimulation(/*fullWindow=*/run.record || forceBatch || run.batchRender);

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

static void StartSimulationFromMenu() {
    StartRun(menuConfig.scene, menuConfig.run, /*forceBatch=*/false);
}

// Starts queue entry `entry` in forced-batch mode; if nothing actually starts (see StartRun's
// fallback), advances past it, since AdvanceQueue is only reachable from AdvanceRendering.
static void StartQueueEntry(const QueueEntry& entry) {
    StartRun(entry.scene, entry.run, /*forceBatch=*/true);
    if (appState != AppState::Rendering) AdvanceQueue();
}

static void StartQueue() {
    if (runQueue.empty()) return;
    queueRunningIndex = 0;
    StartQueueEntry(runQueue[0]);
}

// Resets the sim in place without leaving Running or touching active outputs (unlike ReturnToMenu).
static void RestartSimulation() {
    ResetSimulation(activeRun.record || activeRun.batchRender || queueRunningIndex >= 0);
}

static void ReturnToMenu() {
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
// Docked-panel UI: a top bar, a left panel (setup tabs / live controls / render progress by
// appState), a right panel (events) and the sim view in the center, which also hosts the event
// gizmos (see ui/EventGizmo.h). All drawn by DrawUi(), the single UI callback.
// ================================================================================================

// Index into UiEvents() of the event the right panel inspects and the gizmo highlights.
static int selectedEvent = -1;

// The event list the panels/gizmo edit: the menu's scene before a run, the live scene during one.
static std::vector<SceneEvent>& UiEvents() {
    return appState == AppState::Menu ? menuConfig.scene.events : ActiveScene.events;
}

// A tinted, rounded "card" grouping related rows. Take widths from GetContentRegionAvail().
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

// Container half-extents as the setup view shows them (same rule as DrawSimOverlay's preview).
static void MenuHalfExtents(const Scene& s, float& halfW, float& halfH) {
    halfW = s.boundary.halfWidthPx  > 0.f ? s.boundary.halfWidthPx  : app.RecordingHalfWidth()  * s.boundary.widthFrac;
    halfH = s.boundary.halfHeightPx > 0.f ? s.boundary.halfHeightPx : app.RecordingHalfHeight() * s.boundary.heightFrac;
}

// Seeds the scene's editable groups from its spawn block (grid counts, circular, offset, packing).
// Called when a scene loads and whenever one of those block controls is edited.
static void SyncGroupsFromSpawn(Scene& s) {
    float halfW, halfH;
    MenuHalfExtents(s, halfW, halfH);
    s.groups    = { SpawnBlockAsGroup(s, halfW, halfH) };
    s.useGroups = true;
}

// Which scene index menuConfig.scene was last seeded from (-1 = never); "Save Current as Preset"
// re-points it so saving doesn't reseed.
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

    // Reseeds menuConfig.scene (from the preset factory or file) when the scene selection changes,
    // including the first call. Other frames leave the user's edits alone.
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
            if (!menuConfig.scene.useGroups) SyncGroupsFromSpawn(menuConfig.scene);
        } else {
            // The preset file is gone/unreadable: fall back to the first built-in and rescan.
            RefreshUserPresets();
            menuConfig.sceneIndex = 0;
            menuConfig.scene = BuildScene(0, ScenePresets[0].spawn.gridCountX, ScenePresets[0].spawn.gridCountY);
            SyncGroupsFromSpawn(menuConfig.scene);
            lastSyncedSceneIndex = 0;
        }
    }

    ImGui::TextDisabled("%d particles  |  real-time limit %d", SceneParticleCount(menuConfig.scene),
                         menuConfig.scene.realTimeLimitX * menuConfig.scene.realTimeLimitY);
    ImGui::TextDisabled("Particle blocks are placed in the Groups tab of the right panel.");
    EndCard();
    }

    // Particle size and base color live here only; the size sets every group's lattice spacing.
    Widgets::Caption("PARTICLES");
    BeginCard("##particleCard");
    {
        auto& pr = menuConfig.scene.particles;
        const float cw = ImGui::GetContentRegionAvail().x;
        FormRow(cw, "Particle radius (px)");
        ImGui::InputFloat("##Circle radius", &pr.circleRadius);
        pr.circleRadius = std::max(pr.circleRadius, 0.5f);
        const float swatchSize = ImGui::GetFrameHeight();
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Particle color");
        ImGui::SameLine(cw - swatchSize);
        ImGui::ColorEdit4("##circlecolor", &pr.circleColor.r, ImGuiColorEditFlags_NoInputs);
        ImGui::TextDisabled("Groups can override the color; color areas and patterns paint over it.");
    }
    EndCard();

    Widgets::Caption("CONTAINER");
    if (ImGui::CollapsingHeader("Boundary", ImGuiTreeNodeFlags_DefaultOpen)) DrawBoundarySection(menuConfig.scene.boundary, w);

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
                // Re-saving the selected preset updates it in place; anything else landing on an
                // existing file gets a numeric suffix.
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

                // Re-scanning can shift user preset indices, so re-point sceneIndex (and
                // lastSyncedSceneIndex, to avoid a reseed) at the just-saved file.
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
    Scene& s = menuConfig.scene;
    Widgets::Caption("FLUID");
    if (ImGui::CollapsingHeader("Equation of state", ImGuiTreeNodeFlags_DefaultOpen)) DrawFluidSection(s.particles, w);
    if (ImGui::CollapsingHeader("Viscosity", ImGuiTreeNodeFlags_DefaultOpen))         DrawViscositySection(s.particles, w);

    Widgets::Caption("FORCES");
    if (ImGui::CollapsingHeader("External force", ImGuiTreeNodeFlags_DefaultOpen))    DrawExternalForceSection(s.physics, w);
    if (ImGui::CollapsingHeader("Self-gravity"))                                      DrawGravitySection(s.gravity, w);

    Widgets::Caption("BOUNDARIES");
    if (ImGui::CollapsingHeader("Walls and obstacles", ImGuiTreeNodeFlags_DefaultOpen)) DrawWallsSection(s.physics, w);

    Widgets::Caption("SOLVER");
    if (ImGui::CollapsingHeader("Time stepping", ImGuiTreeNodeFlags_DefaultOpen))     DrawIntegrationSection(s.physics, w);
    ImGui::PopTextWrapPos();
}

// Every enabled output (video, CSV data, velocity profile) needs a title for its file name. A
// start or queue without one jumps to the Output tab and marks the empty fields.
static bool TitleBlank(const char* s) {
    for (; *s; ++s)
        if (!std::isspace(static_cast<unsigned char>(*s))) return false;
    return true;
}
static bool MissingTitles(const RunSettings& r) {
    return (r.record && TitleBlank(r.title)) || (r.saveData && TitleBlank(r.dataTitle)) ||
           (r.saveProfile && TitleBlank(r.profileTitle));
}

static bool showTitleErrors = false; // mark empty required titles (after a refused Start)
static int  requestedTab    = -1;    // setup tab to switch to next frame (index into DrawSetupPanel's tabs)
constexpr int OutputTabIndex = 2;

static bool RequireTitles(const RunSettings& r) {
    showTitleErrors = MissingTitles(r);
    if (showTitleErrors) requestedTab = OutputTabIndex;
    return !showTitleErrors;
}

static void DrawTitleInput(float w, const char* label, const char* id, char* buf, size_t size) {
    FormRow(w, label);
    const bool bad = showTitleErrors && TitleBlank(buf);
    if (bad) {
        ImGui::PushStyleColor(ImGuiCol_Border, Theme::Coral);
        ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.5f);
    }
    ImGui::InputText(id, buf, size);
    if (bad) {
        ImGui::PopStyleVar();
        ImGui::PopStyleColor();
        ImGui::TextColored(Theme::Coral, "A title is required.");
    }
}

static void DrawQueueSection();

// A run's output settings: particle coloring, what gets saved, and the run queue.
static void DrawOutputTab() {
    const float w = ImGui::GetContentRegionAvail().x;
    ImGui::PushTextWrapPos(0.f);

    // Always available so coloring can be set up before a run, including for batch renders.
    Widgets::Caption("APPEARANCE");
    if (ImGui::CollapsingHeader("Color by")) DrawColorByControls(w);

    Widgets::Caption("SAVE");
    ImGui::Checkbox("Record to video", &menuConfig.run.record);
    if (menuConfig.run.record) {
        DrawTitleInput(w, "Title", "##Title", menuConfig.run.title, sizeof(menuConfig.run.title));
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
        DrawTitleInput(w, "Data title", "##Data title", menuConfig.run.dataTitle, sizeof(menuConfig.run.dataTitle));
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
        DrawTitleInput(w, "Profile title", "##Profile title", menuConfig.run.profileTitle, sizeof(menuConfig.run.profileTitle));
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

    ImGui::Spacing();
    Widgets::Caption("RUN QUEUE");
    DrawQueueSection();

    ImGui::PopTextWrapPos();
}

static void DrawQueueSection() {
    const float w = ImGui::GetContentRegionAvail().x;
    ImGui::PushTextWrapPos(0.f);
    if (ImGui::Button("Add to Queue", ImVec2(w, 0.f))) {
        if (HasDeterministicEnd(menuConfig.run) && RequireTitles(menuConfig.run))
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
        ImGui::Text("%s (%d particles)%s%s%s", runQueue[i].scene.name.c_str(),
                    SceneParticleCount(runQueue[i].scene),
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
    // Sticky footer: the tab content above scrolls; the particle readout and Start button don't.
    const float buttonH = ImGui::GetFrameHeight() * 1.5f;
    const float footerH = buttonH + ImGui::GetTextLineHeight() + style.ItemSpacing.y * 3.f;

    // Errors clear once every required title is filled in, so a newly enabled output doesn't start red.
    if (showTitleErrors && !MissingTitles(menuConfig.run)) showTitleErrors = false;

    if (ImGui::BeginTabBar("##setupTabs")) {
        struct Tab { const char* name; void (*draw)(); };
        const Tab tabs[] = { { "Scene", DrawSceneTab }, { "Physics", DrawPhysicsTab },
                             { "Output", DrawOutputTab } };
        for (const Tab& t : tabs) {
            const int index = static_cast<int>(&t - tabs);
            // A refused Start selects the Output tab via SetSelected on the requesting frame.
            const ImGuiTabItemFlags flags = index == requestedTab ? ImGuiTabItemFlags_SetSelected : ImGuiTabItemFlags_None;
            if (!ImGui::BeginTabItem(t.name, nullptr, flags)) continue;
            ImGui::BeginChild("##tabScroll", ImVec2(0.f, -footerH));
            ImGui::Spacing();
            t.draw();
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    requestedTab = -1;

    ImGui::Separator();
    const int count = SceneParticleCount(menuConfig.scene);
    ImGui::TextDisabled("%d particles  |  real-time limit %d", count,
                         menuConfig.scene.realTimeLimitX * menuConfig.scene.realTimeLimitY);
    if (Widgets::PrimaryButton("Start Simulation", ImVec2(-FLT_MIN, buttonH)) && RequireTitles(menuConfig.run))
        StartSimulationFromMenu();
}

// ---- Live view controls (Running state) -----------------------------------------------------

static void DrawLivePanel() {
    ImGui::PushTextWrapPos(0.f);

    Widgets::Caption("RUN");
    BeginCard("##runCard");
    // Label on the left, value right-aligned to the card's edge.
    auto statRow = [](const char* label, const char* value, const ImVec4* color = nullptr) {
        ImGui::TextUnformatted(label);
        ImGui::SameLine();
        const float valueW = ImGui::CalcTextSize(value).x;
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(ImGui::GetContentRegionAvail().x - valueW, 0.f));
        if (color) ImGui::TextColored(*color, "%s", value);
        else       ImGui::TextUnformatted(value);
    };
    char buf[48];
    std::snprintf(buf, sizeof(buf), "%.2f s", simTime);
    statRow("Simulated time", buf);
    std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(objects.size()));
    statRow("Particles", buf);
    // The sim and renderer run side by side, so these don't add up: Frame is one (vsync-limited)
    // frame, Compute is the sim thread's time per step, Render is the UI thread's drawing work
    // without GPU/vsync waits, and Sim speed is the fraction of real time the sim keeps up with.
    const Profiler::Stats& perf = profiler.GetStats();
    std::snprintf(buf, sizeof(buf), "%.0f", perf.fps);
    statRow("FPS", buf);
    std::snprintf(buf, sizeof(buf), "%.2f ms", perf.frameMs);
    statRow("Frame", buf);
    std::snprintf(buf, sizeof(buf), "%.2f ms", simStepMs.load());
    statRow("Compute", buf);
    std::snprintf(buf, sizeof(buf), "%.2f ms", renderCpuMs);
    statRow("Render (CPU)", buf);
    const float speed = simSpeed.load();
    if (paused) {
        statRow("Sim speed", "paused");
    } else {
        std::snprintf(buf, sizeof(buf), "%.0f%%", speed * 100.f);
        const ImVec4 speedColor = speed < 0.97f ? Theme::Coral : Theme::Teal;
        statRow("Sim speed", buf, &speedColor);
    }
    ImGui::TextDisabled("Space: pause  |  Right: step  |  Left: step x10");
    if (simFaulted) {
        ImGui::Spacing();
        // TextWrapped wraps at the card's content edge; the panel's outer PushTextWrapPos(0) would clip.
        ImGui::PushStyleColor(ImGuiCol_Text, Theme::Coral);
        ImGui::TextWrapped("%s", simFaultMessage.c_str());
        ImGui::PopStyleColor();
        ImGui::TextDisabled("Stepping is stopped. Press Restart to reset the sim.");
    }
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

// Never reaches the recorded video: RenderFrame captures the scene pass before drawing ImGui.
static void DrawRenderingPanel() {
    const float w = ImGui::GetContentRegionAvail().x;
    ImGui::PushTextWrapPos(0.f);

    if (queueRunningIndex >= 0)
        ImGui::Text("Queue: %d / %d", queueRunningIndex + 1, static_cast<int>(runQueue.size()));
    ImGui::TextUnformatted(ActiveScene.name.c_str());
    ImGui::Spacing();

    // A data-only entry never sets app.IsRecording(), so track elapsed time via the data-save sample count.
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

// Result of the async "Browse..." dialog; SDL may call back on another thread, so it is handed
// over through a mutex.
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
        // The base color is set in the Scene tab and per group; this panel only paints over it.
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

    ImGui::Checkbox("Blend colors", &menuConfig.scene.blendColorRegions);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Gradient between areas' colors (and between pattern cells) instead of hard edges");
    if (menuConfig.scene.blendColorRegions) {
        Scene& bs = menuConfig.scene;
        FormRow(w, "Blend strength");
        ImGui::SliderFloat("##blendStrength", &bs.blendStrength, 0.f, 1.f, "%.2f");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("0 = flat colors with hard edges, 1 = full gradient");
        FormRow(w, "Blend radius (px)");
        ImGui::SliderFloat("##blendRadius", &bs.blendRadius, 0.f, 1000.f, bs.blendRadius <= 0.f ? "Auto" : "%.0f");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("How far the gradient reaches. 0 = automatic");
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

// Setup-only list of solid obstacles (see SceneObstacle).
static void DrawObstaclesPanel() {
    auto& obstacles = menuConfig.scene.obstacles;
    if (selectedObstacle >= static_cast<int>(obstacles.size())) selectedObstacle = -1;
    const float w = ImGui::GetContentRegionAvail().x;

    ImGui::TextDisabled("Drag in the view to draw an obstacle. Drag it to move, its handles to resize\n"
                         "(hold Alt to resize from the center). Del removes the selected one. No\n"
                         "particles spawn inside, and the fluid can't enter it during the run. Keep\n"
                         "obstacles off the seam of a periodic-X scene.");
    ImGui::Spacing();

    if (Widgets::PrimaryButton("+ New obstacle at center", ImVec2(w, 0.f))) {
        SceneObstacle o = placeObstacleTemplate;
        o.cx = 0.f; o.cy = 0.f;
        obstacles.push_back(o);
        selectedObstacle = static_cast<int>(obstacles.size()) - 1;
    }

    if (ImGui::CollapsingHeader("New obstacle defaults")) {
        static const char* ShapeNames[] = { "Circle", "Rect" };
        int shape = static_cast<int>(placeObstacleTemplate.shape);
        FormRow(w, "Shape");
        if (ImGui::Combo("##defObstacleShape", &shape, ShapeNames, 2)) placeObstacleTemplate.shape = static_cast<EventShape>(shape);
        const float swatchSize = ImGui::GetFrameHeight();
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Color");
        ImGui::SameLine(w - swatchSize);
        ImGui::ColorEdit4("##defObstacleColor", &placeObstacleTemplate.color.r, ImGuiColorEditFlags_NoInputs);
    }

    Widgets::Caption("OBSTACLES");
    if (obstacles.empty()) ImGui::TextDisabled("None yet.");
    for (int i = 0; i < static_cast<int>(obstacles.size()); i++) {
        const SceneObstacle& o = obstacles[i];
        ImGui::PushID(i);
        char label[96];
        std::snprintf(label, sizeof(label), "      %d   %s", i + 1, o.shape == EventShape::Circle ? "Circle" : "Rect");
        const ImVec2 rowPos = ImGui::GetCursorScreenPos();
        if (ImGui::Selectable(label, i == selectedObstacle)) selectedObstacle = i;
        ImGui::GetWindowDrawList()->AddCircleFilled(
            ImVec2(rowPos.x + ImGui::GetFontSize() * 0.7f, rowPos.y + ImGui::GetFrameHeight() * 0.5f - 2.f),
            ImGui::GetFontSize() * 0.28f, ImGui::ColorConvertFloat4ToU32(ImVec4(o.color.r, o.color.g, o.color.b, 1.f)));
        ImGui::PopID();
    }

    if (selectedObstacle >= 0) {
        SceneObstacle& o = obstacles[selectedObstacle];
        char title[32];
        std::snprintf(title, sizeof(title), "OBSTACLE %d", selectedObstacle + 1);
        Widgets::Caption(title);
        BeginCard("##obstacleInspector");
        DrawColorRegionFields(o, ImGui::GetContentRegionAvail().x);
        ImGui::Spacing();
        const float bw = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) / 2.f;
        if (ImGui::Button("Duplicate", ImVec2(bw, 0.f))) {
            SceneObstacle copy = o;
            copy.cx += 30.f; copy.cy -= 30.f;
            obstacles.push_back(copy);
            selectedObstacle = static_cast<int>(obstacles.size()) - 1;
        } else {
            ImGui::SameLine();
            if (Widgets::DangerButton("Delete", ImVec2(bw, 0.f))) {
                obstacles.erase(obstacles.begin() + selectedObstacle);
                selectedObstacle = -1;
            }
        }
        EndCard();
    }
}

// Setup-only list of particle groups (see SceneGroup). Deleting them all gives an empty scene.
static void DrawGroupsPanel() {
    auto& groups = menuConfig.scene.groups;
    if (selectedGroup >= static_cast<int>(groups.size())) selectedGroup = -1;
    const float w = ImGui::GetContentRegionAvail().x;
    const float circleR = menuConfig.scene.particles.circleRadius;

    ImGui::TextDisabled("Each group is a block of starting particles. Drag in the view to draw one,\n"
                         "drag it to move it, its handles to resize it (hold Alt to resize from the\n"
                         "center). Del removes the selected group; with none left the scene starts\n"
                         "empty. Particle size is set in the Scene tab.");
    ImGui::Spacing();

    if (Widgets::PrimaryButton("+ New group at center", ImVec2(w, 0.f))) {
        SceneGroup g = placeGroupTemplate;
        g.cx = 0.f; g.cy = 0.f;
        groups.push_back(g);
        selectedGroup = static_cast<int>(groups.size()) - 1;
    }

    if (ImGui::CollapsingHeader("New group defaults")) {
        static const char* ShapeNames[] = { "Circle", "Rect" };
        int shape = static_cast<int>(placeGroupTemplate.shape);
        FormRow(w, "Shape");
        if (ImGui::Combo("##defGroupShape", &shape, ShapeNames, 2)) placeGroupTemplate.shape = static_cast<EventShape>(shape);
        FormRow(w, "Packing (spacing x)");
        ImGui::SliderFloat("##defGroupSpacing", &placeGroupTemplate.spacingScale, 0.5f, 2.0f, "%.2f");
    }

    Widgets::Caption("GROUPS");
    if (groups.empty()) ImGui::TextDisabled("None - the scene starts empty.");
    for (int i = 0; i < static_cast<int>(groups.size()); i++) {
        const SceneGroup& g = groups[i];
        const Color c = g.customColor ? g.color : menuConfig.scene.particles.circleColor;
        ImGui::PushID(i);
        char label[96];
        std::snprintf(label, sizeof(label), "      %d   %s   %d", i + 1, g.shape == EventShape::Circle ? "Circle" : "Rect",
                      GroupParticleCount(g, GroupSpacing(g, circleR)));
        const ImVec2 rowPos = ImGui::GetCursorScreenPos();
        if (ImGui::Selectable(label, i == selectedGroup)) selectedGroup = i;
        ImGui::GetWindowDrawList()->AddCircleFilled(
            ImVec2(rowPos.x + ImGui::GetFontSize() * 0.7f, rowPos.y + ImGui::GetFrameHeight() * 0.5f - 2.f),
            ImGui::GetFontSize() * 0.28f, ImGui::ColorConvertFloat4ToU32(ImVec4(c.r, c.g, c.b, 1.f)));
        ImGui::PopID();
    }

    if (selectedGroup >= 0) {
        SceneGroup& g = groups[selectedGroup];
        char title[32];
        std::snprintf(title, sizeof(title), "GROUP %d", selectedGroup + 1);
        Widgets::Caption(title);
        BeginCard("##groupInspector");
        const float cw = ImGui::GetContentRegionAvail().x;
        DrawColorRegionFields(g, cw, /*withColor=*/false);
        FormRow(cw, "Packing (spacing x)");
        ImGui::SliderFloat("##groupSpacing", &g.spacingScale, 0.5f, 2.0f, "%.2f");
        const float spacing = GroupSpacing(g, circleR);
        // Editing the count resizes the block at the current spacing (the view updates live).
        int count = GroupParticleCount(g, spacing);
        FormRow(cw, "Particle count");
        if (ImGui::InputInt("##groupCount", &count, 0, 0))
            ResizeGroupToCount(g, spacing, count);
        ImGui::TextDisabled("spacing %.1f px  |  particle radius %.1f px", spacing, circleR);
        ImGui::Checkbox("Own color", &g.customColor);
        if (g.customColor) {
            const float swatchSize = ImGui::GetFrameHeight();
            ImGui::SameLine(cw - swatchSize);
            ImGui::ColorEdit4("##groupColor", &g.color.r, ImGuiColorEditFlags_NoInputs);
        } else {
            ImGui::SameLine();
            ImGui::TextDisabled("(scene particle color)");
        }
        ImGui::Spacing();
        const float bw = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) / 2.f;
        if (ImGui::Button("Duplicate", ImVec2(bw, 0.f))) {
            SceneGroup copy = g;
            copy.cx += 30.f; copy.cy -= 30.f;
            groups.push_back(copy);
            selectedGroup = static_cast<int>(groups.size()) - 1;
        } else {
            ImGui::SameLine();
            if (Widgets::DangerButton("Delete", ImVec2(bw, 0.f))) {
                groups.erase(groups.begin() + selectedGroup);
                selectedGroup = -1;
            }
        }
        EndCard();
    }
}

static void DrawGroupsWindow() {
    groupsTabActive = false;
    if (appState != AppState::Menu) return;
    if (const ImGuiWindow* rw = ImGui::FindWindowByName(Workspace::RightPanelId); rw && rw->DockId)
        ImGui::SetNextWindowDockID(rw->DockId, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Groups###GroupsPanel")) {
        groupsTabActive = true;
        ImGui::PushTextWrapPos(0.f);
        DrawGroupsPanel();
        ImGui::PopTextWrapPos();
    }
    ImGui::End();
}

// Own window, docked as a tab beside "Events" and "Persistent Color" on first appearance.
static void DrawObstaclesWindow() {
    obstaclesTabActive = false;
    if (appState != AppState::Menu) return;
    if (const ImGuiWindow* rw = ImGui::FindWindowByName(Workspace::RightPanelId); rw && rw->DockId)
        ImGui::SetNextWindowDockID(rw->DockId, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Obstacles###ObstaclesPanel")) {
        obstaclesTabActive = true;
        ImGui::PushTextWrapPos(0.f);
        DrawObstaclesPanel();
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

// Draws/edits a list of shapes (SceneColorRegion or SceneObstacle) through SceneEvent proxies so
// the events gizmo can drag them. Only interactive when `edit`. `fixedTint` (optional) replaces
// every shape's color; `fillScale` thins the fill so what's underneath stays visible.
template <typename Shape>
static void RunShapeGizmo(std::vector<Shape>& list, const Shape& place, int& selected, bool edit,
                          const EventGizmo::View& view, const char* label,
                          const ImVec4* fixedTint = nullptr, float fillScale = 1.f) {
    std::vector<SceneEvent> proxy(list.size());
    std::vector<ImVec4>     tints(list.size());
    for (size_t i = 0; i < list.size(); i++) {
        const Shape& s = list[i];
        proxy[i].shape = s.shape;
        proxy[i].cx = s.cx; proxy[i].cy = s.cy;
        proxy[i].radius = s.radius; proxy[i].halfW = s.halfW; proxy[i].halfH = s.halfH;
        tints[i] = fixedTint ? *fixedTint : ImVec4(s.color.r, s.color.g, s.color.b, 1.f);
    }
    SceneEvent proxyDefaults;
    proxyDefaults.shape = place.shape;
    int dummySel = -1;
    EventGizmo::Context c;
    c.view        = view;
    c.events      = &proxy;
    c.selected    = edit ? &selected : &dummySel;
    c.defaults    = &proxyDefaults;
    c.interactive = edit;
    c.showArrows  = false;
    c.colors      = &tints;
    c.label       = label;
    c.fillScale   = fillScale;
    const EventGizmo::Result rr = EventGizmo::Update(c);
    if (rr.deleted >= 0 && rr.deleted < static_cast<int>(list.size()))
        list.erase(list.begin() + rr.deleted);
    for (size_t i = 0; i < list.size() && i < proxy.size(); i++) {
        list[i].cx = proxy[i].cx; list[i].cy = proxy[i].cy;
        list[i].radius = proxy[i].radius;
        list[i].halfW = proxy[i].halfW; list[i].halfH = proxy[i].halfH;
    }
    if (rr.created && proxy.size() > list.size()) {
        const SceneEvent& p = proxy.back();
        Shape n = place;
        n.shape = p.shape; n.cx = p.cx; n.cy = p.cy;
        n.radius = p.radius; n.halfW = p.halfW; n.halfH = p.halfH;
        list.push_back(n);
    }
}

// ---- Setup-view particle preview ---------------------------------------------------------------
// While configuring, every particle the scene would spawn is drawn at its real size, spacing and
// color through the sim's instanced circle path, with no physics. Mirrors Init()'s layout rules.

constexpr int PreviewParticleCap = 100000; // the renderer's instance capacity (VulkanContext kMaxObjects)

// The groups the menu shows: the scene's own, or (before the menu seeds them) the spawn block.
static std::vector<SceneGroup> MenuPreviewGroups(const Scene& s, float halfW, float halfH) {
    if (s.useGroups) return s.groups;
    Scene tmp = s; // SpawnBlockAsGroup writes the grid counts back into its scene
    return { SpawnBlockAsGroup(tmp, halfW, halfH) };
}

// The color Init()'s spawnAt gives a particle spawned at (x, y) with group color `base`.
static Color PreviewParticleColor(const Scene& s, const PatternSampler& sampler, const Color& base, float x, float y) {
    Color c = base;
    if (Color pc; sampler.Sample(x, y, pc)) c = pc;
    if (s.blendColorRegions) {
        BlendColorRegions(s.colorRegions, x, y, c, s.blendStrength, s.blendRadius);
        return c;
    }
    for (const SceneColorRegion& r : s.colorRegions) {
        const float dx = x - r.cx, dy = y - r.cy;
        const bool inside = r.shape == EventShape::Circle
            ? dx * dx + dy * dy <= r.radius * r.radius
            : std::fabs(dx) <= r.halfW && std::fabs(dy) <= r.halfH;
        if (inside) c = r.color;
    }
    return c;
}

static void AddMenuPreviewParticles() {
    const Scene& s = menuConfig.scene;
    float halfW, halfH;
    MenuHalfExtents(s, halfW, halfH);
    const std::vector<SceneGroup> groups = MenuPreviewGroups(s, halfW, halfH);

    // The box every group fits in: the frame a color pattern is laid over (as in Init()).
    float boxMinX = 0.f, boxMinY = 0.f, boxMaxX = 0.f, boxMaxY = 0.f;
    bool  boxSet = false;
    for (const SceneGroup& g : groups) {
        const float hx = g.shape == EventShape::Circle ? g.radius : g.halfW;
        const float hy = g.shape == EventShape::Circle ? g.radius : g.halfH;
        boxMinX = boxSet ? std::min(boxMinX, g.cx - hx) : g.cx - hx;
        boxMaxX = boxSet ? std::max(boxMaxX, g.cx + hx) : g.cx + hx;
        boxMinY = boxSet ? std::min(boxMinY, g.cy - hy) : g.cy - hy;
        boxMaxY = boxSet ? std::max(boxMaxY, g.cy + hy) : g.cy + hy;
        boxSet = true;
    }
    const PatternSampler sampler(s.colorPattern, boxMinX, boxMinY, boxMaxX, boxMaxY,
                                 s.blendColorRegions ? s.blendStrength : 0.f, s.blendRadius);

    const float radius = s.particles.circleRadius;
    int budget = PreviewParticleCap;
    for (const SceneGroup& g : groups) {
        const Color base = g.customColor ? g.color : s.particles.circleColor;
        ForEachGroupPoint(g, GroupSpacing(g, radius), [&](float x, float y) {
            if (budget <= 0) return;
            if (InsideAnyObstacle(s.obstacles, x, y, radius)) return;
            budget--;
            app.AddCircle(x, y, radius, PreviewParticleColor(s, sampler, base, x, y));
        });
    }
}

// Keeps the smallest shape of `list` containing (x, y) in `best`, so a small obstacle inside a big group wins.
template <typename Shape>
static void PickSmallestHit(const std::vector<Shape>& list, float x, float y, int layer,
                            int& bestLayer, int& bestIndex, float& bestArea) {
    for (int i = 0; i < static_cast<int>(list.size()); i++) {
        const Shape& s = list[i];
        const float dx = x - s.cx, dy = y - s.cy;
        const bool  circle = s.shape == EventShape::Circle;
        const bool  inside = circle ? dx * dx + dy * dy <= s.radius * s.radius
                                    : std::fabs(dx) <= s.halfW && std::fabs(dy) <= s.halfH;
        const float area   = circle ? 3.14159265f * s.radius * s.radius : 4.f * s.halfW * s.halfH;
        if (inside && area < bestArea) { bestArea = area; bestLayer = layer; bestIndex = i; }
    }
}

// Setup view: double-clicking any group, obstacle, color area or event selects it and brings its
// tab to the front (a single click only works on the open tab's layer).
static void SelectShapeOnDoubleClick(const EventGizmo::View& view) {
    const ImGuiIO& io = ImGui::GetIO();
    // The raw per-frame flag, not IsMouseDoubleClicked(), which ImGui's key-owner system can veto.
    if (!io.MouseDoubleClicked[ImGuiMouseButton_Left]) return;
    const bool inView = io.MousePos.x >= view.clipMin.x && io.MousePos.x <= view.clipMax.x &&
                        io.MousePos.y >= view.clipMin.y && io.MousePos.y <= view.clipMax.y;
    if (io.WantCaptureMouse || !inView) return;

    const float wx =  (io.MousePos.x - view.origin.x) / view.scale;
    const float wy = -(io.MousePos.y - view.origin.y) / view.scale;
    Scene& s = menuConfig.scene;

    int bestLayer = -1, bestIndex = -1;
    float bestArea = FLT_MAX;
    PickSmallestHit(s.groups,       wx, wy, 0, bestLayer, bestIndex, bestArea);
    PickSmallestHit(s.obstacles,    wx, wy, 1, bestLayer, bestIndex, bestArea);
    PickSmallestHit(s.colorRegions, wx, wy, 2, bestLayer, bestIndex, bestArea);
    PickSmallestHit(s.events,       wx, wy, 3, bestLayer, bestIndex, bestArea);

    // Brings a docked panel's tab to the front (SetWindowFocus alone may not reselect it).
    auto showTab = [](const char* name) {
        ImGuiWindow* w = ImGui::FindWindowByName(name);
        if (!w) return;
        if (w->DockNode && w->DockNode->TabBar)
            w->DockNode->TabBar->NextSelectedTabId = w->TabId;
        ImGui::SetWindowFocus(name);
    };
    switch (bestLayer) {
        case 0: selectedGroup    = bestIndex; showTab("Groups###GroupsPanel");           break;
        case 1: selectedObstacle = bestIndex; showTab("Obstacles###ObstaclesPanel");     break;
        case 2: selectedRegion   = bestIndex; showTab("Persistent Color###ColorsPanel"); break;
        case 3: selectedEvent    = bestIndex; showTab("Events###RightPanel");            break;
        default: break;
    }
}

static void DrawSimOverlay(const Workspace::Frame& frame) {
    const ImGuiIO& io = ImGui::GetIO();
    const float fb = std::max(io.DisplayFramebufferScale.x, 0.01f); // framebuffer pixels per ImGui point

    // The sim renders into the free center area (see VulkanContext::SetSceneViewport); ignored while recording.
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
        // Pre-run preview: the container and each group's outline. The renderer draws the particles.
        const Scene& s = menuConfig.scene;
        float halfW, halfH;
        MenuHalfExtents(s, halfW, halfH);
        ImDrawList* dl = ImGui::GetBackgroundDrawList();
        dl->PushClipRect(frame.centerMin, frame.centerMax, true);
        const ImVec2 a(view.origin.x - halfW * view.scale, view.origin.y - halfH * view.scale);
        const ImVec2 b(view.origin.x + halfW * view.scale, view.origin.y + halfH * view.scale);
        dl->AddRect(a, b, Theme::U32(Theme::WithAlpha(Theme::TextDim, 0.8f)), 14.f, 0, 3.f);

        const ImU32 line = Theme::U32(ImVec4(0.88f, 0.88f, 0.90f, 0.45f)); // matches the group boxes
        for (const SceneGroup& g : MenuPreviewGroups(s, halfW, halfH)) {
            if (g.shape == EventShape::Circle) {
                const ImVec2 c(view.origin.x + g.cx * view.scale, view.origin.y - g.cy * view.scale);
                dl->AddCircle(c, g.radius * view.scale, line, 96, 1.5f);
            } else {
                const ImVec2 pa(view.origin.x + (g.cx - g.halfW) * view.scale, view.origin.y - (g.cy + g.halfH) * view.scale);
                const ImVec2 pb(view.origin.x + (g.cx + g.halfW) * view.scale, view.origin.y - (g.cy - g.halfH) * view.scale);
                dl->AddRect(pa, pb, line, 8.f, 0, 1.5f);
            }
        }

        const int count = SceneParticleCount(s);
        char buf[96];
        std::snprintf(buf, sizeof(buf), s.obstacles.empty() ? "%d particles" : "up to %d particles", count);
        if (count > PreviewParticleCap)
            std::snprintf(buf + std::strlen(buf), sizeof(buf) - std::strlen(buf), "  (preview shows %d)", PreviewParticleCap);
        dl->AddText(ImVec2(a.x + 14.f, a.y + 10.f), Theme::U32(Theme::TextDim), buf);
        dl->PopClipRect();
    }

    if (appState == AppState::Menu || (appState == AppState::Running && showEventAreas)) {
        if (appState == AppState::Running && ActiveScene.boundary.periodicX && cameraFollow) {
            view.cameraOffsetX = cameraOffsetX;
            view.wrapHalfWidth = ScreenHalfWidth();
        }
        // Setup only: persistent-color areas, edited through SceneEvent proxies. Interactive only
        // while the "Persistent Color" tab is open (the events gizmo is then draw-only).
        const bool editColors = appState == AppState::Menu && colorsTabActive;
        // Obstacles use the same proxy trick in their own tab.
        const bool editObstacles = appState == AppState::Menu && obstaclesTabActive;
        const bool editGroups    = appState == AppState::Menu && groupsTabActive;
        if (appState == AppState::Menu) {
            SelectShapeOnDoubleClick(view);
            // Light gray, thin fill, so the particles inside keep their own colors.
            static const ImVec4 GroupTint(0.88f, 0.88f, 0.90f, 1.f);
            RunShapeGizmo(menuConfig.scene.groups, placeGroupTemplate, selectedGroup, editGroups, view, "Group",
                          &GroupTint, 0.2f);
            RunShapeGizmo(menuConfig.scene.colorRegions, placeRegionTemplate, selectedRegion, editColors, view, "Persistent color area");
            RunShapeGizmo(menuConfig.scene.obstacles, placeObstacleTemplate, selectedObstacle, editObstacles, view, "Obstacle");
        }

        EventGizmo::Context ctx;
        ctx.view     = view;
        ctx.events   = &UiEvents();
        ctx.selected = &selectedEvent;
        ctx.defaults = &placeEventTemplate;
        ctx.fired    = &eventFired;
        ctx.simTime  = simTime;
        ctx.live     = appState == AppState::Running;
        ctx.interactive = !editColors && !editObstacles && !editGroups;
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
    if (simFaulted) { ImGui::SameLine(); Widgets::Chip(appState == AppState::Menu ? "LAST RUN DIVERGED" : "DIVERGED", Theme::Coral); }
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

// Drawn instead of DrawUi() when the worker is stuck in a substep and simMutex can't be taken. Touches no sim state.
static void DrawUnresponsiveUi() {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                                   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove;
    if (ImGui::Begin("##simNotResponding", nullptr, flags)) {
        ImGui::TextColored(Theme::Coral, "Simulation not responding");
        ImGui::TextDisabled("A single substep is taking unusually long. The window stays alive;");
        ImGui::TextDisabled("close it to quit, or wait for the sim to recover.");
    }
    ImGui::End();
}

// The single UI callback: top bar, dockspace, panels, then the sim-view overlay, in that order so
// the gizmo's io.WantCaptureMouse already reflects this frame's panels.
static void DrawUi() {
    // A different run state or scene means the old selection no longer points at the same event.
    static AppState lastState = AppState::Menu;
    static int      lastScene = -2;
    if (appState != lastState || menuConfig.sceneIndex != lastScene) {
        selectedEvent = -1;
        selectedObstacle = -1;
        selectedGroup = -1;
        lastState = appState;
        lastScene = menuConfig.sceneIndex;
    }

    if (Workspace::BeginTopBar()) DrawTopBarContents();
    Workspace::EndTopBar();
    const Workspace::Frame frame = Workspace::DockSpace();

    DrawLeftPanel();
    DrawEventsPanel();
    DrawColorsWindow();
    DrawGroupsWindow();
    DrawObstaclesWindow();
    DrawSimOverlay(frame);
}

Object& CreateObject(float x, float y, Renderable r) {
    objects.push_back({ .pos = Vec2(x, y), .renderable = r });
    return objects.back();
}

// The simulation container's half-extents: the scene's boundary fractions applied to the
// recording's resolution while recording (see VulkanContext::RecordingHalfWidth/Height), or the
// window size otherwise. SceneBoundary::halfWidthPx/halfHeightPx override both.
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
static bool Integrate(float subDt) {
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

int main(int, char**) {
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
