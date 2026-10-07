// Top bar, setup tabs, live/render panels and DrawUi(), the single UI callback.
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
#include "engine/SceneHelpers.h"
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


// The mode + gradient picker used by DrawAndSubmitFrame, shared by the live panel and the setup
// view so a batch render can set it before starting.
void DrawColorByControls(float contentWidth) {
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

// One labelled input row: the label in a fixed left column (wrapping if long), the widget filling
// the right. Call immediately before the widget and give it a "##"-hidden label.
void FormRow(float contentWidth, const char* label) {
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

// The simulated area: the video's resolution and the space the whole scene is laid out in. The
// view shows it scaled to fit; changing it scales the layout (groups, obstacles, ...) along.
void DrawResolutionSection(Scene& s, float contentWidth) {
    int w = s.boundary.resWidth, h = s.boundary.resHeight;
    bool changed = false;
    FormRow(contentWidth, "Width (px)");
    ImGui::InputInt("##ResWidth", &w, 0, 0);
    changed |= ImGui::IsItemDeactivatedAfterEdit();
    FormRow(contentWidth, "Height (px)");
    ImGui::InputInt("##ResHeight", &h, 0, 0);
    changed |= ImGui::IsItemDeactivatedAfterEdit();

    struct Preset { const char* label; int w, h; };
    static constexpr Preset presets[] = { { "720p", 1280, 720 }, { "1080p", 1920, 1080 }, { "1440p", 2560, 1440 }, { "4K", 3840, 2160 } };
    for (const Preset& p : presets) {
        if (ImGui::SmallButton(p.label)) { w = p.w; h = p.h; changed = true; }
        ImGui::SameLine();
    }
    ImGui::NewLine();

    if (changed) SetSceneResolution(s, w, h);
    ImGui::TextDisabled("Simulated area and video resolution. The view shows it scaled to fit;\n"
                         "changing it scales groups, obstacles, events and colors with it.");
}

void DrawBoundarySection(SceneBoundary& b, float contentWidth) {
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

void DrawExternalForceSection(ScenePhysics& p, float contentWidth) {
    FormRow(contentWidth, "Force (x, y)");
    ImGui::InputFloat2("##Force (x, y)", &p.force.x);
    ImGui::TextDisabled("Uniform acceleration on every particle (e.g. gravity), px/s^2.");
}

void DrawWallsSection(ScenePhysics& p, float contentWidth) {
    FormRow(contentWidth, "Restitution");
    ImGui::SliderFloat("##Restitution", &p.restitution, 0.f, 1.f, "%.2f");
    FormRow(contentWidth, "Friction");
    ImGui::SliderFloat("##Friction", &p.friction, 0.f, 1.f, "%.2f");
    ImGui::Checkbox("No-slip walls", &p.noSlipWalls);
    ImGui::TextDisabled("Applies to the container and to obstacles.");
}

void DrawIntegrationSection(ScenePhysics& p, float contentWidth) {
    FormRow(contentWidth, "Substeps");
    ImGui::InputInt("##Substeps", &p.substeps);
    p.substeps = std::max(p.substeps, 1);
    FormRow(contentWidth, "Force interval");
    ImGui::InputInt("##Force interval", &p.forceInterval);
    p.forceInterval = std::max(p.forceInterval, 1);
    ImGui::TextDisabled("Substeps per frame; forces are recomputed every <interval> substeps.");
}

const char* GravityMethodNames[] = { "Barnes-Hut", "Genuine 2D" };

void DrawGravitySection(SceneGravity& g, float contentWidth) {
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

const char* EosModelNames[] = { "WCSPH", "Polytropic" };
const char* ViscosityModelNames[] = { "Monaghan (artificial)", "Morris (physical)" };

// The fluid itself: equation of state, stiffness, rest density and smoothing radius.
void DrawFluidSection(SceneParticles& pr, float contentWidth) {
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

void DrawViscositySection(SceneParticles& pr, float contentWidth) {
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

// ================================================================================================
// Docked-panel UI: a top bar, a left panel (setup tabs / live controls / render progress by
// appState), a right panel (events) and the sim view in the center, which also hosts the event
// gizmos (see ui/EventGizmo.h). All drawn by DrawUi(), the single UI callback.
// ================================================================================================

// The event list the panels/gizmo edit: the menu's scene before a run, the live scene during one.
std::vector<SceneEvent>& UiEvents() {
    return appState == AppState::Menu ? menuConfig.scene.events : ActiveScene.events;
}

// A tinted, rounded "card" grouping related rows. Take widths from GetContentRegionAvail().
void BeginCard(const char* id) {
    ImGui::PushStyleColor(ImGuiCol_ChildBg, Theme::PanelAlt);
    ImGui::BeginChild(id, ImVec2(0.f, 0.f),
        ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AlwaysUseWindowPadding);
}
void EndCard() {
    ImGui::EndChild();
    ImGui::PopStyleColor();
}

// ---- Setup tabs (Menu state) ----------------------------------------------------------------

// Container half-extents as the setup view shows them (same rule as DrawSimOverlay's preview).
void MenuHalfExtents(const Scene& s, float& halfW, float& halfH) {
    halfW = ContainerHalfWidth(s.boundary);
    halfH = ContainerHalfHeight(s.boundary);
}

// Seeds the scene's editable groups from its spawn block (grid counts, circular, offset, packing).
// Called when a scene loads and whenever one of those block controls is edited.
void SyncGroupsFromSpawn(Scene& s) {
    float halfW, halfH;
    MenuHalfExtents(s, halfW, halfH);
    s.groups    = { SpawnBlockAsGroup(s, halfW, halfH) };
    s.useGroups = true;
}

// Which scene index menuConfig.scene was last seeded from (-1 = never); "Save Current as Preset"
// re-points it so saving doesn't reseed.
int lastSyncedSceneIndex = -1;

void DrawSceneTab() {
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
        static bool autoTuneSph = true;
        FormRow(cw, "Particle radius (px)");
        bool retune = ImGui::InputFloat("##Circle radius", &pr.circleRadius);
        pr.circleRadius = std::max(pr.circleRadius, 0.5f);
        if (ImGui::Checkbox("Auto-tune SPH to size", &autoTuneSph)) retune = true;
        ImGui::TextDisabled("Scales smoothing radius, target density, stiffness and viscosity so any\n"
                            "size behaves like the defaults, and refits group boxes to the lattice.");
        if (retune) {
            if (autoTuneSph) AutoTuneSph(pr);
            for (SceneGroup& g : menuConfig.scene.groups)
                SnapGroupToLattice(g, GroupSpacing(g, pr.circleRadius));
        }
        const float swatchSize = ImGui::GetFrameHeight();
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Particle color");
        ImGui::SameLine(cw - swatchSize);
        ImGui::ColorEdit4("##circlecolor", &pr.circleColor.r, ImGuiColorEditFlags_NoInputs);
        ImGui::TextDisabled("Groups can override the color; color areas and patterns paint over it.");
    }
    EndCard();

    Widgets::Caption("CONTAINER");
    if (ImGui::CollapsingHeader("Resolution", ImGuiTreeNodeFlags_DefaultOpen)) DrawResolutionSection(menuConfig.scene, w);
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

void DrawPhysicsTab() {
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
bool TitleBlank(const char* s) {
    for (; *s; ++s)
        if (!std::isspace(static_cast<unsigned char>(*s))) return false;
    return true;
}
bool MissingTitles(const RunSettings& r) {
    return (r.record && TitleBlank(r.title)) || (r.saveData && TitleBlank(r.dataTitle)) ||
           (r.saveProfile && TitleBlank(r.profileTitle));
}

bool showTitleErrors = false; // mark empty required titles (after a refused Start)
int  requestedTab    = -1;    // setup tab to switch to next frame (index into DrawSetupPanel's tabs)
constexpr int OutputTabIndex = 2;

bool RequireTitles(const RunSettings& r) {
    showTitleErrors = MissingTitles(r);
    if (showTitleErrors) requestedTab = OutputTabIndex;
    return !showTitleErrors;
}

void DrawTitleInput(float w, const char* label, const char* id, char* buf, size_t size) {
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

void DrawQueueSection();

// A run's output settings: particle coloring, what gets saved, and the run queue.
void DrawOutputTab() {
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

void DrawQueueSection() {
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

void DrawSetupPanel() {
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

void DrawLivePanel() {
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
void DrawRenderingPanel() {
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

void DrawLeftPanel() {
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

// ---- Top bar --------------------------------------------------------------------------------

void DrawTopBarContents() {
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
void DrawUnresponsiveUi() {
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
void DrawUi() {
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
