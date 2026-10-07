// Right-panel and floating editors (events, color areas, obstacles, groups) and the sim-view overlay.
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

// Persistent-color patches (see SceneColorRegion), edited like events: panel + drag gizmo.
SceneColorRegion placeRegionTemplate;
int              selectedRegion = -1;
// True on frames where the "Persistent Color" window is the visible tab of the right dock node.
bool             colorsTabActive = false;

// Solid obstacles (see SceneObstacle): same editing model as the color areas.
SceneObstacle    placeObstacleTemplate;
int              selectedObstacle = -1;
bool             obstaclesTabActive = false;

// Particle groups (see SceneGroup): the spawn blocks, edited like obstacles.
SceneGroup       placeGroupTemplate;
int              selectedGroup = -1;
bool             groupsTabActive = false;

// Shared by SceneColorRegion, SceneObstacle and SceneGroup (groups draw their own optional-color row).
template <typename Shape>
void DrawColorRegionFields(Shape& r, float contentWidth, bool withColor = true) {
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

bool       showEventAreas     = true;  // draw event gizmos over the live view (never recorded)
SceneEvent placeEventTemplate = [] { SceneEvent e; e.accel = Vec2(400.f, 0.f); return e; }();

const char* EventShapeNames[] = { "Circle", "Rect" };
const char* EventModeNames[]  = { "Continuous (acceleration)", "Once (velocity kick)", "Vector field" };
const char* FieldKindNames[]  = { "Curl (swirl)", "Attract (to center)", "Repel (from center)" };

void DrawEventFields(SceneEvent& e, float contentWidth) {
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

// Index into UiEvents() of the event the right panel inspects and the gizmo highlights.
int selectedEvent = -1;

// ---- Events panel (right) -------------------------------------------------------------------

// Shape/mode/push/duration used for areas created by dragging in the sim view or "+ New event".
void DrawEventDefaults(float w) {
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

void RemoveEventAt(std::vector<SceneEvent>& events, int index) {
    if (index < 0 || index >= static_cast<int>(events.size())) return;
    events.erase(events.begin() + index);
    if (appState != AppState::Menu && index < static_cast<int>(eventFired.size()))
        eventFired.erase(eventFired.begin() + index);
    selectedEvent = -1;
}

// Result of the async "Browse..." dialog; SDL may call back on another thread, so it is handed
// over through a mutex.
std::mutex  pickedImageMutex;
std::string pickedImagePath;

void SDLCALL OnImagePicked(void*, const char* const* files, int) {
    if (files && files[0]) {
        std::lock_guard<std::mutex> lock(pickedImageMutex);
        pickedImagePath = files[0];
    }
}

void DrawColorPatternSection(SceneColorPattern& p, float w) {
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
void DrawColorRegionsPanel() {
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
void DrawColorsWindow() {
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
void DrawObstaclesPanel() {
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
void DrawGroupsPanel() {
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
        if (ImGui::SliderFloat("##groupSpacing", &g.spacingScale, 0.5f, 2.0f, "%.2f"))
            SnapGroupToLattice(g, GroupSpacing(g, circleR));
        const float spacing = GroupSpacing(g, circleR);
        if (g.shape != EventShape::Circle) {
            // Columns/rows are always editable by number: typing one sizes the box to that grid.
            // Fix size additionally keeps the box locked to it (handle drags, packing, particle size).
            if (!g.fixSize) {
                int nx, ny;
                GroupGrid(g, spacing, nx, ny);
                if (nx > 0) { g.cols = nx; g.rows = ny; }
            }
            FormRow(cw, "Columns");
            const bool colsEdited = ImGui::InputInt("##groupCols", &g.cols, 1, 10);
            FormRow(cw, "Rows");
            const bool rowsEdited = ImGui::InputInt("##groupRows", &g.rows, 1, 10);
            ImGui::Checkbox("Fix size", &g.fixSize);
            if (colsEdited || rowsEdited || g.fixSize) {
                const bool wasFixed = g.fixSize;
                g.fixSize = true;
                FitGroupToGrid(g, spacing);
                g.fixSize = wasFixed;
            }
        }
        // Editing the count resizes the block at the current spacing (the view updates live).
        int count = GroupParticleCount(g, spacing);
        FormRow(cw, "Particle count");
        ImGui::BeginDisabled(g.fixSize && g.shape != EventShape::Circle);
        if (ImGui::InputInt("##groupCount", &count, 0, 0))
            ResizeGroupToCount(g, spacing, count);
        ImGui::EndDisabled();
        // Fits the lattice to the box at the current spacing and particle size: rounds the box to
        // whole cells, so the count becomes exactly what fills it (a circle already fills its disk).
        if (ImGui::Button("Fill group", ImVec2(cw, 0.f))) {
            SnapGroupToLattice(g, spacing);
            count = GroupParticleCount(g, spacing);
        }
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

void DrawGroupsWindow() {
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
void DrawObstaclesWindow() {
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

void DrawEventsPanel() {
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
void RunShapeGizmo(std::vector<Shape>& list, const Shape& place, int& selected, bool edit,
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
std::vector<SceneGroup> MenuPreviewGroups(const Scene& s, float halfW, float halfH) {
    if (s.useGroups) return s.groups;
    Scene tmp = s; // SpawnBlockAsGroup writes the grid counts back into its scene
    return { SpawnBlockAsGroup(tmp, halfW, halfH) };
}

// The color Init()'s spawnAt gives a particle spawned at (x, y) with group color `base`.
Color PreviewParticleColor(const Scene& s, const PatternSampler& sampler, const Color& base, float x, float y) {
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

void AddMenuPreviewParticles() {
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
void PickSmallestHit(const std::vector<Shape>& list, float x, float y, int layer,
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
void SelectShapeOnDoubleClick(const EventGizmo::View& view) {
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

void DrawSimOverlay(const Workspace::Frame& frame) {
    const ImGuiIO& io = ImGui::GetIO();
    const float fb = std::max(io.DisplayFramebufferScale.x, 0.01f); // framebuffer pixels per ImGui point

    // The sim renders into the free center area (see VulkanContext::SetSceneViewport); ignored while recording.
    app.SetSceneViewport(frame.centerMin.x * fb, frame.centerMin.y * fb,
                         (frame.centerMax.x - frame.centerMin.x) * fb,
                         (frame.centerMax.y - frame.centerMin.y) * fb);
    // The view shows the scene's simulated area scaled to fit (VulkanContext::SceneZoom).
    {
        const SceneBoundary& area = (appState == AppState::Menu ? menuConfig.scene : ActiveScene).boundary;
        app.SetSimArea(static_cast<float>(area.resWidth), static_cast<float>(area.resHeight));
    }
    float cxPx = 0.f, cyPx = 0.f;
    app.SceneCenterPx(cxPx, cyPx);

    EventGizmo::View view;
    view.origin  = ImVec2(cxPx / fb, cyPx / fb);
    view.scale   = app.SceneZoom() / fb;
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
