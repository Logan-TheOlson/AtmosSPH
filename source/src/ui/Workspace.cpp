#include "ui/Workspace.h"
#include <imgui_internal.h>

namespace {
    bool layoutResetRequested = false;

    void BuildDefaultLayout(ImGuiID dockId, const ImVec2& size) {
        ImGui::DockBuilderRemoveNode(dockId);
        ImGui::DockBuilderAddNode(dockId, ImGuiDockNodeFlags_DockSpace);
        ImGui::DockBuilderSetNodeSize(dockId, size);

        ImGuiID rest = dockId, center = 0;
        const ImGuiID left  = ImGui::DockBuilderSplitNode(rest, ImGuiDir_Left,  0.22f, nullptr, &rest);
        const ImGuiID right = ImGui::DockBuilderSplitNode(rest, ImGuiDir_Right, 0.24f, nullptr, &center);

        ImGui::DockBuilderDockWindow(Workspace::LeftPanelId,  left);
        ImGui::DockBuilderDockWindow(Workspace::RightPanelId, right);
        ImGui::DockBuilderFinish(dockId);
    }
}

void Workspace::RequestLayoutReset() { layoutResetRequested = true; }

bool Workspace::BeginTopBar() {
    const float height = ImGui::GetFrameHeight() + 26.f;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(18.f, 12.f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.f);
    return ImGui::BeginViewportSideBar("##TopBar", ImGui::GetMainViewport(), ImGuiDir_Up, height,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoNav);
}

void Workspace::EndTopBar() {
    ImGui::End();
    ImGui::PopStyleVar(2);
}

Workspace::Frame Workspace::DockSpace() {
    ImGuiViewport* vp = ImGui::GetMainViewport();
    // Passthru: the central node draws nothing, so the sim shows through and clicks reach it.
    const ImGuiID id = ImGui::DockSpaceOverViewport(0, vp, ImGuiDockNodeFlags_PassthruCentralNode);

    const ImGuiDockNode* node = ImGui::DockBuilderGetNode(id);
    if (layoutResetRequested || node == nullptr || node->IsLeafNode()) {
        // No saved layout (or only the bare root): build the default.
        BuildDefaultLayout(id, vp->WorkSize);
        layoutResetRequested = false;
    }

    Frame f{ vp->WorkPos, ImVec2(vp->WorkPos.x + vp->WorkSize.x, vp->WorkPos.y + vp->WorkSize.y) };
    if (const ImGuiDockNode* c = ImGui::DockBuilderGetCentralNode(id); c && c->Size.x > 1.f && c->Size.y > 1.f) {
        f.centerMin = c->Pos;
        f.centerMax = ImVec2(c->Pos.x + c->Size.x, c->Pos.y + c->Size.y);
    }
    return f;
}
