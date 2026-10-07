#include "ui/EventGizmo.h"
#include "ui/Theme.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace EventGizmo {
namespace {

    enum class Drag { None, Create, Move, Resize, Arrow };

    // Resize handles sit on each side (and, for rects, each corner) of an area's bounding box.
    // Dragging one moves that side/corner while the opposite stays put (Alt resizes about the
    // center). A circle is edited through its bounding square: only the side handles apply.
    enum class Handle { None, E, W, N, S, NE, NW, SE, SW };

    struct State {
        Drag      mode      = Drag::None;
        int       index     = -1;       // event being moved/resized/aimed (Create: unused)
        Handle    handle    = Handle::None; // Resize: which handle is being dragged
        float     startX = 0.f, startY = 0.f; // Create: press position (world)
        float     grabDx = 0.f, grabDy = 0.f; // Move: press position relative to the event center
        SceneEvent preview;              // Create: the area being dragged out
    };
    State st;

    constexpr float MinArrowLen   = 40.f;  // world px: arrow handle never collapses onto the center
    constexpr float MinCreateSize = 8.f;   // world px: a smaller drag is treated as a plain click

    float WrapX(float x, float hw) {
        const float span = 2.f * hw;
        if (span <= 0.f) return x;
        x = std::fmod(x + hw, span);
        if (x < 0.f) x += span;
        return x - hw;
    }

    ImVec2 ToScreen(const View& v, float wx, float wy) {
        const float dx = v.wrapHalfWidth > 0.f ? WrapX(wx - v.cameraOffsetX, v.wrapHalfWidth) : wx;
        return ImVec2(v.origin.x + dx * v.scale, v.origin.y - wy * v.scale);
    }
    void ToWorld(const View& v, const ImVec2& s, float& wx, float& wy) {
        wx = (s.x - v.origin.x) / v.scale + (v.wrapHalfWidth > 0.f ? v.cameraOffsetX : 0.f);
        wy = -(s.y - v.origin.y) / v.scale;
    }

    bool Inside(const SceneEvent& e, float wx, float wy) {
        const float dx = wx - e.cx, dy = wy - e.cy;
        if (e.shape == EventShape::Circle) return dx * dx + dy * dy <= e.radius * e.radius;
        return std::fabs(dx) <= e.halfW && std::fabs(dy) <= e.halfH;
    }

    float HalfW(const SceneEvent& e) { return e.shape == EventShape::Circle ? e.radius : e.halfW; }
    float HalfH(const SceneEvent& e) { return e.shape == EventShape::Circle ? e.radius : e.halfH; }

    // World position of a handle on the event's bounding box.
    void HandlePos(const SceneEvent& e, Handle h, float& x, float& y) {
        const float hw = HalfW(e), hh = HalfH(e);
        x = e.cx; y = e.cy;
        switch (h) {
            case Handle::E:  x += hw; break;
            case Handle::W:  x -= hw; break;
            case Handle::N:  y += hh; break;
            case Handle::S:  y -= hh; break;
            case Handle::NE: x += hw; y += hh; break;
            case Handle::NW: x -= hw; y += hh; break;
            case Handle::SE: x += hw; y -= hh; break;
            case Handle::SW: x -= hw; y -= hh; break;
            default: break;
        }
    }

    bool HandleApplies(const SceneEvent& e, Handle h) {
        const bool corner = h == Handle::NE || h == Handle::NW || h == Handle::SE || h == Handle::SW;
        return !(corner && e.shape == EventShape::Circle);
    }

    constexpr Handle AllHandles[] = { Handle::E, Handle::W, Handle::N, Handle::S,
                                      Handle::NE, Handle::NW, Handle::SE, Handle::SW };

    ImGuiMouseCursor HandleCursor(Handle h) {
        switch (h) {
            case Handle::E: case Handle::W:   return ImGuiMouseCursor_ResizeEW;
            case Handle::N: case Handle::S:   return ImGuiMouseCursor_ResizeNS;
            case Handle::NE: case Handle::SW: return ImGuiMouseCursor_ResizeNESW; // "/" on screen
            case Handle::NW: case Handle::SE: return ImGuiMouseCursor_ResizeNWSE; // "\\" on screen
            default:                          return ImGuiMouseCursor_Arrow;
        }
    }

    // Moves the dragged side/corner to the mouse, keeping the opposite one fixed (or mirroring
    // about the center with `symmetric`), with a minimum span.
    void ApplyHandleDrag(SceneEvent& e, Handle h, float mx, float my, bool symmetric) {
        constexpr float MinHalf = 4.f;
        float left = e.cx - HalfW(e), right = e.cx + HalfW(e);
        float bottom = e.cy - HalfH(e), top = e.cy + HalfH(e);

        const bool moveE = h == Handle::E || h == Handle::NE || h == Handle::SE;
        const bool moveW = h == Handle::W || h == Handle::NW || h == Handle::SW;
        const bool moveN = h == Handle::N || h == Handle::NE || h == Handle::NW;
        const bool moveS = h == Handle::S || h == Handle::SE || h == Handle::SW;

        if (symmetric) {
            if (moveE || moveW) { const float half = std::max(std::fabs(mx - e.cx), MinHalf); left = e.cx - half; right = e.cx + half; }
            if (moveN || moveS) { const float half = std::max(std::fabs(my - e.cy), MinHalf); bottom = e.cy - half; top = e.cy + half; }
        } else {
            if (moveE) right  = std::max(mx, left + 2.f * MinHalf);
            if (moveW) left   = std::min(mx, right - 2.f * MinHalf);
            if (moveN) top    = std::max(my, bottom + 2.f * MinHalf);
            if (moveS) bottom = std::min(my, top - 2.f * MinHalf);
        }

        if (e.shape == EventShape::Circle) {
            // The dragged axis sets the diameter; the other follows.
            const bool horizontal = moveE || moveW;
            const float span = horizontal ? right - left : top - bottom;
            e.radius = span * 0.5f;
            if (horizontal) e.cx = (left + right) * 0.5f;
            else            e.cy = (bottom + top) * 0.5f;
        } else {
            e.halfW = (right - left) * 0.5f; e.cx = (left + right) * 0.5f;
            e.halfH = (top - bottom) * 0.5f; e.cy = (bottom + top) * 0.5f;
        }
    }

    void ArrowTip(const SceneEvent& e, float& tx, float& ty) {
        const float len = std::sqrt(e.accel.x * e.accel.x + e.accel.y * e.accel.y);
        float dirX = 1.f, dirY = 0.f;
        if (len > 1e-3f) { dirX = e.accel.x / len; dirY = e.accel.y / len; }
        const float px = std::max(len * ArrowPxPerUnit, MinArrowLen);
        tx = e.cx + dirX * px;
        ty = e.cy + dirY * px;
    }

    ImVec4 EventColor(const SceneEvent& e) { return e.mode == EventMode::Once ? Theme::Amber : e.mode == EventMode::Field ? Theme::Teal : Theme::Violet; }

    // The push arrow/handle belongs to uniform-acceleration events only (a field has no single direction).
    bool HasArrow(const Context& c, const SceneEvent& e) { return c.showArrows && e.mode != EventMode::Field; }

    bool IsActive(const Context& c, size_t i) {
        const SceneEvent& e = (*c.events)[i];
        if (!c.live) return false;
        if (e.mode == EventMode::Once) return c.fired && i < c.fired->size() && (*c.fired)[i];
        return c.simTime >= e.startTime && (e.duration <= 0.f || c.simTime < static_cast<double>(e.startTime) + e.duration);
    }

    float Dist(float ax, float ay, float bx, float by) { return std::hypot(ax - bx, ay - by); }

    void DrawArrow(ImDrawList* dl, const ImVec2& from, const ImVec2& to, ImU32 col, float thickness, float head = 12.f) {
        dl->AddLine(from, to, col, thickness);
        const float dx = to.x - from.x, dy = to.y - from.y;
        const float len = std::hypot(dx, dy);
        if (len < 1.f) return;
        const float ux = dx / len, uy = dy / len;
        dl->AddTriangleFilled(to,
            ImVec2(to.x - ux * head - uy * head * 0.55f, to.y - uy * head + ux * head * 0.55f),
            ImVec2(to.x - ux * head + uy * head * 0.55f, to.y - uy * head - ux * head * 0.55f), col);
    }

    void DrawArea(ImDrawList* dl, const View& v, const SceneEvent& e, ImU32 fill, ImU32 line, float thickness) {
        const ImVec2 c = ToScreen(v, e.cx, e.cy);
        if (e.shape == EventShape::Circle) {
            dl->AddCircleFilled(c, e.radius * v.scale, fill, 64);
            dl->AddCircle(c, e.radius * v.scale, line, 64, thickness);
        } else {
            const ImVec2 a(c.x - e.halfW * v.scale, c.y - e.halfH * v.scale);
            const ImVec2 b(c.x + e.halfW * v.scale, c.y + e.halfH * v.scale);
            const float round = std::min(12.f, std::min(e.halfW, e.halfH) * v.scale * 0.5f);
            dl->AddRectFilled(a, b, fill, round);
            dl->AddRect(a, b, line, round, 0, thickness);
        }
    }

    void DrawLabel(ImDrawList* dl, const ImVec2& pos, const char* text, const ImVec4& color) {
        const ImVec2 ts = ImGui::CalcTextSize(text);
        const ImVec2 pad(8.f, 3.f);
        const ImVec2 a(pos.x, pos.y), b(pos.x + ts.x + pad.x * 2.f, pos.y + ts.y + pad.y * 2.f);
        dl->AddRectFilled(a, b, Theme::U32(Theme::WithAlpha(Theme::Panel, 0.88f)), (b.y - a.y) * 0.5f);
        dl->AddText(ImVec2(a.x + pad.x, a.y + pad.y), Theme::U32(color), text);
    }

    void DrawHandle(ImDrawList* dl, const ImVec2& p, const ImVec4& color, bool hot) {
        const float r = ImGui::GetFontSize() * (hot ? 0.40f : 0.32f);
        dl->AddCircleFilled(p, r, Theme::U32(Theme::Panel), 24);
        dl->AddCircleFilled(p, r - 2.f, Theme::U32(hot ? Theme::AccentHov : color), 24);
    }

    bool NearHandle(const View& v, float hx, float hy, const ImVec2& mouse) {
        const ImVec2 s = ToScreen(v, hx, hy);
        return std::hypot(mouse.x - s.x, mouse.y - s.y) <= ImGui::GetFontSize() * 0.7f;
    }

} // namespace

Result Update(const Context& ctx) {
    Result res;
    if (!ctx.events || !ctx.selected) return res;
    std::vector<SceneEvent>& events = *ctx.events;
    int& selected = *ctx.selected;
    if (selected >= static_cast<int>(events.size())) selected = -1;

    const View& v = ctx.view;
    ImGuiIO& io = ImGui::GetIO();
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    dl->PushClipRect(v.clipMin, v.clipMax, true);

    const ImVec2 mouse = io.MousePos;
    const bool mouseInView = mouse.x >= v.clipMin.x && mouse.x <= v.clipMax.x &&
                             mouse.y >= v.clipMin.y && mouse.y <= v.clipMax.y;
    const bool canStart = ctx.interactive && st.mode == Drag::None && mouseInView && !io.WantCaptureMouse;
    float mwx = 0.f, mwy = 0.f;
    ToWorld(v, mouse, mwx, mwy);

    // ---- Hit testing (for hover feedback and to start a drag) ----------------------------------
    enum class Hit { None, Body, Resize, Arrow } hit = Hit::None;
    int hitIndex = -1;
    Handle hitHandle = Handle::None;
    if (ctx.interactive && st.mode == Drag::None && mouseInView && !io.WantCaptureMouse) {
        if (selected >= 0) {
            const SceneEvent& e = events[selected];
            float tx, ty;
            ArrowTip(e, tx, ty);
            if (HasArrow(ctx, e) && NearHandle(v, tx, ty, mouse)) { hit = Hit::Arrow; hitIndex = selected; }
            else for (Handle h : AllHandles) {
                if (!HandleApplies(e, h)) continue;
                float hx, hy;
                HandlePos(e, h, hx, hy);
                if (NearHandle(v, hx, hy, mouse)) { hit = Hit::Resize; hitIndex = selected; hitHandle = h; break; }
            }
        }
        if (hit == Hit::None) {
            // The topmost (last-created) area wins; the selected one is tried first.
            if (selected >= 0 && Inside(events[selected], mwx, mwy)) { hit = Hit::Body; hitIndex = selected; }
            else for (int i = static_cast<int>(events.size()) - 1; i >= 0; i--)
                if (Inside(events[i], mwx, mwy)) { hit = Hit::Body; hitIndex = i; break; }
        }
    }

    // ---- Input state machine -------------------------------------------------------------------
    if (canStart && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        if (hit == Hit::Arrow)       { st.mode = Drag::Arrow;  st.index = hitIndex; }
        else if (hit == Hit::Resize) { st.mode = Drag::Resize; st.index = hitIndex; st.handle = hitHandle; }
        else if (hit == Hit::Body) {
            selected = hitIndex;
            st.mode = Drag::Move; st.index = hitIndex;
            st.grabDx = mwx - events[hitIndex].cx;
            st.grabDy = mwy - events[hitIndex].cy;
        } else if (ctx.defaults) {
            st.mode = Drag::Create; st.index = -1;
            st.startX = mwx; st.startY = mwy;
            st.preview = *ctx.defaults;
            st.preview.cx = mwx; st.preview.cy = mwy;
            st.preview.radius = 0.f; st.preview.halfW = 0.f; st.preview.halfH = 0.f;
            selected = -1;
        }
    }

    if (ctx.interactive && st.mode != Drag::None) {
        if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
            st.mode = Drag::None;
        } else if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            if (st.mode == Drag::Create) {
                st.preview.radius = Dist(mwx, mwy, st.startX, st.startY);
                st.preview.halfW  = std::fabs(mwx - st.startX);
                st.preview.halfH  = std::fabs(mwy - st.startY);
            } else if (st.index >= 0 && st.index < static_cast<int>(events.size())) {
                SceneEvent& e = events[st.index];
                if (st.mode == Drag::Move) {
                    e.cx = mwx - st.grabDx; e.cy = mwy - st.grabDy;
                } else if (st.mode == Drag::Resize) {
                    ApplyHandleDrag(e, st.handle, mwx, mwy, io.KeyAlt);
                } else if (st.mode == Drag::Arrow) {
                    e.accel = Vec2((mwx - e.cx) / ArrowPxPerUnit, (mwy - e.cy) / ArrowPxPerUnit);
                    // Snap to whole units so the numbers shown elsewhere stay readable.
                    e.accel.x = std::round(e.accel.x / 5.f) * 5.f;
                    e.accel.y = std::round(e.accel.y / 5.f) * 5.f;
                }
                res.changed = true;
            }
        } else {
            // Mouse released: commit a created area (if it's bigger than a click), end any drag.
            if (st.mode == Drag::Create) {
                const SceneEvent& p = st.preview;
                const float size = p.shape == EventShape::Circle ? p.radius : std::max(p.halfW, p.halfH);
                if (size >= MinCreateSize) {
                    SceneEvent e = p;
                    if (e.shape == EventShape::Rect) { e.halfW = std::max(e.halfW, 4.f); e.halfH = std::max(e.halfH, 4.f); }
                    e.startTime = ctx.live ? static_cast<float>(ctx.simTime) : ctx.defaults->startTime;
                    events.push_back(e);
                    selected = static_cast<int>(events.size()) - 1;
                    res.created = true;
                }
            }
            st.mode = Drag::None;
        }
    }

    if (ctx.interactive && selected >= 0 && !io.WantTextInput && st.mode == Drag::None &&
        (ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace))) {
        events.erase(events.begin() + selected);
        res.deleted = selected;
        selected = -1;
    }

    // ---- Drawing -------------------------------------------------------------------------------
    for (size_t i = 0; i < events.size(); i++) {
        const SceneEvent& e = events[i];
        const bool isSel   = static_cast<int>(i) == selected;
        const bool isHover = hit != Hit::None && hitIndex == static_cast<int>(i);
        const bool active  = IsActive(ctx, i);
        const bool   tinted = ctx.colors && i < ctx.colors->size();
        const ImVec4 col   = tinted ? (*ctx.colors)[i] : EventColor(e);

        const float fillA = (tinted ? (isSel || isHover ? 0.55f : 0.40f)
                                    : active ? 0.30f : (isSel || isHover ? 0.20f : 0.12f)) * ctx.fillScale;
        DrawArea(dl, v, e, Theme::U32(Theme::WithAlpha(col, fillA)),
                 Theme::U32(Theme::WithAlpha(col, isSel ? 1.f : 0.75f)), isSel ? 3.f : 2.f);

        // Index chip at the area's top-left.
        const ImVec2 c = ToScreen(v, e.cx, e.cy);
        const float topLeftX = e.shape == EventShape::Circle ? c.x - e.radius * v.scale * 0.7f : c.x - e.halfW * v.scale;
        const float topLeftY = e.shape == EventShape::Circle ? c.y - e.radius * v.scale * 0.7f : c.y - e.halfH * v.scale;
        char buf[48];
        std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(i) + 1);
        DrawLabel(dl, ImVec2(topLeftX + 6.f, topLeftY + 6.f), buf, col);

        if (e.mode == EventMode::Field) {
            // Sample the field on a 5x5 lattice over the bounding box and draw its direction.
            const float bw = e.shape == EventShape::Circle ? e.radius : e.halfW;
            const float bh = e.shape == EventShape::Circle ? e.radius : e.halfH;
            const ImU32 gc = Theme::U32(Theme::WithAlpha(col, isSel ? 1.f : 0.8f));
            for (int iy = 0; iy < 5; iy++)
                for (int ix = 0; ix < 5; ix++) {
                    const float wx = e.cx + (ix - 2) * 0.4f * bw, wy = e.cy + (iy - 2) * 0.4f * bh;
                    float ax, ay;
                    if (!EventFieldAccel(e, wx, wy, ax, ay)) continue;
                    const float len = std::hypot(ax, ay);
                    if (len < 1e-3f) continue;
                    const float scr = 9.f * std::min(1.f, 0.35f + len / std::max(std::fabs(e.strength), 1.f) * 0.65f);
                    const ImVec2 f = ToScreen(v, wx, wy);
                    DrawArrow(dl, ImVec2(f.x - ax / len * scr, f.y + ay / len * scr),
                              ImVec2(f.x + ax / len * scr, f.y - ay / len * scr), gc, 1.5f, 5.f);
                }
        }

        // Push direction/strength arrow.
        float tx, ty;
        ArrowTip(e, tx, ty);
        const bool aimed = HasArrow(ctx, e) && (e.accel.x != 0.f || e.accel.y != 0.f);
        if (aimed) DrawArrow(dl, c, ToScreen(v, tx, ty), Theme::U32(Theme::WithAlpha(col, isSel ? 1.f : 0.85f)), isSel ? 3.f : 2.f);

        if (isSel) {
            for (Handle h : AllHandles) {
                if (!HandleApplies(e, h)) continue;
                float hx, hy;
                HandlePos(e, h, hx, hy);
                const bool hot = (hit == Hit::Resize && hitHandle == h) || (st.mode == Drag::Resize && st.handle == h);
                DrawHandle(dl, ToScreen(v, hx, hy), col, hot);
            }
            if (HasArrow(ctx, e)) {
                DrawHandle(dl, ToScreen(v, tx, ty), Theme::Accent, hit == Hit::Arrow || st.mode == Drag::Arrow);

                const float mag = std::hypot(e.accel.x, e.accel.y);
                std::snprintf(buf, sizeof(buf), e.mode == EventMode::Once ? "%.0f px/s kick" : "%.0f px/s^2", mag);
                const ImVec2 ts = ToScreen(v, tx, ty);
                DrawLabel(dl, ImVec2(ts.x + 12.f, ts.y - 12.f), buf, col);
            }
        }
    }

    if (ctx.interactive && st.mode == Drag::Create) {
        DrawArea(dl, v, st.preview, Theme::U32(Theme::WithAlpha(Theme::Accent, 0.16f)), Theme::U32(Theme::Accent), 2.f);
    }

    dl->PopClipRect();

    // Cursor feedback + tooltip (only while nothing is being dragged).
    if (ctx.interactive && st.mode == Drag::None && hit != Hit::None) {
        ImGui::SetMouseCursor(hit == Hit::Body   ? ImGuiMouseCursor_ResizeAll
                            : hit == Hit::Resize ? HandleCursor(hitHandle)
                                                 : ImGuiMouseCursor_Hand);
        if (hit == Hit::Body && hitIndex >= 0 && !ctx.showArrows) {
            ImGui::SetTooltip("%s %d\nDrag to move  -  Del to delete", ctx.label, hitIndex + 1);
        } else if (hit == Hit::Body && hitIndex >= 0) {
            const SceneEvent& e = events[hitIndex];
            ImGui::SetTooltip("Event %d  -  %s  -  %s\nstart %.1fs%s\nDrag to move  -  Del to delete",
                hitIndex + 1, e.shape == EventShape::Circle ? "circle" : "rect",
                e.mode == EventMode::Once ? "one-shot kick" : e.mode == EventMode::Field ? "force field" : "continuous push", e.startTime,
                e.mode != EventMode::Once && e.duration > 0.f ? " (limited duration)" : "");
        }
    } else if (ctx.interactive && st.mode != Drag::None) {
        ImGui::SetMouseCursor(st.mode == Drag::Create ? ImGuiMouseCursor_Arrow
                            : st.mode == Drag::Move   ? ImGuiMouseCursor_ResizeAll
                            : st.mode == Drag::Resize ? HandleCursor(st.handle)
                                                      : ImGuiMouseCursor_Hand);
    }
    return res;
}

} // namespace EventGizmo
