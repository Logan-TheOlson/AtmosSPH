#pragma once
#include <functional>
#include <string>
#include "Window.h"
#include "VulkanContext.h"
#include "util/Profiler.h"

class App {
public:
    void Init(const char* title, int w, int h);
    bool PollEvents();
    void RenderFrame(float dt);
    void Shutdown();
    void AddCircle(float cx, float cy, float radius, Color color);
    void AddRectOutline(float cx, float cy, float halfWidth, float halfHeight, float borderThickness, Color color);
    bool TakeSpaceToggle()                         { return window.TakeSpaceToggle(); }
    bool TakeStepForward()                          { return window.TakeStepForward(); }
    bool TakeStepForwardBig()                       { return window.TakeStepForwardBig(); }
    void SetUICallback(std::function<void()> cb)    { vk.SetUICallback(std::move(cb)); }
    float LastBlockedMs() const                     { return vk.LastBlockedMs(); }
    int   Width()      const;
    int   Height()     const;
    float HalfWidth()  const;
    float HalfHeight() const;
    // See VulkanContext::SetSimArea / SceneZoom.
    void  SetSimArea(float w, float h)   { vk.SetSimArea(w, h); }
    float SceneZoom()           const { return vk.SceneZoom(); }
    // See VulkanContext::SetSceneViewport / SceneCenterPx (framebuffer pixels).
    void  SetSceneViewport(float x, float y, float w, float h) { vk.SetSceneViewport(x, y, w, h); }
    void  SceneCenterPx(float& cx, float& cy) const { vk.SceneCenterPx(cx, cy); }

    // Starts (or restarts) recording the composited frame to <recordingsDir>/<title>.mp4 via
    // ffmpeg. lengthSeconds <= 0 records until StopRecording().
    // width x height is the video's resolution (see VulkanContext::StartRecording).
    bool  StartRecording(const std::string& title, int fps, float lengthSeconds, int width, int height);
    void  StopRecording()                { vk.StopRecording(); }
    bool  IsRecording()            const { return vk.IsRecording(); }
    float RecordedSeconds()        const { return vk.RecordedSeconds(); }
    // See VulkanContext::SetCaptureFromSwapchain.
    void  SetCaptureFromSwapchain(bool enable) { vk.SetCaptureFromSwapchain(enable); }
    // See VulkanContext::RenderOffscreenFrame.
    void  RenderOffscreenFrame()         { vk.RenderOffscreenFrame(); }

private:
    Window        window;
    VulkanContext vk;
};
