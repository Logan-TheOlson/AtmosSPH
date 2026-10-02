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
    bool TakeF1Toggle()                             { return window.TakeF1Toggle(); }
    bool TakeSpaceToggle()                          { return window.TakeSpaceToggle(); }
    bool TakeStepForward()                          { return window.TakeStepForward(); }
    bool TakeStepForwardBig()                       { return window.TakeStepForwardBig(); }
    void SetProfilerOpen(bool open)                 { vk.SetProfilerOpen(open); }
    void SetProfilerStats(const Profiler::Stats& s) { vk.SetProfilerStats(s); }
    void SetObjectCount(int n)                      { vk.SetObjectCount(n); }
    void SetKineticEnergy(float ke)                 { vk.SetKineticEnergy(ke); }
    void SetUICallback(std::function<void()> cb)    { vk.SetUICallback(std::move(cb)); }
    int   Width()      const;
    int   Height()     const;
    float HalfWidth()  const;
    float HalfHeight() const;
    // See VulkanContext::RecordingHalfWidth/Height — what the simulation's own boundary math
    // should use instead of HalfWidth()/HalfHeight() while a recording is active.
    float RecordingHalfWidth()  const { return vk.RecordingHalfWidth(); }
    float RecordingHalfHeight() const { return vk.RecordingHalfHeight(); }
    // See VulkanContext::SetSceneViewport / SceneCenterPx (framebuffer pixels).
    void  SetSceneViewport(float x, float y, float w, float h) { vk.SetSceneViewport(x, y, w, h); }
    void  SceneCenterPx(float& cx, float& cy) const { vk.SceneCenterPx(cx, cy); }

    // Starts (or restarts) recording the composited frame to <recordingsDir>/<title>.mp4 via
    // ffmpeg. lengthSeconds <= 0 means "until StopRecording() is called".
    bool  StartRecording(const std::string& title, int fps, float lengthSeconds);
    void  StopRecording()                { vk.StopRecording(); }
    bool  IsRecording()            const { return vk.IsRecording(); }
    float RecordedSeconds()        const { return vk.RecordedSeconds(); }
    // See VulkanContext::SetCaptureFromSwapchain — must be set false while batch rendering so the
    // per-tick overlay-refresh RenderFrame() call doesn't splice blank frames into the video.
    void  SetCaptureFromSwapchain(bool enable) { vk.SetCaptureFromSwapchain(enable); }
    // See VulkanContext::RenderOffscreenFrame — draws+captures a video frame with no swapchain or
    // window involvement at all, so a batch render can't be throttled by vsync or by the window
    // being occluded/backgrounded.
    void  RenderOffscreenFrame()         { vk.RenderOffscreenFrame(); }

private:
    Window        window;
    VulkanContext vk;
};
