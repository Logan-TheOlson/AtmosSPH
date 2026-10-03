#pragma once
#include <volk.h>
#include <atomic>
#include <functional>
#include <string>
#include <vector>
#include "Config.h"
#include "util/Profiler.h"
#include "util/Recorder.h"

struct SDL_Window;

class VulkanContext {
public:
    bool Init(SDL_Window* window);
    void InitImGui(SDL_Window* window);
    void AddCircle(float cx, float cy, float radius, Color color);
    // Draws a hollow rectangular frame (border), `borderThickness` pixels wide, centered at
    // (cx, cy) with the given half-extents — used for boundary markers, not particles.
    void AddRectOutline(float cx, float cy, float halfWidth, float halfHeight, float borderThickness, Color color);
    void RenderFrame(float dt);
    void Shutdown();
    void SetUICallback(std::function<void()> cb)    { uiCallback_ = std::move(cb); }
    // Milliseconds the last RenderFrame() spent blocked on the GPU/vsync rather than working.
    float LastBlockedMs() const                     { return blockedMs_; }
    int   Width()      const { return (int)swapchainExtent.width; }
    int   Height()     const { return (int)swapchainExtent.height; }
    float HalfWidth()  const { return swapchainExtent.width  * 0.5f; }
    float HalfHeight() const { return swapchainExtent.height * 0.5f; }

    // The half-extents the simulation itself should measure its boundary against: pinned to the
    // active recording's fixed resolution (recordWidth_/recordHeight_) while one is in progress,
    // since that's the coordinate system the captured video frame actually uses — the live
    // swapchain can still change size underneath a batch render (see RecreateSwapchain), and
    // using HalfWidth()/HalfHeight() directly there would draw the boundary (and bounce particles
    // off walls) at a position that no longer matches where the fixed-size recorded frame puts
    // its own edges, splitting the border into visible stray lines partway through the video.
    // Live (non-batch) recording never actually hits this divergence — a swapchain resize there
    // stops the recording outright (see RecreateSwapchain) before hw/hh can drift — but pinning
    // unconditionally whenever IsRecording() is simpler than special-casing which capture mode.
    float RecordingHalfWidth()  const { return recorder_.IsActive() ? recordWidth_  * 0.5f : UsesSceneViewport() ? sceneViewport_.w * 0.5f : HalfWidth(); }
    float RecordingHalfHeight() const { return recorder_.IsActive() ? recordHeight_ * 0.5f : UsesSceneViewport() ? sceneViewport_.h * 0.5f : HalfHeight(); }

    // Restricts where the live (swapchain) scene pass draws to this framebuffer-pixel rect — the
    // free area left in the middle of the docked UI panels — with the world origin at the rect's
    // center instead of the window's. w/h = 0 means the full window. Ignored (full window) while a
    // recording is active: live capture reads the swapchain image from (0,0), and offscreen batch
    // rendering never goes through this path at all. See RecordingHalfWidth/Height.
    struct SceneViewport { float x = 0.f, y = 0.f, w = 0.f, h = 0.f; };
    void SetSceneViewport(float x, float y, float w, float h) { sceneViewport_ = { x, y, w, h }; }
    bool UsesSceneViewport() const { return sceneViewport_.w > 0.f && sceneViewport_.h > 0.f && !recorder_.IsActive(); }
    // Center of the rect the live scene is drawn into, in framebuffer pixels (window center if
    // the viewport isn't in use) — the world origin on screen.
    void SceneCenterPx(float& cx, float& cy) const {
        if (UsesSceneViewport()) { cx = sceneViewport_.x + sceneViewport_.w * 0.5f; cy = sceneViewport_.y + sceneViewport_.h * 0.5f; }
        else                     { cx = HalfWidth(); cy = HalfHeight(); }
    }

    // Captures the composited (scene + UI) swapchain image at `fps`, pacing capture against real
    // time (via RenderFrame's dt) rather than the render loop's actual rate. lengthSeconds <= 0
    // means "record until StopRecording() is called".
    bool  StartRecording(const std::string& outputPath, int fps, float lengthSeconds);
    void  StopRecording();
    bool  IsRecording() const { return recorder_.IsActive(); }
    float RecordedSeconds() const { return recordFps_ > 0 ? static_cast<float>(recordedFrames_) / recordFps_ : 0.f; }

    // Batch rendering (see Engine.cpp's AdvanceRendering) drives capture entirely through
    // RenderOffscreenFrame, but the main loop still calls RenderFrame once per real tick to keep
    // the progress overlay on screen. Without this flag RenderFrame's own capture gate would fire
    // too (recorder_.IsActive() is still true), copying that swapchain frame — which is just the
    // overlay, since circles/rects were already cleared by the offscreen pass — into the video as
    // a blank frame spliced in among the real ones. False while batch rendering; true otherwise.
    void  SetCaptureFromSwapchain(bool enable) { captureFromSwapchain_ = enable; }

    // Renders the scene (not UI) straight to an offscreen target and captures it, with no
    // swapchain/present involvement at all — unlike RenderFrame(), this can't be throttled by
    // vsync or by the OS deprioritizing an occluded/backgrounded window, since nothing is ever
    // shown on screen. Every call captures unconditionally (no fps-interval gating — the caller,
    // AdvanceRendering, already only calls this once per intended video frame) and auto-stops the
    // recording via the same length cap StartRecording() set up. A no-op if no recording is active.
    void RenderOffscreenFrame();

private:
    // SSBO instance data â€” world-space coords; NDC computed in vertex shader via push constants
    struct CircleData {
        float r, g, b, a;  // vec4 color  â€” offset  0
        float cx, cy;      // vec2 center â€” offset 16
        float radius;      //               offset 24
        float _pad;        //               offset 28, total 32
    };
    static_assert(sizeof(CircleData) == 32);

    // SSBO instance data for the hollow-frame rect shader (see shaders/rect.vert/.frag).
    struct RectData {
        float r, g, b, a;             // vec4 color  — offset  0
        float cx, cy;                 // vec2 rect.xy — offset 16
        float halfWidth, halfHeight;  // vec2 rect.zw — offset 24
        float borderThickness;        //               offset 32
        float _pad0, _pad1, _pad2;    //               offset 36..44, total 48
    };
    static_assert(sizeof(RectData) == 48);

    static constexpr uint32_t kMaxObjects = 100000;
    static constexpr uint32_t kMaxRects   = 64;

    SDL_Window*      sdlWindow{ nullptr };

    VkInstance       instance{ VK_NULL_HANDLE };
    VkSurfaceKHR     surface{ VK_NULL_HANDLE };
    VkPhysicalDevice physicalDevice{ VK_NULL_HANDLE };
    VkDevice         device{ VK_NULL_HANDLE };
    VkQueue          queue{ VK_NULL_HANDLE };
    uint32_t         queueFamily{ 0 };

    VkFormat         imageFormat{ VK_FORMAT_UNDEFINED };
    VkColorSpaceKHR  colorSpace{ VK_COLORSPACE_SRGB_NONLINEAR_KHR };
    VkExtent2D       swapchainExtent{};

    // Sample count the scene (circle/rect) pass renders at, chosen once in Init() from what the
    // device actually supports (falls back to VK_SAMPLE_COUNT_1_BIT, i.e. no MSAA, if 4x isn't
    // available). This anti-aliases more than each shape's own edge (see circle.frag's smoothstep
    // rim, which already handled that) — at high particle counts, particles shrink to a couple of
    // pixels across and their arrangement (e.g. the Vogel/Fibonacci spiral MakeSelfGravity's
    // circular spawn uses) becomes a near-regular point lattice; sampling it at only one point per
    // pixel aliases that regularity into a visible moiré "screen" pattern that per-shape edge AA
    // can't touch. Multisampling the whole pass softens that by sampling coverage several times
    // per pixel before resolving down to one color, the same fix a renderer would use for any
    // dense, regular geometry — see swapchainMsaaImage/offscreenMsaaImage below.
    VkSampleCountFlagBits sceneSampleCount{ VK_SAMPLE_COUNT_1_BIT };

    VkSwapchainKHR         swapchain{ VK_NULL_HANDLE };
    std::vector<VkImage>   images;
    std::vector<VkImageView> imageViews;

    // Multisampled color target the scene pass actually draws into when sceneSampleCount > 1,
    // resolved into imageViews[imageIndex] as part of the same pass (see RecordScenePass) — never
    // read from directly (no copy, no present), so its contents don't need to survive between
    // frames; recreated alongside imageViews whenever CreateSwapchain() runs. Left at VK_NULL_HANDLE
    // for the lifetime of the app if the device doesn't support sceneSampleCount's sample count.
    VkImage        swapchainMsaaImage{ VK_NULL_HANDLE };
    VkDeviceMemory swapchainMsaaMemory{ VK_NULL_HANDLE };
    VkImageView    swapchainMsaaView{ VK_NULL_HANDLE };

    // RenderOffscreenFrame()'s render target — same format as the swapchain, sized to match it at
    // StartRecording() time (see recordWidth_/recordHeight_) and recreated only if that changes.
    VkImage        offscreenImage{ VK_NULL_HANDLE };
    VkDeviceMemory offscreenMemory{ VK_NULL_HANDLE };
    VkImageView    offscreenView{ VK_NULL_HANDLE };
    VkExtent2D     offscreenExtent{};
    // offscreenImage's MSAA counterpart — same role/lifetime as swapchainMsaaImage above, just
    // sized to/recreated alongside the offscreen target instead of the swapchain.
    VkImage        offscreenMsaaImage{ VK_NULL_HANDLE };
    VkDeviceMemory offscreenMsaaMemory{ VK_NULL_HANDLE };
    VkImageView    offscreenMsaaView{ VK_NULL_HANDLE };
    // Whether offscreenImage has ever been rendered to — its very first barrier transitions from
    // UNDEFINED, every one after that from TRANSFER_SRC_OPTIMAL (where the previous frame's
    // capture copy left it).
    bool           offscreenEverRendered{ false };
    void EnsureOffscreenTarget(uint32_t w, uint32_t h);
    void DestroyOffscreenTarget();

    VkCommandPool   commandPool{ VK_NULL_HANDLE };
    VkCommandBuffer cb{ VK_NULL_HANDLE };

    VkSemaphore acquireSem{ VK_NULL_HANDLE };
    VkSemaphore renderSem{ VK_NULL_HANDLE };
    VkFence     fence{ VK_NULL_HANDLE };

    VkDescriptorSetLayout shapeDescSetLayout{ VK_NULL_HANDLE };
    VkDescriptorPool      descPool{ VK_NULL_HANDLE };
    VkDescriptorSet       circleDescSet{ VK_NULL_HANDLE };
    VkDescriptorSet       rectDescSet{ VK_NULL_HANDLE };

    VkBuffer       circleSSBO{ VK_NULL_HANDLE };
    VkDeviceMemory circleSSBOMemory{ VK_NULL_HANDLE };
    void*          circleMapped{ nullptr };

    VkBuffer       rectSSBO{ VK_NULL_HANDLE };
    VkDeviceMemory rectSSBOMemory{ VK_NULL_HANDLE };
    void*          rectMapped{ nullptr };

    VkPipelineLayout circlePipelineLayout{ VK_NULL_HANDLE };
    VkPipeline       circlePipeline{ VK_NULL_HANDLE };

    VkPipelineLayout rectPipelineLayout{ VK_NULL_HANDLE };
    VkPipeline       rectPipeline{ VK_NULL_HANDLE };

    std::vector<CircleData> circles;
    std::vector<RectData>   rects;

    SceneViewport   sceneViewport_{};
    std::function<void()> uiCallback_;
    float                 blockedMs_{ 0.f };

    // Frame recording: the final composited image is copied into one of a small ring of
    // host-visible readback buffers. Recorder's background thread reads a slot directly (an
    // fread of GPU-visible memory over PCIe can be slow) and calls its release callback when
    // done, which is what frees that slot for reuse — so that slow read never sits on the render
    // thread's critical path. Only one GPU copy is ever in flight (this renderer has a single
    // frame in flight generally), so kCaptureSlots' only job is to give the background reader a
    // few frames of grace before a slot it's still draining is needed again; if it falls behind
    // regardless, capture is skipped for that frame rather than stalling to wait for it.
    static constexpr int kCaptureSlots = 3;

    Recorder     recorder_;
    int          recordFps_{ 0 };
    float        recordLengthSeconds_{ 0.f };
    float        recordAccum_{ 0.f };
    uint32_t     recordedFrames_{ 0 };
    uint32_t     recordWidth_{ 0 }, recordHeight_{ 0 };
    VkDeviceSize captureBufferSize_{ 0 };
    // Whether captureMemories_'s type is HOST_COHERENT; if not, CPU reads need an explicit
    // vkInvalidateMappedMemoryRanges first (see CreateSSBO's memory-type selection).
    bool         captureMemoryCoherent_{ true };

    VkBuffer          captureBuffers_[kCaptureSlots]{};
    VkDeviceMemory    captureMemories_[kCaptureSlots]{};
    void*             captureMapped_[kCaptureSlots]{};
    std::atomic<bool> captureSlotBusy_[kCaptureSlots]{};
    int               captureWriteIndex_{ 0 };
    int               pendingCaptureSlot_{ -1 }; // slot the most recent submission copied into, not yet handed to Recorder
    bool              captureFromSwapchain_{ true }; // see SetCaptureFromSwapchain

    // Hands pendingCaptureSlot_ (if any) to Recorder and clears it. Only valid to call once the
    // GPU copy into that slot is known to have finished (i.e. after a fence wait or
    // vkDeviceWaitIdle) — called both per-frame and from StopRecording(), so a stop never drops
    // the frame the most recent RenderFrame() call just captured.
    void FlushPendingCapture();

    void CreateSwapchain();
    void RecreateSwapchain();
    // The circle/rect draw pass shared verbatim by RenderFrame() and RenderOffscreenFrame() —
    // clears `drawView` and draws the current circles/rects into it at `extent`, then clears both
    // vectors. Doesn't begin/end the command buffer or touch barriers; callers own those (including
    // transitioning `drawView`'s image to COLOR_ATTACHMENT_OPTIMAL beforehand).
    //
    // `resolveView` is VK_NULL_HANDLE when sceneSampleCount is 1 (no MSAA support): `drawView` is
    // then the final single-sample target and this behaves exactly like the pre-MSAA version — load
    // /clear/store it directly, no resolve. Otherwise `drawView` must be the matching MSAA target
    // (swapchainMsaaView/offscreenMsaaView) and `resolveView` the final single-sample image the
    // multisampled result resolves into as part of this same pass; the MSAA image itself is left
    // DONT_CARE since nothing ever reads it back.
    void RecordScenePass(VkImageView drawView, VkImageView resolveView, VkExtent2D extent,
                         const SceneViewport* viewport = nullptr);
    void CreateShapePipeline(const char* vertSpv, const char* fragSpv,
                             VkDescriptorSetLayout descSetLayout,
                             VkPipelineLayout& outLayout, VkPipeline& outPipeline);
    // `preferred` flags are matched on top of HOST_VISIBLE where available (falling back to
    // plain HOST_VISIBLE|HOST_COHERENT otherwise); pass HOST_CACHED for a readback buffer the
    // CPU will read from — device memory that's HOST_VISIBLE|HOST_COHERENT but not cached is
    // typically an uncached PCIe-mapped region, and CPU reads from it can be an order of
    // magnitude slower than reads from normal (cached) system memory.
    void CreateSSBO(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags preferred,
                    VkBuffer& buf, VkDeviceMemory& mem, void*& mapped, bool* outCoherent = nullptr);

    // Shared image+memory+view allocation for any of this class's device-local color attachments
    // (offscreenImage, and now the MSAA targets below) — `usage` distinguishes the ones that also
    // need to be copy-read (TRANSFER_SRC) from the MSAA ones that only ever get rendered into.
    void CreateColorImage(VkExtent2D extent, VkSampleCountFlagBits samples, VkImageUsageFlags usage,
                          VkImage& img, VkDeviceMemory& mem, VkImageView& view);
    // Destroys and nulls out an image created by CreateColorImage — a no-op if `img` is already
    // VK_NULL_HANDLE. Callers own waiting for the GPU to be done with it first.
    void DestroyColorImage(VkImage& img, VkDeviceMemory& mem, VkImageView& view);
};
