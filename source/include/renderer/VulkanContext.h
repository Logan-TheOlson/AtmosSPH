#pragma once
#include <volk.h>
#include <algorithm>
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
    // Draws a hollow rectangular frame `borderThickness` pixels wide, centered at (cx, cy).
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

    // The simulated area, in pixels: the world the scene is laid out in and the resolution
    // recordings are captured at, independent of the window. The live view shows it scaled to fit
    // the view area (SceneZoom()). Set from the scene's boundary resolution.
    void  SetSimArea(float w, float h) { simW_ = w; simH_ = h; }

    // Restricts the live scene pass to this framebuffer-pixel rect (the free area between the docked
    // panels), with the world origin at its center. w/h = 0 means the full window. Recordings are
    // unaffected: they are captured offscreen at the simulated area's resolution.
    struct SceneViewport { float x = 0.f, y = 0.f, w = 0.f, h = 0.f; };
    void SetSceneViewport(float x, float y, float w, float h) { sceneViewport_ = { x, y, w, h }; }
    bool UsesSceneViewport() const { return ActiveViewport() != nullptr; }
    // World-to-screen scale of the live scene pass: the simulated area scaled to fit the view.
    float SceneZoom() const {
        const SceneViewport* vp = ActiveViewport();
        const float vw = vp ? vp->w : static_cast<float>(swapchainExtent.width);
        const float vh = vp ? vp->h : static_cast<float>(swapchainExtent.height);
        if (!(simW_ > 0.f) || !(simH_ > 0.f) || !(vw > 0.f) || !(vh > 0.f)) return 1.f;
        constexpr float Margin = 0.96f; // leaves the container outline some room
        return Margin * std::min(vw / simW_, vh / simH_);
    }
    // Center of the rect the live scene is drawn into, in framebuffer pixels: the world origin on screen.
    void SceneCenterPx(float& cx, float& cy) const {
        if (const SceneViewport* vp = ActiveViewport()) { cx = vp->x + vp->w * 0.5f; cy = vp->y + vp->h * 0.5f; }
        else                                            { cx = HalfWidth(); cy = HalfHeight(); }
    }

    // Captures the scene (not the UI) at width x height pixels (the simulated area; even, for the
    // encoder) at `fps`. Live capture is paced against real time (via RenderFrame's dt).
    // lengthSeconds <= 0 records until StopRecording().
    bool  StartRecording(const std::string& outputPath, int fps, float lengthSeconds, int width, int height);
    void  StopRecording();
    bool  IsRecording() const { return recorder_.IsActive(); }
    float RecordedSeconds() const { return recordFps_ > 0 ? static_cast<float>(recordedFrames_) / recordFps_ : 0.f; }

    // False while batch rendering, so the per-tick RenderFrame() that keeps the progress overlay up
    // doesn't also capture a frame into the video. True otherwise.
    void  SetCaptureFromSwapchain(bool enable) { captureFromSwapchain_ = enable; }

    // Renders the scene (not UI) to the offscreen target and captures it, with no swapchain or
    // present, so it can't be throttled by vsync. Every call captures one frame and auto-stops at
    // the length cap set by StartRecording(). A no-op if no recording is active.
    void RenderOffscreenFrame();

private:
    // SSBO instance data: world-space coords; NDC computed in the vertex shader via push constants
    struct CircleData {
        float r, g, b, a;  // vec4 color  - offset  0
        float cx, cy;      // vec2 center - offset 16
        float radius;      //               offset 24
        float _pad;        //               offset 28, total 32
    };
    static_assert(sizeof(CircleData) == 32);

    // SSBO instance data for the hollow-frame rect shader.
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

    // Sample count of the scene pass: 4x MSAA if the device supports it, else 1. Beyond per-shape
    // edge AA, it removes the moire that tiny, regularly arranged particles otherwise produce.
    VkSampleCountFlagBits sceneSampleCount{ VK_SAMPLE_COUNT_1_BIT };

    VkSwapchainKHR         swapchain{ VK_NULL_HANDLE };
    std::vector<VkImage>   images;
    std::vector<VkImageView> imageViews;

    // Multisampled target the scene pass draws into when sceneSampleCount > 1, resolved into
    // imageViews[imageIndex] in the same pass. Recreated with the swapchain; VK_NULL_HANDLE
    // without MSAA.
    VkImage        swapchainMsaaImage{ VK_NULL_HANDLE };
    VkDeviceMemory swapchainMsaaMemory{ VK_NULL_HANDLE };
    VkImageView    swapchainMsaaView{ VK_NULL_HANDLE };

    // RenderOffscreenFrame()'s render target: swapchain format, sized at StartRecording() time.
    VkImage        offscreenImage{ VK_NULL_HANDLE };
    VkDeviceMemory offscreenMemory{ VK_NULL_HANDLE };
    VkImageView    offscreenView{ VK_NULL_HANDLE };
    VkExtent2D     offscreenExtent{};
    // offscreenImage's MSAA counterpart (see swapchainMsaaImage).
    VkImage        offscreenMsaaImage{ VK_NULL_HANDLE };
    VkDeviceMemory offscreenMsaaMemory{ VK_NULL_HANDLE };
    VkImageView    offscreenMsaaView{ VK_NULL_HANDLE };
    // Whether offscreenImage has been rendered to yet (its first barrier starts from UNDEFINED).
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
    float           simW_{ 0.f }, simH_{ 0.f };

    // The rect the live scene pass draws into, or null for the whole swapchain.
    const SceneViewport* ActiveViewport() const {
        return sceneViewport_.w > 0.f && sceneViewport_.h > 0.f ? &sceneViewport_ : nullptr;
    }
    std::function<void()> uiCallback_;
    float                 blockedMs_{ 0.f };

    // Frame recording: the final image is copied into one of a small ring of host-visible readback
    // buffers. Recorder's background thread reads a slot and calls its release callback to free
    // it, keeping the slow read off the render thread. If the reader falls behind, capture is
    // skipped for that frame rather than stalling.
    static constexpr int kCaptureSlots = 3;

    Recorder     recorder_;
    int          recordFps_{ 0 };
    float        recordLengthSeconds_{ 0.f };
    float        recordAccum_{ 0.f };
    uint32_t     recordedFrames_{ 0 };
    uint32_t     recordWidth_{ 0 }, recordHeight_{ 0 };
    VkDeviceSize captureBufferSize_{ 0 };
    // Whether captureMemories_ is HOST_COHERENT; if not, CPU reads need vkInvalidateMappedMemoryRanges.
    bool         captureMemoryCoherent_{ true };

    VkBuffer          captureBuffers_[kCaptureSlots]{};
    VkDeviceMemory    captureMemories_[kCaptureSlots]{};
    void*             captureMapped_[kCaptureSlots]{};
    std::atomic<bool> captureSlotBusy_[kCaptureSlots]{};
    int               captureWriteIndex_{ 0 };
    int               pendingCaptureSlot_{ -1 }; // slot the most recent submission copied into, not yet handed to Recorder
    bool              captureFromSwapchain_{ true }; // see SetCaptureFromSwapchain (name kept: it gates RenderFrame()'s capture)

    // Hands pendingCaptureSlot_ (if any) to Recorder and clears it. Call only once the GPU copy has
    // finished (after a fence wait or vkDeviceWaitIdle).
    void FlushPendingCapture();

    void CreateSwapchain();
    void RecreateSwapchain();
    // The circle/rect draw pass shared by RenderFrame() and the offscreen capture: clears
    // `drawView` and draws the current circles/rects at `extent`, scaled by `zoom`, then clears
    // both vectors unless `keepShapes`. Callers own the command buffer and barriers (including
    // transitioning to COLOR_ATTACHMENT_OPTIMAL).
    //
    // `resolveView` is VK_NULL_HANDLE without MSAA, and `drawView` is then the final target.
    // Otherwise `drawView` is the MSAA target and `resolveView` the single-sample image it resolves into.
    void RecordScenePass(VkImageView drawView, VkImageView resolveView, VkExtent2D extent,
                         const SceneViewport* viewport = nullptr, float zoom = 1.f, bool keepShapes = false);
    // Records, into the current command buffer, a full-resolution scene pass into offscreenImage
    // followed by its copy into captureBuffers_[slot]. Leaves the shapes for a later pass if `keepShapes`.
    void RecordOffscreenCapture(int slot, bool keepShapes);
    void CreateShapePipeline(const char* vertSpv, const char* fragSpv,
                             VkDescriptorSetLayout descSetLayout,
                             VkPipelineLayout& outLayout, VkPipeline& outPipeline);
    // `preferred` flags are matched on top of HOST_VISIBLE where available. Pass HOST_CACHED for
    // buffers the CPU reads, since uncached mappings are an order of magnitude slower to read.
    void CreateSSBO(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags preferred,
                    VkBuffer& buf, VkDeviceMemory& mem, void*& mapped, bool* outCoherent = nullptr);

    // Image+memory+view allocation for this class's device-local color attachments.
    void CreateColorImage(VkExtent2D extent, VkSampleCountFlagBits samples, VkImageUsageFlags usage,
                          VkImage& img, VkDeviceMemory& mem, VkImageView& view);
    // Destroys and nulls an image from CreateColorImage; a no-op if already null. Callers must
    // first wait for the GPU to finish with it.
    void DestroyColorImage(VkImage& img, VkDeviceMemory& mem, VkImageView& view);
};
