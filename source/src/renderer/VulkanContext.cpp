#include <volk.h>
#include <SDL3/SDL.h>
#include <SDL3/SDL_vulkan.h>
#include "renderer/VulkanContext.h"
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_vulkan.h>
#include <algorithm>
#include "ui/Theme.h"
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <cstdio>
#include <cstring>

static void chk(VkResult r) {
    if (r != VK_SUCCESS) { std::cerr << "Vulkan error: " << r << "\n"; exit(1); }
}

static std::vector<uint32_t> LoadSpv(const char* name) {
    static std::string base = [] {
        const char* b = SDL_GetBasePath();
        return std::string(b ? b : "");
    }();
    std::string path = base + "shaders/" + name;
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) { fprintf(stderr, "Cannot open shader: %s\n", path.c_str()); exit(1); }
    fseek(f, 0, SEEK_END); size_t size = ftell(f); fseek(f, 0, SEEK_SET);
    std::vector<uint32_t> buf(size / 4);
    fread(buf.data(), 1, size, f); fclose(f);
    return buf;
}

static VkShaderModule MakeShader(VkDevice device, const std::vector<uint32_t>& code) {
    VkShaderModuleCreateInfo ci{ .sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO, .codeSize = code.size() * 4, .pCode = code.data() };
    VkShaderModule mod; chk(vkCreateShaderModule(device, &ci, nullptr, &mod)); return mod;
}

bool VulkanContext::Init(SDL_Window* window) {
    sdlWindow = window;
    volkInitialize();

    // Instance
    VkApplicationInfo appInfo{ .sType = VK_STRUCTURE_TYPE_APPLICATION_INFO, .pApplicationName = "Stjarna", .apiVersion = VK_API_VERSION_1_3 };
    uint32_t extCount = 0;
    const char* const* exts = SDL_Vulkan_GetInstanceExtensions(&extCount);
    VkInstanceCreateInfo instanceCI{ .sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO, .pApplicationInfo = &appInfo, .enabledExtensionCount = extCount, .ppEnabledExtensionNames = exts };
    chk(vkCreateInstance(&instanceCI, nullptr, &instance));
    volkLoadInstance(instance);

    // Surface
    if (!SDL_Vulkan_CreateSurface(window, instance, nullptr, &surface)) {
        std::cerr << "SDL_Vulkan_CreateSurface failed: " << SDL_GetError() << "\n"; return false;
    }

    // Physical device
    uint32_t deviceCount = 0;
    chk(vkEnumeratePhysicalDevices(instance, &deviceCount, nullptr));
    if (deviceCount == 0) { std::cerr << "No Vulkan-capable GPU found\n"; return false; }
    std::vector<VkPhysicalDevice> devices(deviceCount);
    chk(vkEnumeratePhysicalDevices(instance, &deviceCount, devices.data()));
    physicalDevice = devices[0];

    // See sceneSampleCount in VulkanContext.h. 4x is enough; fall back to no MSAA if unsupported.
    {
        VkPhysicalDeviceProperties props{};
        vkGetPhysicalDeviceProperties(physicalDevice, &props);
        constexpr VkSampleCountFlagBits desired = VK_SAMPLE_COUNT_4_BIT;
        sceneSampleCount = (props.limits.framebufferColorSampleCounts & desired) ? desired : VK_SAMPLE_COUNT_1_BIT;
    }

    // Queue family
    uint32_t qfCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &qfCount, nullptr);
    std::vector<VkQueueFamilyProperties> qfs(qfCount);
    vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &qfCount, qfs.data());
    bool found = false;
    for (uint32_t i = 0; i < qfCount; i++) {
        VkBool32 present = VK_FALSE;
        vkGetPhysicalDeviceSurfaceSupportKHR(physicalDevice, i, surface, &present);
        if ((qfs[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) && present) { queueFamily = i; found = true; break; }
    }
    if (!found) { std::cerr << "No graphics+present queue family\n"; return false; }

    // Logical device with dynamic rendering
    const float qp = 1.0f;
    VkDeviceQueueCreateInfo queueCI{ .sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO, .queueFamilyIndex = queueFamily, .queueCount = 1, .pQueuePriorities = &qp };
    const char* devExts[]{ VK_KHR_SWAPCHAIN_EXTENSION_NAME };
    VkPhysicalDeviceDynamicRenderingFeatures dynFeatures{ .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DYNAMIC_RENDERING_FEATURES, .dynamicRendering = VK_TRUE };
    VkDeviceCreateInfo deviceCI{ .sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO, .pNext = &dynFeatures, .queueCreateInfoCount = 1, .pQueueCreateInfos = &queueCI, .enabledExtensionCount = 1, .ppEnabledExtensionNames = devExts };
    chk(vkCreateDevice(physicalDevice, &deviceCI, nullptr, &device));
    volkLoadDevice(device);
    vkGetDeviceQueue(device, queueFamily, 0, &queue);

    // Surface format
    uint32_t formatCount = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, nullptr);
    std::vector<VkSurfaceFormatKHR> formats(formatCount);
    vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, formats.data());
    // Prefer UNORM over _SRGB: colors are written as final display values, and an _SRGB format
    // would re-encode every write and wash out all colors. Don't revert this.
    imageFormat = formats[0].format; colorSpace = formats[0].colorSpace;
    for (auto& f : formats)
        if (f.format == VK_FORMAT_B8G8R8A8_UNORM && f.colorSpace == VK_COLORSPACE_SRGB_NONLINEAR_KHR)
            { imageFormat = f.format; colorSpace = f.colorSpace; break; }

    CreateSwapchain();

    // Shared descriptor set layout: one SSBO at binding 0, vertex stage
    VkDescriptorSetLayoutBinding ssboBinding{
        .binding = 0,
        .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
        .descriptorCount = 1,
        .stageFlags = VK_SHADER_STAGE_VERTEX_BIT,
    };
    VkDescriptorSetLayoutCreateInfo dslCI{
        .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
        .bindingCount = 1, .pBindings = &ssboBinding,
    };
    chk(vkCreateDescriptorSetLayout(device, &dslCI, nullptr, &shapeDescSetLayout));

    CreateShapePipeline("circle.vert.spv", "circle.frag.spv", shapeDescSetLayout, circlePipelineLayout, circlePipeline);
    CreateShapePipeline("rect.vert.spv", "rect.frag.spv", shapeDescSetLayout, rectPipelineLayout, rectPipeline);

    // SSBOs: persistently mapped, host-visible + coherent
    const VkDeviceSize circleSSBOSize = kMaxObjects * sizeof(CircleData);
    CreateSSBO(circleSSBOSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
               circleSSBO, circleSSBOMemory, circleMapped);
    const VkDeviceSize rectSSBOSize = kMaxRects * sizeof(RectData);
    CreateSSBO(rectSSBOSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
               rectSSBO, rectSSBOMemory, rectMapped);

    // Descriptor pool + sets: one storage-buffer set each for circles and rects, sharing shapeDescSetLayout.
    VkDescriptorPoolSize poolSize{ .type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, .descriptorCount = 2 };
    VkDescriptorPoolCreateInfo poolCI{
        .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO,
        .maxSets = 2, .poolSizeCount = 1, .pPoolSizes = &poolSize,
    };
    chk(vkCreateDescriptorPool(device, &poolCI, nullptr, &descPool));

    VkDescriptorSetAllocateInfo dsAllocInfo{
        .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO,
        .descriptorPool = descPool, .descriptorSetCount = 1, .pSetLayouts = &shapeDescSetLayout,
    };
    chk(vkAllocateDescriptorSets(device, &dsAllocInfo, &circleDescSet));
    chk(vkAllocateDescriptorSets(device, &dsAllocInfo, &rectDescSet));

    VkDescriptorBufferInfo circleBI{ .buffer = circleSSBO, .offset = 0, .range = VK_WHOLE_SIZE };
    VkDescriptorBufferInfo rectBI{ .buffer = rectSSBO, .offset = 0, .range = VK_WHOLE_SIZE };
    VkWriteDescriptorSet writes[2]{
        { .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, .dstSet = circleDescSet, .dstBinding = 0, .descriptorCount = 1, .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, .pBufferInfo = &circleBI },
        { .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, .dstSet = rectDescSet,   .dstBinding = 0, .descriptorCount = 1, .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, .pBufferInfo = &rectBI },
    };
    vkUpdateDescriptorSets(device, 2, writes, 0, nullptr);

    VkCommandPoolCreateInfo cpCI{ .sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO, .flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT, .queueFamilyIndex = queueFamily };
    chk(vkCreateCommandPool(device, &cpCI, nullptr, &commandPool));
    VkCommandBufferAllocateInfo cbAI{ .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO, .commandPool = commandPool, .commandBufferCount = 1 };
    chk(vkAllocateCommandBuffers(device, &cbAI, &cb));

    VkSemaphoreCreateInfo semCI{ .sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
    VkFenceCreateInfo fenceCI{ .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO, .flags = VK_FENCE_CREATE_SIGNALED_BIT };
    chk(vkCreateSemaphore(device, &semCI, nullptr, &acquireSem));
    chk(vkCreateSemaphore(device, &semCI, nullptr, &renderSem));
    chk(vkCreateFence(device, &fenceCI, nullptr, &fence));

    circles.reserve(kMaxObjects);
    rects.reserve(kMaxRects);
    return true;
}

// Flat, softly-rounded dark theme built from ui/Theme.h's palette. Panels are lighter than the sim
// canvas (Theme::Canvas, also the scene pass's clear color).
static void ApplyModernStyle() {
    ImGuiStyle& style = ImGui::GetStyle();

    style.WindowRounding    = 14.0f;
    style.ChildRounding     = 12.0f;
    style.FrameRounding     = 10.0f;
    style.PopupRounding     = 12.0f;
    style.ScrollbarRounding = 12.0f;
    style.GrabRounding      = 10.0f;
    style.TabRounding       = 10.0f;
    style.WindowBorderSize  = 0.0f;
    style.ChildBorderSize   = 0.0f;
    style.FrameBorderSize   = 0.0f;
    style.PopupBorderSize   = 0.0f;
    style.TabBorderSize     = 0.0f;
    style.WindowPadding     = ImVec2(14.f, 14.f);
    style.FramePadding      = ImVec2(10.f, 6.f);
    style.ItemSpacing       = ImVec2(10.f, 8.f);
    style.ItemInnerSpacing  = ImVec2(8.f, 6.f);
    style.ScrollbarSize     = 12.f;
    style.GrabMinSize       = 10.f;
    style.SeparatorTextBorderSize = 1.f;
    style.DockingSeparatorSize    = 3.f;
    style.ScaleAllSizes(1.2f);

    using namespace Theme;
    const ImVec4 headerBase = WithAlpha(Accent, 0.28f);
    const ImVec4 headerHov  = WithAlpha(Accent, 0.45f);
    const ImVec4 headerAct  = WithAlpha(Accent, 0.60f);

    ImVec4* c = style.Colors;
    c[ImGuiCol_Text]                 = Text;
    c[ImGuiCol_TextDisabled]         = TextDim;
    c[ImGuiCol_WindowBg]             = Panel;
    c[ImGuiCol_ChildBg]              = ImVec4(0.f, 0.f, 0.f, 0.f);
    c[ImGuiCol_PopupBg]              = PanelAlt;
    c[ImGuiCol_Border]               = ImVec4(1.f, 1.f, 1.f, 0.07f);
    c[ImGuiCol_FrameBg]              = Field;
    c[ImGuiCol_FrameBgHovered]       = FieldHov;
    c[ImGuiCol_FrameBgActive]        = FieldHov;
    c[ImGuiCol_TitleBg]              = Panel;
    c[ImGuiCol_TitleBgActive]        = Panel;
    c[ImGuiCol_TitleBgCollapsed]     = Panel;
    c[ImGuiCol_MenuBarBg]            = PanelAlt;
    c[ImGuiCol_ScrollbarBg]          = ImVec4(0.f, 0.f, 0.f, 0.f);
    c[ImGuiCol_ScrollbarGrab]        = Field;
    c[ImGuiCol_ScrollbarGrabHovered] = FieldHov;
    c[ImGuiCol_ScrollbarGrabActive]  = Accent;
    c[ImGuiCol_CheckMark]            = Accent;
    c[ImGuiCol_SliderGrab]           = Accent;
    c[ImGuiCol_SliderGrabActive]     = AccentHov;
    c[ImGuiCol_Button]               = Field;
    c[ImGuiCol_ButtonHovered]        = FieldHov;
    c[ImGuiCol_ButtonActive]         = AccentAct;
    c[ImGuiCol_Header]               = headerBase;
    c[ImGuiCol_HeaderHovered]        = headerHov;
    c[ImGuiCol_HeaderActive]         = headerAct;
    c[ImGuiCol_Separator]            = ImVec4(1.f, 1.f, 1.f, 0.08f);
    c[ImGuiCol_SeparatorHovered]     = Accent;
    c[ImGuiCol_SeparatorActive]      = AccentHov;
    c[ImGuiCol_ResizeGrip]           = ImVec4(1.f, 1.f, 1.f, 0.04f);
    c[ImGuiCol_ResizeGripHovered]    = Accent;
    c[ImGuiCol_ResizeGripActive]     = AccentHov;
    c[ImGuiCol_Tab]                  = PanelAlt;
    c[ImGuiCol_TabHovered]           = headerHov;
    c[ImGuiCol_TabSelected]          = WithAlpha(Accent, 0.35f);
    c[ImGuiCol_TabSelectedOverline]  = Accent;
    c[ImGuiCol_TabDimmed]            = PanelAlt;
    c[ImGuiCol_TabDimmedSelected]    = Field;
    c[ImGuiCol_TabDimmedSelectedOverline] = ImVec4(0.f, 0.f, 0.f, 0.f);
    c[ImGuiCol_DockingPreview]       = WithAlpha(Accent, 0.45f);
    c[ImGuiCol_DockingEmptyBg]       = Canvas;
    c[ImGuiCol_PlotLines]            = Accent;
    c[ImGuiCol_PlotLinesHovered]     = AccentHov;
    c[ImGuiCol_PlotHistogram]        = Accent;
    c[ImGuiCol_PlotHistogramHovered] = AccentHov;
    c[ImGuiCol_TextSelectedBg]       = headerBase;
    c[ImGuiCol_DragDropTarget]       = Accent;
    c[ImGuiCol_NavCursor]            = Accent;
}

void VulkanContext::InitImGui(SDL_Window* window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    ApplyModernStyle();

    // Karla, fetched with imgui (misc/fonts) and copied next to the exe by CMake's Fonts target,
    // replaces ImGui's default bitmap font.
    ImGuiIO& io = ImGui::GetIO();
    const char* baseDir = SDL_GetBasePath();
    std::string fontPath = std::string(baseDir ? baseDir : "") + "fonts/Karla-Regular.ttf";
    ImFontConfig fontCfg;
    fontCfg.OversampleH = 3;
    fontCfg.OversampleV = 1;
    if (!io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 22.0f, &fontCfg))
        io.Fonts->AddFontDefault();

    ImGui_ImplSDL3_InitForVulkan(window);

    ImGui_ImplVulkan_InitInfo info{};
    info.ApiVersion      = VK_API_VERSION_1_3;
    info.Instance        = instance;
    info.PhysicalDevice  = physicalDevice;
    info.Device          = device;
    info.QueueFamily     = queueFamily;
    info.Queue           = queue;
    info.DescriptorPoolSize = 16;
    info.MinImageCount   = 2;
    info.ImageCount      = (uint32_t)images.size();
    info.UseDynamicRendering = true;
    info.PipelineInfoMain.PipelineRenderingCreateInfo = {
        .sType                   = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO,
        .colorAttachmentCount    = 1,
        .pColorAttachmentFormats = &imageFormat,
    };
    ImGui_ImplVulkan_Init(&info);
}

void VulkanContext::CreateSwapchain() {
    VkSurfaceCapabilitiesKHR caps{};
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, surface, &caps);
    int w = 0, h = 0;
    SDL_GetWindowSizeInPixels(sdlWindow, &w, &h);
    swapchainExtent = caps.currentExtent.width == 0xFFFFFFFF ? VkExtent2D{ (uint32_t)w, (uint32_t)h } : caps.currentExtent;
    uint32_t imgCount = caps.minImageCount + 1;
    if (caps.maxImageCount > 0 && imgCount > caps.maxImageCount) imgCount = caps.maxImageCount;

    for (auto& iv : imageViews) vkDestroyImageView(device, iv, nullptr);
    imageViews.clear();

    VkSwapchainKHR old = swapchain;
    VkSwapchainCreateInfoKHR swapCI{
        .sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR, .surface = surface,
        .minImageCount = imgCount, .imageFormat = imageFormat, .imageColorSpace = colorSpace,
        .imageExtent = swapchainExtent, .imageArrayLayers = 1,
        .imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
        .preTransform = caps.currentTransform,
        .compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR,
        .presentMode = VK_PRESENT_MODE_FIFO_KHR, .oldSwapchain = old,
    };
    chk(vkCreateSwapchainKHR(device, &swapCI, nullptr, &swapchain));
    if (old != VK_NULL_HANDLE) vkDestroySwapchainKHR(device, old, nullptr);

    uint32_t imageCount = 0;
    chk(vkGetSwapchainImagesKHR(device, swapchain, &imageCount, nullptr));
    images.resize(imageCount);
    chk(vkGetSwapchainImagesKHR(device, swapchain, &imageCount, images.data()));

    imageViews.resize(imageCount);
    for (uint32_t i = 0; i < imageCount; i++) {
        VkImageViewCreateInfo ivCI{
            .sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO, .image = images[i],
            .viewType = VK_IMAGE_VIEW_TYPE_2D, .format = imageFormat,
            .subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 },
        };
        chk(vkCreateImageView(device, &ivCI, nullptr, &imageViews[i]));
    }

    // Sized to swapchainExtent, so rebuilt whenever that changes; the caller has already idled the GPU.
    DestroyColorImage(swapchainMsaaImage, swapchainMsaaMemory, swapchainMsaaView);
    if (sceneSampleCount != VK_SAMPLE_COUNT_1_BIT)
        CreateColorImage(swapchainExtent, sceneSampleCount, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
                         swapchainMsaaImage, swapchainMsaaMemory, swapchainMsaaView);
}

void VulkanContext::RecreateSwapchain() {
    // Recordings capture an offscreen target sized by the scene, so a window resize doesn't touch them.
    vkDeviceWaitIdle(device);
    CreateSwapchain();
}

// Shared by RenderFrame() and RenderOffscreenFrame(); see the header comment. Must run between
// vkCmdBeginCommandBuffer and the caller's own barriers/vkCmdEndCommandBuffer.
void VulkanContext::RecordScenePass(VkImageView drawView, VkImageView resolveView, VkExtent2D extent,
                                    const SceneViewport* vp, float zoom, bool keepShapes) {
    const bool msaa = resolveView != VK_NULL_HANDLE;
    VkRenderingAttachmentInfo att{
        .sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO,
        .imageView = drawView, .imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
        .resolveMode = msaa ? VK_RESOLVE_MODE_AVERAGE_BIT : VK_RESOLVE_MODE_NONE,
        .resolveImageView = resolveView,
        .resolveImageLayout = msaa ? VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL : VK_IMAGE_LAYOUT_UNDEFINED,
        .loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR,
        // Only the resolved output is read again, so the MSAA attachment is DONT_CARE.
        .storeOp = msaa ? VK_ATTACHMENT_STORE_OP_DONT_CARE : VK_ATTACHMENT_STORE_OP_STORE,
        .clearValue = { .color = { .float32 = { Theme::Canvas.x, Theme::Canvas.y, Theme::Canvas.z, 1.0f } } }, // Theme::Canvas
    };
    VkRenderingInfo ri{
        .sType = VK_STRUCTURE_TYPE_RENDERING_INFO,
        .renderArea = { {0,0}, extent },
        .layerCount = 1, .colorAttachmentCount = 1, .pColorAttachments = &att,
    };
    vkCmdBeginRendering(cb, &ri);
    {
        // A sub-rect viewport (the free center area) re-centers the world origin and, since
        // invScreen derives from the same size, keeps 1 world unit = 1 pixel.
        VkViewport viewport{ 0, 0, (float)extent.width, (float)extent.height, 0.0f, 1.0f };
        VkRect2D   scissor{ {0,0}, extent };
        float vw = (float)extent.width, vh = (float)extent.height;
        if (vp) {
            const float x0 = std::clamp(vp->x, 0.f, vw - 1.f), y0 = std::clamp(vp->y, 0.f, vh - 1.f);
            const float x1 = std::clamp(vp->x + vp->w, x0 + 1.f, vw), y1 = std::clamp(vp->y + vp->h, y0 + 1.f, vh);
            viewport = { x0, y0, x1 - x0, y1 - y0, 0.0f, 1.0f };
            scissor  = { { (int32_t)x0, (int32_t)y0 }, { (uint32_t)(x1 - x0), (uint32_t)(y1 - y0) } };
            vw = x1 - x0; vh = y1 - y0;
        }
        vkCmdSetViewport(cb, 0, 1, &viewport);
        vkCmdSetScissor(cb, 0, 1, &scissor);

        float invScreen[2] = { 2.0f * zoom / vw, 2.0f * zoom / vh };

        if (!circles.empty()) {
            const uint32_t n = (uint32_t)circles.size();
            memcpy(circleMapped, circles.data(), n * sizeof(CircleData));
            vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, circlePipeline);
            vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, circlePipelineLayout, 0, 1, &circleDescSet, 0, nullptr);
            vkCmdPushConstants(cb, circlePipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(invScreen), invScreen);
            vkCmdDraw(cb, 6, n, 0, 0);
        }

        if (!rects.empty()) {
            const uint32_t n = (uint32_t)rects.size();
            memcpy(rectMapped, rects.data(), n * sizeof(RectData));
            vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, rectPipeline);
            vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, rectPipelineLayout, 0, 1, &rectDescSet, 0, nullptr);
            vkCmdPushConstants(cb, rectPipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(invScreen), invScreen);
            vkCmdDraw(cb, 6, n, 0, 0);
        }
    }
    vkCmdEndRendering(cb);
    if (!keepShapes) {
        circles.clear();
        rects.clear();
    }
}

void VulkanContext::RecordOffscreenCapture(int captureSlot, bool keepShapes) {
    VkImageSubresourceRange range{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
    VkImageMemoryBarrier toRender{
        .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
        .srcAccessMask = offscreenEverRendered ? VK_ACCESS_TRANSFER_READ_BIT : VkAccessFlags(0),
        .dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
        // Every frame after the first starts in TRANSFER_SRC_OPTIMAL (left by its capture copy).
        .oldLayout = offscreenEverRendered ? VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL : VK_IMAGE_LAYOUT_UNDEFINED,
        .newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
        .image = offscreenImage, .subresourceRange = range,
    };
    vkCmdPipelineBarrier(cb, offscreenEverRendered ? VK_PIPELINE_STAGE_TRANSFER_BIT : VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                          VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, 0, 0, nullptr, 0, nullptr, 1, &toRender);
    offscreenEverRendered = true;

    // Always from UNDEFINED, as with RenderFrame()'s swapchainMsaaImage barrier.
    if (sceneSampleCount != VK_SAMPLE_COUNT_1_BIT) {
        VkImageMemoryBarrier msaaToRender{
            .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
            .srcAccessMask = 0, .dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
            .oldLayout = VK_IMAGE_LAYOUT_UNDEFINED, .newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            .image = offscreenMsaaImage, .subresourceRange = range,
        };
        vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, 0, 0, nullptr, 0, nullptr, 1, &msaaToRender);
    }

    RecordScenePass(sceneSampleCount != VK_SAMPLE_COUNT_1_BIT ? offscreenMsaaView : offscreenView,
                    sceneSampleCount != VK_SAMPLE_COUNT_1_BIT ? offscreenView : VK_NULL_HANDLE,
                    offscreenExtent, nullptr, 1.f, keepShapes);

    VkImageMemoryBarrier toTransferSrc{
        .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
        .srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, .dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT,
        .oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, .newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
        .image = offscreenImage, .subresourceRange = range,
    };
    vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &toTransferSrc);

    VkBufferImageCopy region{
        .bufferOffset = 0, .bufferRowLength = 0, .bufferImageHeight = 0,
        .imageSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 },
        .imageOffset = { 0, 0, 0 }, .imageExtent = { offscreenExtent.width, offscreenExtent.height, 1 },
    };
    vkCmdCopyImageToBuffer(cb, offscreenImage, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, captureBuffers_[captureSlot], 1, &region);
}

void VulkanContext::EnsureOffscreenTarget(uint32_t w, uint32_t h) {
    if (offscreenImage != VK_NULL_HANDLE && offscreenExtent.width == w && offscreenExtent.height == h) return;
    DestroyOffscreenTarget();

    CreateColorImage({ w, h }, VK_SAMPLE_COUNT_1_BIT,
                     VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
                     offscreenImage, offscreenMemory, offscreenView);
    if (sceneSampleCount != VK_SAMPLE_COUNT_1_BIT)
        CreateColorImage({ w, h }, sceneSampleCount, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
                         offscreenMsaaImage, offscreenMsaaMemory, offscreenMsaaView);

    offscreenExtent = { w, h };
    offscreenEverRendered = false;
}

void VulkanContext::DestroyOffscreenTarget() {
    if (offscreenImage == VK_NULL_HANDLE) return;
    vkDeviceWaitIdle(device);
    DestroyColorImage(offscreenImage, offscreenMemory, offscreenView);
    DestroyColorImage(offscreenMsaaImage, offscreenMsaaMemory, offscreenMsaaView);
    offscreenExtent = {};
}

void VulkanContext::CreateColorImage(VkExtent2D extent, VkSampleCountFlagBits samples, VkImageUsageFlags usage,
                                     VkImage& img, VkDeviceMemory& mem, VkImageView& view) {
    VkImageCreateInfo imgCI{
        .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
        .imageType = VK_IMAGE_TYPE_2D, .format = imageFormat,
        .extent = { extent.width, extent.height, 1 }, .mipLevels = 1, .arrayLayers = 1,
        .samples = samples, .tiling = VK_IMAGE_TILING_OPTIMAL,
        .usage = usage, .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
        .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
    };
    chk(vkCreateImage(device, &imgCI, nullptr, &img));

    VkMemoryRequirements memReq{};
    vkGetImageMemoryRequirements(device, img, &memReq);
    VkPhysicalDeviceMemoryProperties memProps{};
    vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProps);
    uint32_t memTypeIndex = UINT32_MAX;
    for (uint32_t i = 0; i < memProps.memoryTypeCount; i++) {
        if ((memReq.memoryTypeBits & (1u << i)) &&
            (memProps.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)) {
            memTypeIndex = i;
            break;
        }
    }
    if (memTypeIndex == UINT32_MAX) { std::cerr << "No suitable memory type for color image\n"; exit(1); }
    VkMemoryAllocateInfo allocInfo{
        .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
        .allocationSize = memReq.size, .memoryTypeIndex = memTypeIndex,
    };
    chk(vkAllocateMemory(device, &allocInfo, nullptr, &mem));
    chk(vkBindImageMemory(device, img, mem, 0));

    VkImageViewCreateInfo ivCI{
        .sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO, .image = img,
        .viewType = VK_IMAGE_VIEW_TYPE_2D, .format = imageFormat,
        .subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 },
    };
    chk(vkCreateImageView(device, &ivCI, nullptr, &view));
}

void VulkanContext::DestroyColorImage(VkImage& img, VkDeviceMemory& mem, VkImageView& view) {
    if (img == VK_NULL_HANDLE) return;
    vkDestroyImageView(device, view, nullptr);
    vkDestroyImage(device, img, nullptr);
    vkFreeMemory(device, mem, nullptr);
    img = VK_NULL_HANDLE; mem = VK_NULL_HANDLE; view = VK_NULL_HANDLE;
}

void VulkanContext::CreateShapePipeline(const char* vertSpv, const char* fragSpv,
                                        VkDescriptorSetLayout descSetLayout,
                                        VkPipelineLayout& outLayout, VkPipeline& outPipeline) {
    VkShaderModule vertMod = MakeShader(device, LoadSpv(vertSpv));
    VkShaderModule fragMod = MakeShader(device, LoadSpv(fragSpv));

    VkPipelineShaderStageCreateInfo stages[2]{
        { .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .stage = VK_SHADER_STAGE_VERTEX_BIT,   .module = vertMod, .pName = "main" },
        { .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .stage = VK_SHADER_STAGE_FRAGMENT_BIT, .module = fragMod, .pName = "main" },
    };
    VkPipelineVertexInputStateCreateInfo   vertexInput  { .sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };
    VkPipelineInputAssemblyStateCreateInfo inputAssembly{ .sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO, .topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST };
    VkPipelineViewportStateCreateInfo      viewportState{ .sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO, .viewportCount = 1, .scissorCount = 1 };
    VkPipelineRasterizationStateCreateInfo rasterizer   { .sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO, .polygonMode = VK_POLYGON_MODE_FILL, .cullMode = VK_CULL_MODE_NONE, .lineWidth = 1.0f };
    // Must match the scene pass's sample count (sceneSampleCount); dynamic rendering won't cross-check it.
    VkPipelineMultisampleStateCreateInfo   multisample  { .sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO, .rasterizationSamples = sceneSampleCount };
    VkPipelineColorBlendAttachmentState blendAtt{
        .blendEnable = VK_TRUE,
        .srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA, .dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, .colorBlendOp = VK_BLEND_OP_ADD,
        .srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE,       .dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO,               .alphaBlendOp = VK_BLEND_OP_ADD,
        .colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT,
    };
    VkPipelineColorBlendStateCreateInfo colorBlend  { .sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO, .attachmentCount = 1, .pAttachments = &blendAtt };
    VkDynamicState dynStates[]{ VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
    VkPipelineDynamicStateCreateInfo    dynamicState{ .sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO, .dynamicStateCount = 2, .pDynamicStates = dynStates };

    VkPushConstantRange pcRange{ .stageFlags = VK_SHADER_STAGE_VERTEX_BIT, .offset = 0, .size = 2 * sizeof(float) };
    VkPipelineLayoutCreateInfo layoutCI{
        .sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,
        .setLayoutCount = 1, .pSetLayouts = &descSetLayout,
        .pushConstantRangeCount = 1, .pPushConstantRanges = &pcRange,
    };
    chk(vkCreatePipelineLayout(device, &layoutCI, nullptr, &outLayout));

    VkPipelineRenderingCreateInfo renderingCI{ .sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO, .colorAttachmentCount = 1, .pColorAttachmentFormats = &imageFormat };
    VkGraphicsPipelineCreateInfo pipelineCI{
        .sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO, .pNext = &renderingCI,
        .stageCount = 2, .pStages = stages,
        .pVertexInputState = &vertexInput, .pInputAssemblyState = &inputAssembly,
        .pViewportState = &viewportState,  .pRasterizationState = &rasterizer,
        .pMultisampleState = &multisample, .pColorBlendState = &colorBlend,
        .pDynamicState = &dynamicState,    .layout = outLayout,
    };
    chk(vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineCI, nullptr, &outPipeline));

    vkDestroyShaderModule(device, vertMod, nullptr);
    vkDestroyShaderModule(device, fragMod, nullptr);
}

void VulkanContext::CreateSSBO(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags preferred,
                               VkBuffer& buf, VkDeviceMemory& mem, void*& mapped, bool* outCoherent) {
    VkBufferCreateInfo bCI{
        .sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,
        .size = size,
        .usage = usage,
        .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
    };
    chk(vkCreateBuffer(device, &bCI, nullptr, &buf));

    VkMemoryRequirements memReq{};
    vkGetBufferMemoryRequirements(device, buf, &memReq);

    VkPhysicalDeviceMemoryProperties memProps{};
    vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProps);

    const VkMemoryPropertyFlags required = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;
    uint32_t memTypeIndex = UINT32_MAX;
    bool coherent = false;

    // Preferred flags first, then plain HOST_VISIBLE|HOST_COHERENT, which every implementation exposes.
    for (VkMemoryPropertyFlags want : { required | preferred, required | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT }) {
        for (uint32_t i = 0; i < memProps.memoryTypeCount; i++) {
            if ((memReq.memoryTypeBits & (1u << i)) && (memProps.memoryTypes[i].propertyFlags & want) == want) {
                memTypeIndex = i;
                coherent = (memProps.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) != 0;
                break;
            }
        }
        if (memTypeIndex != UINT32_MAX) break;
    }
    if (memTypeIndex == UINT32_MAX) { std::cerr << "No suitable memory type for SSBO\n"; exit(1); }

    VkMemoryAllocateInfo allocInfo{
        .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
        .allocationSize = memReq.size,
        .memoryTypeIndex = memTypeIndex,
    };
    chk(vkAllocateMemory(device, &allocInfo, nullptr, &mem));
    chk(vkBindBufferMemory(device, buf, mem, 0));
    chk(vkMapMemory(device, mem, 0, VK_WHOLE_SIZE, 0, &mapped));
    if (outCoherent) *outCoherent = coherent;
}

void VulkanContext::AddCircle(float cx, float cy, float radius, Color color) {
    if (circles.size() >= kMaxObjects) return;
    circles.push_back({ color.r, color.g, color.b, color.a, cx, cy, radius, 0.0f });
}

void VulkanContext::AddRectOutline(float cx, float cy, float halfWidth, float halfHeight, float borderThickness, Color color) {
    if (rects.size() >= kMaxRects) return;
    rects.push_back({ color.r, color.g, color.b, color.a, cx, cy, halfWidth, halfHeight, borderThickness, 0.f, 0.f, 0.f });
}

bool VulkanContext::StartRecording(const std::string& outputPath, int fps, float lengthSeconds, int width, int height) {
    if (recorder_.IsActive()) StopRecording();

    const char* pixelFormat;
    if (imageFormat == VK_FORMAT_B8G8R8A8_SRGB || imageFormat == VK_FORMAT_B8G8R8A8_UNORM)
        pixelFormat = "bgra";
    else if (imageFormat == VK_FORMAT_R8G8B8A8_SRGB || imageFormat == VK_FORMAT_R8G8B8A8_UNORM)
        pixelFormat = "rgba";
    else {
        std::cerr << "Recorder: unsupported swapchain format for capture\n";
        return false;
    }

    if (width < 2 || height < 2) {
        std::cerr << "Recorder: invalid recording resolution " << width << "x" << height << "\n";
        return false;
    }
    recordWidth_  = static_cast<uint32_t>(width);
    recordHeight_ = static_cast<uint32_t>(height);
    EnsureOffscreenTarget(recordWidth_, recordHeight_);
    captureBufferSize_ = VkDeviceSize(recordWidth_) * recordHeight_ * 4;
    for (int i = 0; i < kCaptureSlots; i++) {
        CreateSSBO(captureBufferSize_, VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_HOST_CACHED_BIT,
                   captureBuffers_[i], captureMemories_[i], captureMapped_[i], &captureMemoryCoherent_);
        captureSlotBusy_[i] = false;
    }

    if (!recorder_.Start(outputPath, (int)recordWidth_, (int)recordHeight_, fps, pixelFormat)) {
        for (int i = 0; i < kCaptureSlots; i++) {
            vkUnmapMemory(device, captureMemories_[i]);
            vkDestroyBuffer(device, captureBuffers_[i], nullptr);
            vkFreeMemory(device, captureMemories_[i], nullptr);
            captureBuffers_[i] = VK_NULL_HANDLE; captureMemories_[i] = VK_NULL_HANDLE; captureMapped_[i] = nullptr;
        }
        return false;
    }

    recordFps_           = fps;
    recordLengthSeconds_ = lengthSeconds;
    recordAccum_         = 0.f;
    recordedFrames_      = 0;
    captureWriteIndex_   = 0;
    pendingCaptureSlot_  = -1;
    return true;
}

void VulkanContext::FlushPendingCapture() {
    if (pendingCaptureSlot_ < 0) return;
    const int slot = pendingCaptureSlot_;
    pendingCaptureSlot_ = -1;
    if (!captureMemoryCoherent_) {
        VkMappedMemoryRange range{ .sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE, .memory = captureMemories_[slot], .offset = 0, .size = VK_WHOLE_SIZE };
        vkInvalidateMappedMemoryRanges(device, 1, &range);
    }
    recorder_.SubmitFrame(captureMapped_[slot], captureBufferSize_, [this, slot] { captureSlotBusy_[slot] = false; });
}

void VulkanContext::StopRecording() {
    if (!recorder_.IsActive() && captureBuffers_[0] == VK_NULL_HANDLE) return;

    // Any in-flight GPU copy finishes here, so the most recent captured frame can be flushed
    // rather than dropped. Recorder::Stop() then drains its queue before the slots are freed.
    vkDeviceWaitIdle(device);
    FlushPendingCapture();
    recorder_.Stop();

    for (int i = 0; i < kCaptureSlots; i++) {
        if (captureBuffers_[i] == VK_NULL_HANDLE) continue;
        vkUnmapMemory(device, captureMemories_[i]);
        vkDestroyBuffer(device, captureBuffers_[i], nullptr);
        vkFreeMemory(device, captureMemories_[i], nullptr);
        captureBuffers_[i] = VK_NULL_HANDLE; captureMemories_[i] = VK_NULL_HANDLE; captureMapped_[i] = nullptr;
        captureSlotBusy_[i] = false;
    }
    recordAccum_ = 0.f;
    recordedFrames_ = 0;
}

void VulkanContext::RenderFrame(float dt) {
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();

    if (uiCallback_) uiCallback_();

    ImGui::Render();

    // Time blocked on the GPU/vsync (fence wait, acquire, present), reported via LastBlockedMs().
    using BlockClock = std::chrono::steady_clock;
    blockedMs_ = 0.f;
    auto addBlocked = [this](BlockClock::time_point since) {
        blockedMs_ += std::chrono::duration<float, std::milli>(BlockClock::now() - since).count();
    };

    const auto fenceWaitStart = BlockClock::now();
    chk(vkWaitForFences(device, 1, &fence, VK_TRUE, UINT64_MAX));
    addBlocked(fenceWaitStart);

    // The wait above proves the previous submission's capture copy has finished, so hand that
    // slot to Recorder's background thread.
    FlushPendingCapture();

    uint32_t imageIndex = 0;
    const auto acquireStart = BlockClock::now();
    VkResult acq = vkAcquireNextImageKHR(device, swapchain, UINT64_MAX, acquireSem, VK_NULL_HANDLE, &imageIndex);
    addBlocked(acquireStart);
    if (acq == VK_ERROR_OUT_OF_DATE_KHR || acq == VK_SUBOPTIMAL_KHR) { RecreateSwapchain(); circles.clear(); rects.clear(); return; }
    if (acq != VK_SUCCESS) chk(acq);

    // Paces captured frames against `dt` rather than the call rate, so the video's duration matches
    // accumulated `dt`. Only applies while captureFromSwapchain_ is set; during batch rendering this
    // is called only to refresh the overlay and capturing would splice in a duplicate frame.
    bool capturingThisFrame = false;
    bool stopRecordingAfterSubmit = false;
    int  captureSlot = -1;
    if (recorder_.IsActive() && captureFromSwapchain_) {
        const float interval = 1.f / static_cast<float>(recordFps_);
        recordAccum_ += dt;
        // If the reader hasn't freed the next slot, skip this frame rather than stall.
        if (recordAccum_ >= interval && !captureSlotBusy_[captureWriteIndex_].load()) {
            recordAccum_ -= interval;
            captureSlot = captureWriteIndex_;
            captureWriteIndex_ = (captureWriteIndex_ + 1) % kCaptureSlots;
            captureSlotBusy_[captureSlot] = true;
            capturingThisFrame = true;
            recordedFrames_++;
            if (recordLengthSeconds_ > 0.f && recordedFrames_ >= static_cast<uint32_t>(recordLengthSeconds_ * recordFps_))
                stopRecordingAfterSubmit = true;
        }
    }

    chk(vkResetFences(device, 1, &fence));
    chk(vkResetCommandBuffer(cb, 0));
    VkCommandBufferBeginInfo cbbi{ .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO, .flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT };
    chk(vkBeginCommandBuffer(cb, &cbbi));

    VkImageSubresourceRange range{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
    VkImageMemoryBarrier toRender{
        .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
        .srcAccessMask = 0, .dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
        .oldLayout = VK_IMAGE_LAYOUT_UNDEFINED, .newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
        .image = images[imageIndex], .subresourceRange = range,
    };
    vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, 0, 0, nullptr, 0, nullptr, 1, &toRender);

    // The MSAA target's contents never survive between frames, so always transition from UNDEFINED.
    if (sceneSampleCount != VK_SAMPLE_COUNT_1_BIT) {
        VkImageMemoryBarrier msaaToRender{
            .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
            .srcAccessMask = 0, .dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
            .oldLayout = VK_IMAGE_LAYOUT_UNDEFINED, .newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            .image = swapchainMsaaImage, .subresourceRange = range,
        };
        vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, 0, 0, nullptr, 0, nullptr, 1, &msaaToRender);
    }

    // The video frame: the scene alone, drawn at the recording's own resolution into the offscreen
    // target (so it is independent of the window and the view's zoom) and copied out. The shapes
    // are kept for the on-screen pass below.
    if (capturingThisFrame) {
        RecordOffscreenCapture(captureSlot, /*keepShapes=*/true);
        pendingCaptureSlot_ = captureSlot;
    }

    // Pass 1: the simulated scene, scaled to fit the view. With MSAA it resolves from
    // swapchainMsaaImage into imageViews[imageIndex].
    RecordScenePass(sceneSampleCount != VK_SAMPLE_COUNT_1_BIT ? swapchainMsaaView : imageViews[imageIndex],
                    sceneSampleCount != VK_SAMPLE_COUNT_1_BIT ? imageViews[imageIndex] : VK_NULL_HANDLE,
                    swapchainExtent, ActiveViewport(), SceneZoom());

    // Pass 2: ImGui on top.
    VkRenderingAttachmentInfo uiAtt{
        .sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO,
        .imageView = imageViews[imageIndex], .imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
        .loadOp = VK_ATTACHMENT_LOAD_OP_LOAD, .storeOp = VK_ATTACHMENT_STORE_OP_STORE,
    };
    VkRenderingInfo uiRenderingInfo{
        .sType = VK_STRUCTURE_TYPE_RENDERING_INFO,
        .renderArea = { {0,0}, swapchainExtent },
        .layerCount = 1, .colorAttachmentCount = 1, .pColorAttachments = &uiAtt,
    };
    vkCmdBeginRendering(cb, &uiRenderingInfo);
    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), cb);
    vkCmdEndRendering(cb);

    VkImageMemoryBarrier toPresent{
        .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
        .srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, .dstAccessMask = 0,
        .oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, .newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
        .image = images[imageIndex], .subresourceRange = range,
    };
    vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, 0, 0, nullptr, 0, nullptr, 1, &toPresent);

    chk(vkEndCommandBuffer(cb));

    VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    VkSubmitInfo si{ .sType = VK_STRUCTURE_TYPE_SUBMIT_INFO, .waitSemaphoreCount = 1, .pWaitSemaphores = &acquireSem, .pWaitDstStageMask = &waitStage, .commandBufferCount = 1, .pCommandBuffers = &cb, .signalSemaphoreCount = 1, .pSignalSemaphores = &renderSem };
    chk(vkQueueSubmit(queue, 1, &si, fence));

    VkPresentInfoKHR pi{ .sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR, .waitSemaphoreCount = 1, .pWaitSemaphores = &renderSem, .swapchainCount = 1, .pSwapchains = &swapchain, .pImageIndices = &imageIndex };
    const auto presentStart = BlockClock::now();
    VkResult presentResult = vkQueuePresentKHR(queue, &pi);
    addBlocked(presentStart);
    if (presentResult == VK_ERROR_OUT_OF_DATE_KHR || presentResult == VK_SUBOPTIMAL_KHR) RecreateSwapchain();
    else if (presentResult != VK_SUCCESS) chk(presentResult);

    if (stopRecordingAfterSubmit) StopRecording();
}

// See the header comment: renders to offscreenImage and captures it, touching neither the swapchain nor ImGui.
void VulkanContext::RenderOffscreenFrame() {
    if (!recorder_.IsActive()) { circles.clear(); rects.clear(); return; }

    EnsureOffscreenTarget(recordWidth_, recordHeight_);

    chk(vkWaitForFences(device, 1, &fence, VK_TRUE, UINT64_MAX));
    FlushPendingCapture();

    // Unlike live capture, batch mode must produce exactly one frame per call, since
    // AdvanceRendering steps the sim by a fixed dt per call and the video length depends on that.
    // So if the encoder is behind, block until it frees a slot rather than skip the frame.
    // TEMP-DEBUG: bounded wait; on timeout log slot state and abort the recording instead of hanging.
    const auto dbgWaitStart = std::chrono::steady_clock::now();
    while (captureSlotBusy_[captureWriteIndex_].load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (std::chrono::steady_clock::now() - dbgWaitStart > std::chrono::seconds(10)) {
            std::cerr << "[rec-dbg] capture slot wait timed out: writeIndex=" << captureWriteIndex_
                      << " busy=[" << captureSlotBusy_[0].load() << captureSlotBusy_[1].load() << captureSlotBusy_[2].load()
                      << "] recordedFrames=" << recordedFrames_ << " - aborting recording\n";
            StopRecording();
            return;
        }
    }
    const int captureSlot = captureWriteIndex_;
    captureWriteIndex_ = (captureWriteIndex_ + 1) % kCaptureSlots;
    captureSlotBusy_[captureSlot] = true;
    recordedFrames_++;
    const bool stopAfter = recordLengthSeconds_ > 0.f
                          && recordedFrames_ >= static_cast<uint32_t>(recordLengthSeconds_ * recordFps_);

    chk(vkResetFences(device, 1, &fence));
    chk(vkResetCommandBuffer(cb, 0));
    VkCommandBufferBeginInfo cbbi{ .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO, .flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT };
    chk(vkBeginCommandBuffer(cb, &cbbi));

    RecordOffscreenCapture(captureSlot, /*keepShapes=*/false);

    chk(vkEndCommandBuffer(cb));

    // No semaphores: nothing external to synchronize against.
    VkSubmitInfo si{ .sType = VK_STRUCTURE_TYPE_SUBMIT_INFO, .commandBufferCount = 1, .pCommandBuffers = &cb };
    chk(vkQueueSubmit(queue, 1, &si, fence));

    pendingCaptureSlot_ = captureSlot;

    if (stopAfter) StopRecording();
}

void VulkanContext::Shutdown() {
    StopRecording();
    vkDeviceWaitIdle(device);
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    vkDestroyPipeline(device, circlePipeline, nullptr);
    vkDestroyPipelineLayout(device, circlePipelineLayout, nullptr);
    vkUnmapMemory(device, circleSSBOMemory);
    vkDestroyBuffer(device, circleSSBO, nullptr);
    vkFreeMemory(device, circleSSBOMemory, nullptr);
    vkDestroyPipeline(device, rectPipeline, nullptr);
    vkDestroyPipelineLayout(device, rectPipelineLayout, nullptr);
    vkUnmapMemory(device, rectSSBOMemory);
    vkDestroyBuffer(device, rectSSBO, nullptr);
    vkFreeMemory(device, rectSSBOMemory, nullptr);
    vkDestroyDescriptorPool(device, descPool, nullptr);
    vkDestroyDescriptorSetLayout(device, shapeDescSetLayout, nullptr);
    vkDestroyFence(device, fence, nullptr);
    vkDestroySemaphore(device, renderSem, nullptr);
    vkDestroySemaphore(device, acquireSem, nullptr);
    vkDestroyCommandPool(device, commandPool, nullptr);
    DestroyOffscreenTarget();
    DestroyColorImage(swapchainMsaaImage, swapchainMsaaMemory, swapchainMsaaView);
    for (auto& iv : imageViews) vkDestroyImageView(device, iv, nullptr);
    vkDestroySwapchainKHR(device, swapchain, nullptr);
    vkDestroySurfaceKHR(instance, surface, nullptr);
    vkDestroyDevice(device, nullptr);
    vkDestroyInstance(instance, nullptr);
}
