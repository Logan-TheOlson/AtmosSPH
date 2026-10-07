#include "renderer/App.h"
#include "util/Filename.h"
#include <SDL3/SDL.h>
#include <filesystem>
#include <iostream>

void App::Init(const char* title, int w, int h) {
    window.Init(title, w, h);
    vk.Init(window.Handle());
    vk.InitImGui(window.Handle());
}

bool App::PollEvents()       { return window.PollEvents(); }
void App::RenderFrame(float dt) { vk.RenderFrame(dt); }

bool App::StartRecording(const std::string& title, int fps, float lengthSeconds) {
    // No fallback name: the caller passes the generated run name.
    const std::string safeTitle = SanitizeFilename(title, "");
    if (safeTitle.empty()) {
        std::cerr << "StartRecording: empty file name\n";
        return false;
    }

    const char* base = SDL_GetBasePath();
    std::filesystem::path dir = std::filesystem::path(base ? base : "") / "recordings";
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
        std::cerr << "StartRecording: couldn't create " << dir << ": " << ec.message() << "\n";
        return false;
    }

    return vk.StartRecording((dir / (safeTitle + ".mp4")).string(), fps, lengthSeconds);
}

void App::AddCircle(float cx, float cy, float radius, Color color) { vk.AddCircle(cx, cy, radius, color); }
void App::AddRectOutline(float cx, float cy, float halfWidth, float halfHeight, float borderThickness, Color color) {
    vk.AddRectOutline(cx, cy, halfWidth, halfHeight, borderThickness, color);
}
int   App::Width()      const { return vk.Width(); }
int   App::Height()     const { return vk.Height(); }
float App::HalfWidth()  const { return vk.HalfWidth(); }
float App::HalfHeight() const { return vk.HalfHeight(); }

void App::Shutdown() {
    vk.Shutdown();
    window.Shutdown();
}
