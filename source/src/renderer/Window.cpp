#include "renderer/Window.h"
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <iostream>

void Window::Init(const char* title, int w, int h) {
    SDL_Init(SDL_INIT_VIDEO);
    window = SDL_CreateWindow(title, w, h, SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE);
    if (!window) { std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << "\n"; exit(1); }
}

bool Window::PollEvents() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL3_ProcessEvent(&event);
        if (event.type == SDL_EVENT_QUIT) return false;
        if (event.type == SDL_EVENT_KEY_DOWN) {
            // Gated on WantCaptureKeyboard and no-repeat so typing in a text field doesn't pause/step.
            if (!event.key.repeat && !ImGui::GetIO().WantCaptureKeyboard) {
                if (event.key.scancode == SDL_SCANCODE_SPACE) spaceToggled_   = true;
                if (event.key.scancode == SDL_SCANCODE_RIGHT) stepForward_    = true;
                if (event.key.scancode == SDL_SCANCODE_LEFT)  stepForwardBig_ = true;
            }
        }
    }
    return true;
}

bool Window::TakeSpaceToggle() {
    if (!spaceToggled_) return false;
    spaceToggled_ = false;
    return true;
}

bool Window::TakeStepForward() {
    if (!stepForward_) return false;
    stepForward_ = false;
    return true;
}

bool Window::TakeStepForwardBig() {
    if (!stepForwardBig_) return false;
    stepForwardBig_ = false;
    return true;
}

void Window::Shutdown() {
    SDL_DestroyWindow(window);
    SDL_Quit();
}
