#pragma once
#include <SDL3/SDL.h>

class Window {
public:
    void Init(const char* title, int w, int h);
    bool PollEvents();
    void Shutdown();

    SDL_Window* Handle() const { return window; }
    bool TakeSpaceToggle();     // returns true once per Space press (pause/resume)
    bool TakeStepForward();     // returns true once per Right Arrow press (step 1 frame)
    bool TakeStepForwardBig();  // returns true once per Left Arrow press (step several frames)

private:
    SDL_Window* window{ nullptr };
    bool        spaceToggled_{ false };
    bool        stepForward_{ false };
    bool        stepForwardBig_{ false };
};
