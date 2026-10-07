#pragma once

struct Color { float r, g, b, a; };

// Window setup only; everything else tunable lives in Scene.h.
namespace Config {
    constexpr const char* WindowTitle  = "Stjarna";
    constexpr int         WindowWidth  = 2560;
    constexpr int         WindowHeight = 1440;
}
