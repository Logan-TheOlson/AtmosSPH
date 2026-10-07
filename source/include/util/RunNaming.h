#pragma once
#include <filesystem>
#include <string>
#include <vector>

// Builds collision-safe base filenames shared by a run's video and CSV output, so e.g.
// "SG_run003_22500p_g20_k1200.mp4" and "...csv" trace back to the same run. Takes plain scalars
// rather than a Scene to avoid depending on engine/Scene.h.
namespace RunNaming {

struct RunNameParams {
    std::string sceneName;
    int         particleCount  = 0;
    bool        gravityEnabled = false;
    float       gravityG       = 0.f; // embedded only when gravityEnabled
    float       stiffness      = 0.f;
    int         runNumber      = 0;
};

// Fixed 2-letter codes for the built-in presets; any other name falls back to the initials of its
// words (capped at 4 letters), or "CU" if it has no letters.
std::string AbbreviateSceneName(const std::string& sceneName);

// e.g. "SG_run003_22500p_g20_k1200" for Self-Gravity, 22500 particles, g=20, stiffness=1200.
std::string BuildBaseName(const RunNameParams& params);

// Per-process counter starting at 1. Call once per run, not once per output. Resets on restart;
// EnsureUniqueBaseName covers collisions across restarts.
int NextRunNumber();

struct OutputTarget {
    std::filesystem::path dir;
    std::string           extension; // includes the leading dot, e.g. ".mp4"
};

// Returns baseName if no target file already exists; otherwise appends _2, _3, ... and rechecks
// every target together, so a video/CSV pair stays paired.
std::string EnsureUniqueBaseName(const std::string& baseName, const std::vector<OutputTarget>& targets);

} // namespace RunNaming
