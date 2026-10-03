#pragma once
#include <filesystem>
#include <string>
#include <vector>

// Builds formatted, collision-safe base filenames shared between a run's video and CSV output, so
// e.g. "SG_run003_150x150_g20_k1200.mp4" and "...csv" both trace back to the same run at a
// glance. Takes plain scalars (not a Scene) so it has no dependency on engine/Scene.h and is
// callable from both App::StartRecording's path and DataRecorder::Start's path.
namespace RunNaming {

struct RunNameParams {
    std::string sceneName;
    int         particleCount  = 0;
    bool        gravityEnabled = false;
    float       gravityG       = 0.f; // embedded only when gravityEnabled
    float       stiffness      = 0.f;
    int         runNumber      = 0;
};

// Fixed 2-letter codes for the 3 compiled presets; any other name (a loaded custom preset) falls
// back to initials of its whitespace-separated words (capped at 4 letters), or "CU" if the name
// has no letters to take initials from.
std::string AbbreviateSceneName(const std::string& sceneName);

// e.g. "SG_run003_22500p_g20_k1200" for Self-Gravity, 22500 particles, g=20, stiffness=1200.
std::string BuildBaseName(const RunNameParams& params);

// Monotonic per-process counter starting at 1 — shared by a run's video and CSV output (call once
// per run, not once per output) so sequential runs in one process session never collide on the
// number alone. Resets across a process restart; EnsureUniqueBaseName is the cross-restart
// safety net.
int NextRunNumber();

struct OutputTarget {
    std::filesystem::path dir;
    std::string           extension; // includes the leading dot, e.g. ".mp4"
};

// If <target.dir>/<baseName><target.extension> doesn't already exist for any target, returns
// baseName unchanged; otherwise appends _2, _3, ... and rechecks every target together, so a
// video/CSV pair sharing one baseName stays paired even if only one of the two collided.
std::string EnsureUniqueBaseName(const std::string& baseName, const std::vector<OutputTarget>& targets);

} // namespace RunNaming
