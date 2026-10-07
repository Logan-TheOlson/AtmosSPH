#pragma once
#include "engine/Scene.h"
#include <filesystem>
#include <string>
#include <vector>

// User-saved scene presets, stored as flat "key=value" text files next to the executable.
// Distinct from the built-in ScenePresets (Scene.h).
struct ScenePresetEntry {
    std::string           name;
    std::string           description;
    std::filesystem::path path;
};

// Writes every Scene field to `path` as "key=value" lines. Overwrites unconditionally. Returns
// false only if the file can't be opened for writing.
bool SaveScenePreset(const std::filesystem::path& path, const Scene& scene);

// Reads a file written by SaveScenePreset into `out`. Keys missing from the file leave `out`'s
// fields untouched; unrecognized keys are skipped with a stderr warning. Returns false only if the
// file can't be opened.
bool LoadScenePreset(const std::filesystem::path& path, Scene& out);

// Scans `presetsDir` for *.scenepreset files sorted by name, skipping (with a stderr warning) any
// that can't be loaded. Does not include the built-in ScenePresets.
std::vector<ScenePresetEntry> ScanUserPresets(const std::filesystem::path& presetsDir);
