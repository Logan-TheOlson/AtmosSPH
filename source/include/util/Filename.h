#pragma once
#include <cctype>
#include <string>
#include <string_view>

// Strips anything unsafe in a filename or inside the double-quoted ffmpeg command line Recorder
// builds (quotes in particular).
inline std::string SanitizeFilename(std::string_view raw, const char* fallback) {
    std::string safe;
    for (char c : raw)
        safe += (std::isalnum((unsigned char)c) || c == '-' || c == '_' || c == ' ') ? c : '_';
    if (safe.empty()) safe = fallback;
    return safe;
}
