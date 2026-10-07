#include "util/RunNaming.h"
#include <cctype>
#include <cstdio>
#include <system_error>

namespace RunNaming {
namespace {
    // %g trims trailing zeros (20.0f -> "20"), keeping filenames compact.
    std::string FormatCompact(float v) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%g", v);
        return buf;
    }
}

std::string AbbreviateSceneName(const std::string& sceneName) {
    if (sceneName == "Open Tank")       return "OT";
    if (sceneName == "Poiseuille Flow") return "PF";
    if (sceneName == "Self-Gravity")    return "SG";

    std::string initials;
    bool atWordStart = true;
    for (char c : sceneName) {
        if (std::isspace(static_cast<unsigned char>(c))) { atWordStart = true; continue; }
        if (atWordStart && std::isalnum(static_cast<unsigned char>(c))) {
            initials += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
            atWordStart = false;
            if (initials.size() >= 4) break;
        } else {
            atWordStart = false;
        }
    }
    return initials.empty() ? "CU" : initials;
}

std::string BuildBaseName(const RunNameParams& p) {
    char runTag[16];
    std::snprintf(runTag, sizeof(runTag), "_run%03d", p.runNumber);

    std::string base = AbbreviateSceneName(p.sceneName);
    base += runTag;
    base += "_" + std::to_string(p.particleCount) + "p";
    if (p.gravityEnabled) base += "_g" + FormatCompact(p.gravityG);
    base += "_k" + FormatCompact(p.stiffness);
    return base;
}

int NextRunNumber() {
    static int counter = 0;
    return ++counter;
}

std::string EnsureUniqueBaseName(const std::string& baseName, const std::vector<OutputTarget>& targets) {
    auto anyExists = [&](const std::string& name) {
        for (const auto& t : targets) {
            std::error_code ec;
            if (std::filesystem::exists(t.dir / (name + t.extension), ec)) return true;
        }
        return false;
    };

    if (!anyExists(baseName)) return baseName;
    for (int suffix = 2; suffix < 10000; suffix++) {
        std::string candidate = baseName + "_" + std::to_string(suffix);
        if (!anyExists(candidate)) return candidate;
    }
    return baseName; // give up disambiguating past 10000 collisions
}

} // namespace RunNaming
