#include "util/ColorPattern.h"
#include <SDL3/SDL.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

struct ColorImage {
    int w = 0, h = 0;
    std::vector<uint8_t> rgba;
};

namespace {
    struct Cache {
        std::string path;
        bool        tried = false;
        ColorImage  image;
        bool        ok = false;
    };
    Cache cache;

    const ColorImage* LoadCached(const std::string& path) {
        if (path.empty()) return nullptr;
        if (!cache.tried || cache.path != path) {
            cache = Cache{};
            cache.path  = path;
            cache.tried = true;
            if (SDL_Surface* raw = SDL_LoadSurface(path.c_str())) {
                if (SDL_Surface* conv = SDL_ConvertSurface(raw, SDL_PIXELFORMAT_RGBA32)) {
                    cache.image.w = conv->w;
                    cache.image.h = conv->h;
                    cache.image.rgba.resize(static_cast<size_t>(conv->w) * conv->h * 4);
                    for (int y = 0; y < conv->h; y++)
                        std::copy_n(static_cast<const uint8_t*>(conv->pixels) + static_cast<size_t>(y) * conv->pitch,
                                    static_cast<size_t>(conv->w) * 4,
                                    cache.image.rgba.begin() + static_cast<size_t>(y) * conv->w * 4);
                    cache.ok = true;
                    SDL_DestroySurface(conv);
                }
                SDL_DestroySurface(raw);
            }
        }
        return cache.ok ? &cache.image : nullptr;
    }

    Color Lerp(const Color& a, const Color& b, float t) {
        return { a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t, a.a + (b.a - a.a) * t };
    }
}

PatternSampler::PatternSampler(const SceneColorPattern& pattern, float x0, float y0, float x1, float y1)
    : p(pattern), minX(x0), minY(y0), maxX(x1), maxY(y1) {
    if (p.kind == ColorPatternKind::Image) image = LoadCached(p.imagePath);
}

bool PatternSampler::Sample(float x, float y, Color& out) const {
    const float cell = std::max(p.cellSize, 1.f);
    const float w = maxX - minX, h = maxY - minY;
    switch (p.kind) {
        case ColorPatternKind::None:
            return false;
        case ColorPatternKind::VStripes:
            out = (static_cast<long>(std::floor((x - minX) / cell)) & 1) ? p.colorB : p.colorA;
            return true;
        case ColorPatternKind::HStripes:
            out = (static_cast<long>(std::floor((y - minY) / cell)) & 1) ? p.colorB : p.colorA;
            return true;
        case ColorPatternKind::Checker: {
            const long ix = static_cast<long>(std::floor((x - minX) / cell));
            const long iy = static_cast<long>(std::floor((y - minY) / cell));
            out = ((ix + iy) & 1) ? p.colorB : p.colorA;
            return true;
        }
        case ColorPatternKind::Rings: {
            const float r = std::hypot(x - (minX + maxX) * 0.5f, y - (minY + maxY) * 0.5f);
            out = (static_cast<long>(std::floor(r / cell)) & 1) ? p.colorB : p.colorA;
            return true;
        }
        case ColorPatternKind::Gradient:
            out = Lerp(p.colorA, p.colorB, w > 0.f ? std::clamp((x - minX) / w, 0.f, 1.f) : 0.f);
            return true;
        case ColorPatternKind::Image: {
            if (!image || w <= 0.f || h <= 0.f) return false;
            // Fit inside the block keeping the image's aspect, centered; +y is up in the world, down in the image.
            const float scale = std::min(w / static_cast<float>(image->w), h / static_cast<float>(image->h));
            const float u = (x - (minX + maxX) * 0.5f) / scale + image->w * 0.5f;
            const float v = image->h * 0.5f - (y - (minY + maxY) * 0.5f) / scale;
            const int px = static_cast<int>(std::floor(u)), py = static_cast<int>(std::floor(v));
            if (px < 0 || py < 0 || px >= image->w || py >= image->h) return false;
            const uint8_t* s = &image->rgba[(static_cast<size_t>(py) * image->w + px) * 4];
            if (s[3] < 128) return false;
            out = { s[0] / 255.f, s[1] / 255.f, s[2] / 255.f, 1.f };
            return true;
        }
    }
    return false;
}

std::string PatternImageStatus(const SceneColorPattern& pattern) {
    if (pattern.kind != ColorPatternKind::Image) return "";
    if (pattern.imagePath.empty()) return "No image chosen.";
    const ColorImage* img = LoadCached(pattern.imagePath);
    if (!img) return "Couldn't load (PNG or BMP only).";
    return "Loaded " + std::to_string(img->w) + " x " + std::to_string(img->h) + " px.";
}
