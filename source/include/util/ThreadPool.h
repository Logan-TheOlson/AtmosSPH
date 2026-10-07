#pragma once
#include <cstdint>
#include <functional>

// Splits [0, total) into one chunk per worker thread and blocks until fn(begin, end) has run for
// every chunk. Used instead of std::execution::par, which runs sequentially on this MinGW toolchain.
void ParallelFor(int32_t total, const std::function<void(int32_t, int32_t)>& fn);
