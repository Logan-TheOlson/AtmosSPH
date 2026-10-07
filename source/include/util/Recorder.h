#pragma once
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

// Pipes raw pixel frames to an external ffmpeg process, which encodes them to an mp4. ffmpeg must
// be on PATH; if the pipe can't be opened, Start() fails quietly (IsActive() stays false). If
// ffmpeg dies mid-recording, writes fail instead of killing the process, and Stop() gives up on a
// stalled pipe after a few seconds.
//
// SubmitFrame() only queues a job; a background thread reads `data` (typically GPU-visible
// memory), writes it to ffmpeg, then calls `release`. The caller must keep `data` valid until
// `release` fires. If the queue exceeds kMaxQueuedFrames, new frames are dropped (`release` is
// called immediately) so a slow encoder never stalls the caller.
class Recorder {
public:
    bool Start(const std::string& outputPath, int width, int height, int fps, const char* pixelFormat);
    void SubmitFrame(const void* data, size_t bytes, std::function<void()> release);
    void Stop();
    bool IsActive() const { return pipe_ != nullptr; }

    ~Recorder();

private:
    static constexpr size_t kMaxQueuedFrames = 4;

    struct Job {
        const void*           data;
        size_t                bytes;
        std::function<void()> release;
    };

    void WriterLoop();

    FILE*       pipe_{ nullptr };
    std::thread worker_;

    std::mutex              queueMutex_;
    std::condition_variable queueCv_;
    std::deque<Job>         queue_;
    bool                    stopping_{ false };

    // Signaled by WriterLoop just before it returns, so Stop() can bound its wait (a stalled
    // ffmpeg can leave the worker stuck in fwrite()).
    std::mutex              doneMutex_;
    std::condition_variable doneCv_;
    bool                    workerDone_{ false };
};
