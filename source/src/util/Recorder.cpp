#include "util/Recorder.h"
#include <algorithm>
#include <chrono>
#include <sstream>
#include <thread>
#ifndef _WIN32
#include <csignal>
#endif

Recorder::~Recorder() { Stop(); }

bool Recorder::Start(const std::string& outputPath, int width, int height, int fps, const char* pixelFormat) {
    Stop();

#ifndef _WIN32
    // Writing to the pipe after ffmpeg exits raises SIGPIPE, which would kill this process by
    // default. Ignoring it makes the fwrite in WriterLoop fail with EPIPE instead.
    signal(SIGPIPE, SIG_IGN);
#endif

    // The sim's thread pool already saturates every hardware thread, so cap x264 to a quarter of
    // the machine (with the cheapest preset) to keep encoding from slowing the sim.
    const unsigned encoderThreads = std::max(1u, std::thread::hardware_concurrency() / 4);

    std::ostringstream cmd;
    cmd << "ffmpeg -y -loglevel error"
        << " -f rawvideo -pixel_format " << pixelFormat
        << " -video_size " << width << 'x' << height
        << " -framerate " << fps
        << " -i - -c:v libx264 -threads " << encoderThreads << " -preset ultrafast -crf 20 -pix_fmt yuv420p"
        << " \"" << outputPath << "\"";

#ifdef _WIN32
    pipe_ = _popen(cmd.str().c_str(), "wb");
#else
    pipe_ = popen(cmd.str().c_str(), "w");
#endif
    if (!pipe_) {
        fprintf(stderr, "Recorder: failed to launch ffmpeg (is it on PATH?)\n");
        return false;
    }

    stopping_   = false;
    workerDone_ = false;
    worker_     = std::thread(&Recorder::WriterLoop, this);
    return true;
}

void Recorder::SubmitFrame(const void* data, size_t bytes, std::function<void()> release) {
    if (!pipe_) { release(); return; }

    std::lock_guard<std::mutex> lock(queueMutex_);
    if (queue_.size() >= kMaxQueuedFrames) { release(); return; } // encoder can't keep up: drop the frame

    queue_.push_back({ data, bytes, std::move(release) });
    queueCv_.notify_one();
}

void Recorder::WriterLoop() {
    while (true) {
        Job job;
        {
            std::unique_lock<std::mutex> lock(queueMutex_);
            queueCv_.wait(lock, [&] { return !queue_.empty() || stopping_; });
            if (queue_.empty() && stopping_) break;
            job = std::move(queue_.front());
            queue_.pop_front();
        }
        // The slow read of `job.data` happens here, off the caller's thread.
        fwrite(job.data, 1, job.bytes, pipe_);
        job.release();
    }

    std::lock_guard<std::mutex> lock(doneMutex_);
    workerDone_ = true;
    doneCv_.notify_all();
}

void Recorder::Stop() {
    if (!pipe_) return;

    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        stopping_ = true;
    }
    queueCv_.notify_all();

    // Bound the wait for the worker to drain: a stalled ffmpeg can block it in fwrite() forever,
    // and an unbounded join() here would freeze the main thread.
    constexpr auto kDrainTimeout = std::chrono::seconds(5);
    bool drained;
    {
        std::unique_lock<std::mutex> lock(doneMutex_);
        drained = doneCv_.wait_for(lock, kDrainTimeout, [&] { return workerDone_; });
    }

    if (!drained) {
        fprintf(stderr, "Recorder: ffmpeg pipe stalled, abandoning writer thread\n");
        worker_.detach(); // still owns pipe_, so leak it rather than close it underneath
        pipe_ = nullptr;
        queue_.clear();
        return;
    }

    if (worker_.joinable()) worker_.join(); // already returned, so immediate

#ifdef _WIN32
    _pclose(pipe_);
#else
    pclose(pipe_);
#endif
    pipe_ = nullptr;
    queue_.clear();
}
