#pragma once
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// One particle's sampled state for a CSV row. Always fully populated; DataRecorder writes only
// the columns selected in Fields at Start().
struct ParticleSample {
    float x, y;
    float vx, vy;
    float speed;
    float density;
    float pressure;
};

// Writes particle-field CSV rows off the sim thread, like Recorder: SubmitFrame() queues a job and
// a background thread formats and writes it. Each Job owns its samples, so when the queue is full
// the frame is simply dropped.
class DataRecorder {
public:
    struct Fields {
        bool position = true;
        bool velocity = true;
        bool speed    = false;
        bool density  = true;
        bool pressure = true;
    };

    bool Start(const std::string& outputPath, const Fields& fields);
    void SubmitFrame(int frame, float time, std::vector<ParticleSample> samples);
    void Stop();
    bool IsActive() const { return file_.is_open(); }

    ~DataRecorder();

private:
    static constexpr size_t kMaxQueuedFrames = 4;

    struct Job {
        int                         frame;
        float                       time;
        std::vector<ParticleSample> samples;
    };

    void WriterLoop();
    void WriteJob(const Job& job);

    std::ofstream file_;
    Fields        fields_{};
    std::thread   worker_;

    std::mutex              queueMutex_;
    std::condition_variable queueCv_;
    std::deque<Job>         queue_;
    bool                    stopping_{ false };
};
