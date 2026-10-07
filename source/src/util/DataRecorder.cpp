#include "util/DataRecorder.h"
#include <cstdio>
#include <sstream>

DataRecorder::~DataRecorder() { Stop(); }

bool DataRecorder::Start(const std::string& outputPath, const Fields& fields) {
    Stop();

    file_.open(outputPath, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) {
        fprintf(stderr, "DataRecorder: failed to open %s\n", outputPath.c_str());
        return false;
    }
    fields_ = fields;

    file_ << "frame,time,id";
    if (fields_.position) file_ << ",x,y";
    if (fields_.velocity) file_ << ",vx,vy";
    if (fields_.speed)    file_ << ",speed";
    if (fields_.density)  file_ << ",density";
    if (fields_.pressure) file_ << ",pressure";
    file_ << '\n';

    stopping_ = false;
    worker_   = std::thread(&DataRecorder::WriterLoop, this);
    return true;
}

void DataRecorder::SubmitFrame(int frame, float time, std::vector<ParticleSample> samples) {
    if (!file_.is_open()) return;

    std::lock_guard<std::mutex> lock(queueMutex_);
    if (queue_.size() >= kMaxQueuedFrames) return; // writer can't keep up: drop the frame

    queue_.push_back({ frame, time, std::move(samples) });
    queueCv_.notify_one();
}

void DataRecorder::WriterLoop() {
    while (true) {
        Job job;
        {
            std::unique_lock<std::mutex> lock(queueMutex_);
            queueCv_.wait(lock, [&] { return !queue_.empty() || stopping_; });
            if (queue_.empty() && stopping_) return;
            job = std::move(queue_.front());
            queue_.pop_front();
        }
        // Formatting happens here, off the caller's thread.
        WriteJob(job);
    }
}

void DataRecorder::WriteJob(const Job& job) {
    std::ostringstream out;
    for (size_t i = 0; i < job.samples.size(); i++) {
        const auto& s = job.samples[i];
        out << job.frame << ',' << job.time << ',' << i;
        if (fields_.position) out << ',' << s.x << ',' << s.y;
        if (fields_.velocity) out << ',' << s.vx << ',' << s.vy;
        if (fields_.speed)    out << ',' << s.speed;
        if (fields_.density)  out << ',' << s.density;
        if (fields_.pressure) out << ',' << s.pressure;
        out << '\n';
    }
    file_ << out.str();
}

void DataRecorder::Stop() {
    if (!file_.is_open()) return;

    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        stopping_ = true;
    }
    queueCv_.notify_all();
    if (worker_.joinable()) worker_.join(); // drains the queue before the file closes

    file_.close();
    queue_.clear();
}
