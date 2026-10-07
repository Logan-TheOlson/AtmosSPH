#pragma once
#include <filesystem>
#include <fstream>

// Channel velocity-profile logger. Every 1/rate simulated seconds it bins particles by y across the
// channel and writes two small CSVs:
//   <name>_profile.csv : frame,time,bin,y_center,count,mean_vx,std_vx,mean_vy,mean_density
//   <name>_frames.csv  : frame,time,mean_vx,max_vx,rms_vy,mean_density,std_density,min_density,max_density
// Both start with '#'-prefixed lines recording the run's geometry and parameters (read with
// pandas' comment='#'). Written synchronously on the sim thread; a sample is only nBins+1 rows.
struct ProfileLogger {
    std::ofstream profile;
    std::ofstream frames;
    int    nBins       = 0;
    int    rate        = 5;
    float  length      = 0.f;
    int    savedFrames = 0;

    bool IsActive() const { return profile.is_open(); }
    double NextSampleTime() const { return static_cast<double>(savedFrames) / rate; }
    float SavedSeconds() const { return savedFrames > 0 ? static_cast<float>(savedFrames - 1) / rate : 0.f; }

    void Stop();
    // Opens the two CSVs at `base`_profile.csv / `base`_frames.csv and records the t = 0 sample.
    bool Start(const std::filesystem::path& base, int sampleRate, float lengthSeconds, int bins);
    void Sample();
};
