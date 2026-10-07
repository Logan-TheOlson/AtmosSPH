#include "engine/ProfileLogger.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "engine/Engine.h"
#include "engine/EngineInternal.h"
#include "engine/Scene.h"

void ProfileLogger::Stop() {
    if (profile.is_open()) profile.close();
    if (frames.is_open())  frames.close();
}

bool ProfileLogger::Start(const std::filesystem::path& base, int sampleRate, float lengthSeconds, int bins) {
    Stop();
    const auto& s = ActiveScene;
    nBins       = bins > 0 ? bins : std::max(s.spawn.gridCountY, 1);
    rate        = std::clamp(sampleRate, 1, 240);
    length      = std::max(lengthSeconds, 0.f);
    savedFrames = 0;

    profile.open(base.string() + "_profile.csv", std::ios::out | std::ios::trunc);
    frames.open(base.string() + "_frames.csv", std::ios::out | std::ios::trunc);
    if (!profile.is_open() || !frames.is_open()) {
        fprintf(stderr, "ProfileLogger: failed to open %s_*.csv\n", base.string().c_str());
        Stop();
        return false;
    }

    const auto& p = s.particles;
    const bool  morris = p.viscosityModel == ViscosityModel::Morris;
    for (std::ofstream* f : { &profile, &frames }) {
        *f << "# scene=" << s.name << '\n'
           << "# H=" << 2.f * ScreenHalfHeight() << '\n'
           << "# L=" << 2.f * ScreenHalfWidth() << '\n'
           << "# N=" << objects.size() << '\n'
           << "# gridCountX=" << s.spawn.gridCountX << '\n'
           << "# gridCountY=" << s.spawn.gridCountY << '\n'
           << "# spacing=" << p.circleRadius * 3.f * s.spawn.spacingScale << '\n'
           << "# h=" << p.smoothingRadius << '\n'
           << "# eos=" << (p.eos == EosModel::WCSPH ? "WCSPH" : "Polytropic") << '\n'
           << "# stiffness=" << p.stiffness << '\n'
           << "# c=" << std::sqrt(p.stiffness) << '\n' // WCSPH reference sound speed
           << "# targetDensity=" << p.targetDensity << '\n'
           << "# viscosityModel=" << (morris ? "Morris" : "Monaghan") << '\n'
           << "# nu=" << (morris ? p.kinematicViscosity : 0.f) << '\n'
           << "# alpha=" << (morris ? 0.f : p.viscosity) << '\n'
           << "# beta=" << (morris ? 0.f : p.viscosityQuadratic) << '\n'
           << "# forceX=" << s.physics.force.x << '\n'
           << "# forceY=" << s.physics.force.y << '\n'
           << "# noSlipWalls=" << (s.physics.noSlipWalls ? 1 : 0) << '\n'
           << "# periodicX=" << (s.boundary.periodicX ? 1 : 0) << '\n'
           << "# substeps=" << s.physics.substeps << '\n'
           << "# sampleRate=" << rate << '\n'
           << "# bins=" << nBins << '\n';
    }
    profile << "frame,time,bin,y_center,count,mean_vx,std_vx,mean_vy,mean_density\n";
    frames  << "frame,time,mean_vx,max_vx,rms_vy,mean_density,std_density,min_density,max_density\n";

    Sample(); // t = 0: the initial condition, before any step
    return true;
}

void ProfileLogger::Sample() {
    if (!IsActive()) return;
    const float hh   = ScreenHalfHeight();
    const float binH = 2.f * hh / static_cast<float>(nBins);

    std::vector<int>    count(nBins, 0);
    std::vector<double> sVx(nBins, 0.0), sVx2(nBins, 0.0), sVy(nBins, 0.0), sRho(nBins, 0.0);
    double gVx = 0.0, gVy2 = 0.0, gRho = 0.0, gRho2 = 0.0;
    double maxVx = -1e30, minRho = 1e30, maxRho = -1e30;

    for (const auto& obj : objects) {
        const int b = std::clamp(static_cast<int>((obj.pos.y + hh) / binH), 0, nBins - 1);
        const double vx = obj.vel.x, vy = obj.vel.y, rho = obj.density;
        count[b]++; sVx[b] += vx; sVx2[b] += vx * vx; sVy[b] += vy; sRho[b] += rho;
        gVx += vx; gVy2 += vy * vy; gRho += rho; gRho2 += rho * rho;
        maxVx  = std::max(maxVx, vx);
        minRho = std::min(minRho, rho);
        maxRho = std::max(maxRho, rho);
    }

    const int    frame = savedFrames;
    const double t     = simTime;
    for (int b = 0; b < nBins; b++) {
        const double yc = -hh + (b + 0.5) * binH;
        const int    c  = count[b];
        const double mVx  = c ? sVx[b] / c : 0.0;
        const double var  = c ? std::max(sVx2[b] / c - mVx * mVx, 0.0) : 0.0;
        profile << frame << ',' << t << ',' << b << ',' << yc << ',' << c << ','
                << mVx << ',' << std::sqrt(var) << ','
                << (c ? sVy[b] / c : 0.0) << ',' << (c ? sRho[b] / c : 0.0) << '\n';
    }

    const double n    = std::max<double>(static_cast<double>(objects.size()), 1.0);
    const double mRho = gRho / n;
    frames << frame << ',' << t << ',' << gVx / n << ',' << maxVx << ','
           << std::sqrt(gVy2 / n) << ',' << mRho << ','
           << std::sqrt(std::max(gRho2 / n - mRho * mRho, 0.0)) << ','
           << minRho << ',' << maxRho << '\n';

    savedFrames++;
    if (length > 0.f && t + 1e-6 >= length) Stop();
}
