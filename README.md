# Stjarna

A 2D particle-fluid simulator built on Vulkan and SDL3. It runs a Smoothed Particle Hydrodynamics (SPH) solver with optional self-gravity, and has a docked ImGui interface for building scenes, running them live, and recording the results as video or CSV data.

## Features

**Simulation**
- Weakly-compressible SPH, multithreaded, with a uniform-grid neighbour search.
- Two equations of state: Becker-Teschner WCSPH and a pure polytrope.
- Two viscosity models: Monaghan artificial viscosity and Morris physical viscosity.
- Optional self-gravity, using either Barnes-Hut (3D 1/r² law restricted to the plane) or genuine 2D (log-potential, 1/r force). It can periodically validate the approximation against brute force.
- A constant external force, wall restitution and friction (with optional no-slip walls), and configurable substeps.
- Periodic-x boundaries for channel-flow tests, with mirrored real particles across the seam.
- Rect and circle obstacles that particles bounce off and that mirror particles across their surface for SPH density.
- Timed events applied to an area: a one-shot velocity kick, a continuous push, or a vector field (curl, attract, repel, optionally fading toward the edge).

**Scenes**
- Five built-in presets: Open Tank, Poiseuille Flow, Packed, Empty and Self-Gravity.
- Particles spawn from editable groups, each a rect lattice or a circular spiral with its own spacing and colour.
- Persistent colours: patches, stripes, checkers, rings, gradients, or an image (PNG/BMP) applied at spawn.
- A live colour-by mode for speed, pressure or density.
- User presets can be saved to `presets/*.scenepreset` (a plain `key=value` text format) and are listed next to the built-in ones.

**Output**
- Video recording to MP4 by piping frames to ffmpeg (needs `ffmpeg` on `PATH`).
- Batch rendering: an offscreen, non-real-time render for a set length of simulated time. Runs can be queued.
- Per-particle CSV data capture (position, velocity, speed, density, pressure) to `data/`.
- Velocity-profile logging for channel-flow scenes, written as `<name>_profile.csv` and `<name>_frames.csv`.
- Output files get matching names that encode the scene, run number, particle count and key parameters.

## Using the app

The window is a docked layout around the simulation view: a left panel, a right panel and a slim top bar. The layout is saved in `imgui.ini`.

- **Left panel:** the Setup tabs (Scene, Physics, Output) before a run, then the live run controls and the batch-rendering status while running.
- **Right panel and floating windows:** Events in the right panel, and Groups, Obstacles and Persistent Color in their own dockable windows. Event areas can also be placed by dragging in the sim view.
- **Keyboard** (when no text field has focus):

| Key | Action |
|-----|--------|
| Space | Pause / resume |
| Right Arrow | Advance one frame |
| Left Arrow | Advance ten frames |

## Architecture

```
source/
  src/main.cpp                SPH core: spatial grid, density, pressure, viscosity, Init()/Update()
  src/engine/Engine.cpp       App state machine, sim worker thread, main loop, all UI panels
  src/engine/Scene.cpp        Built-in presets and scene construction
  src/engine/ScenePresetIO.cpp  Save/load user presets
  src/engine/Object.cpp       Per-particle radius and draw
  src/physics/                Barnes-Hut quadtree and gravity (3D-in-plane and genuine 2D)
  src/renderer/               App, SDL window, Vulkan context (instanced circle/rect rendering,
                              offscreen capture)
  src/ui/                     Dock workspace and on-canvas event/obstacle gizmos
  src/util/                   ThreadPool, Recorder (ffmpeg), DataRecorder (CSV), colour patterns,
                              run naming, profiler, Vec2
  shaders/                    GLSL, compiled to SPIR-V at build time
  include/                    Headers, mirroring src/
```

The simulation steps on a worker thread, guarded by a mutex shared with the UI thread. If the worker stalls, the UI stays responsive and shows a "not responding" notice. Batch renders step the simulation directly from the main thread and draw offscreen, so they are not limited by vsync.

Coordinates in scenes are world pixels, with the origin at the window centre and +y up.

## Dependencies

- **Vulkan SDK** with `glslc` — set the `VULKAN_SDK` environment variable.
- **SDL3** — set `SDL3_ROOT` (default `C:/SDL3/x86_64-w64-mingw32`); it must contain `lib/` and `bin/SDL3.dll`.
- **Dear ImGui** (docking branch) and **volk** — fetched automatically by CMake.
- **ffmpeg** — optional, only for video recording; must be on `PATH`.
- A C++20 compiler. The project is set up for Windows, with a MinGW preset in `CMakePresets.json`.

## Build

```bash
cd source
cmake -B cmake-build-release -DCMAKE_BUILD_TYPE=Release
cmake --build cmake-build-release
```

Shaders and the Karla font are copied next to the executable in `bin/` as part of the build. Recordings, data, and presets are written next to the executable in `recordings/`, `data/` and `presets/`.

## Configuration

`Config.h` holds only the window title and size. Everything else (boundary, spawn layout, physics, particle constants, gravity, events, obstacles, groups and colours) belongs to a `Scene` (`Scene.h`) and is edited in the UI. The active scene is available as `ActiveScene`.
