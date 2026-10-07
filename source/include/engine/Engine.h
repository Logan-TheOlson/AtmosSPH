#pragma once
#include "Object.h"
#include "Renderable.h"
#include <cstddef>
#include <vector>

// Scene state
extern std::vector<Object> objects;

// Object management
Object& CreateObject(float x, float y, Renderable r);

// Runs the application until the window closes; returns the process exit code.
int RunEngine();

// Screen dimensions (available after Init)
float ScreenHalfWidth();
float ScreenHalfHeight();
