#include "LineFollower/line_steering.h"
static_assert(lineShapeError(0.0f) == 0.0f, "Center must not steer");
static_assert(lineShapeError(0.02f) == 0.0f, "Ignore tiny positive noise");
static_assert(lineShapeError(-0.02f) == 0.0f, "Ignore tiny negative noise");
static_assert(lineShapeError(0.05f) > 0 && lineShapeError(0.05f) < 0.025f, "Soften small corrections");
static_assert(lineShapeError(-0.05f) == -lineShapeError(0.05f), "Steering must be symmetric");
static_assert(lineShapeError(0.8f) > 0.75f, "Keep recovery authority far from center");
static_assert(lineShapeError(0.1f) < lineShapeError(0.2f), "Correction must grow with error");
static_assert(lineAbs(lineCruiseLimit(0.8f,0.48f,0.17f)-0.31f) < 0.00001f, "Reserve margin despite excessive request");
static_assert(lineCruiseLimit(0.22f,0.48f,0.17f) == 0.22f, "Preserve lower requested speed");
static_assert(lineCruiseLimit(0.8f,0.1f,0.17f) == 0, "Handle unavailable margin");
