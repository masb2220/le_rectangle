#include "cup_return.h"
constexpr float absValue(float a) { return a < 0 ? -a : a; }
static_assert(cupReturnDistance(-0.25f) == 0.25f, "Return opposes rear pickup");
static_assert(cupReturnDistance(0.25f) == -0.25f, "Inverse also works for forward travel");
static_assert(cupReturnDistance(-0.35f) == 0.35f, "Same distance when configured travel changes");
static_assert(cupReturnDistance(0) == 0, "No travel when no pickup");
static_assert(cupReturnAngle(1.0f, 1.0f) == 0, "Already aligned");
static_assert(cupReturnAngle(0.5f, 1.0f) == -0.5f, "Restore right");
static_assert(cupReturnAngle(1.0f, 0.5f) == 0.5f, "Restore left");
static_assert(absValue(cupReturnAngle(CUP_PI-0.1f,-CUP_PI+0.1f)+0.2f) < 0.0001f, "Shortest path across pi");
static_assert(absValue(cupReturnAngle(-CUP_PI+0.1f,CUP_PI-0.1f)-0.2f) < 0.0001f, "Opposite wrap");
