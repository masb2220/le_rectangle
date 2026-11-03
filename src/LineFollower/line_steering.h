#pragma once

constexpr float lineAbs(float value) { return value < 0 ? -value : value; }
constexpr float lineMin(float a, float b) { return a < b ? a : b; }

// Transition continue : petite zone neutre et gain progressif pres du centre.
constexpr float lineShapeError(float error)
{
    return lineAbs(error) <= 0.025f ? 0.0f :
        (error < 0 ? -1.0f : 1.0f) * ((lineAbs(error)-0.025f)/0.975f) *
        (0.45f + 0.55f*lineMin(lineAbs(error)/0.40f, 1.0f));
}

// Reserver une marge symetrique pour la correction, avant saturation des roues.
constexpr float lineCruiseLimit(float requested, float maximum, float turnMargin)
{
    return lineMin(requested, maximum > turnMargin ? maximum-turnMargin : 0.0f);
}
