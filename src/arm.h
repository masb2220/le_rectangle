#pragma once
#include <Arduino.h>

constexpr uint8_t FLIP_SERVO = 0;
constexpr uint8_t THUMB_SERVO = 1;
constexpr uint8_t FLIP_MIN = 0;
constexpr uint8_t FLIP_MAX = 171;
constexpr uint8_t THUMB_MIN = 79;
constexpr uint8_t THUMB_MAX = 152;

// Inverser ces positions si le montage fonctionne dans l'autre sens.
constexpr uint8_t FLIP_DOWN_ANGLE = FLIP_MIN;
constexpr uint8_t FLIP_UP_ANGLE = FLIP_MAX;
constexpr uint8_t THUMB_OPEN_ANGLE = THUMB_MAX;
constexpr uint8_t THUMB_CLOSED_ANGLE = THUMB_MIN;

void armBegin();
void armUpdate(); // Appeler dans loop pour executer le jiggle.
void armDisable();
void closeThumb();
void openThumb();
void flipUp();
void flipDown();
// Un cycle = ecart de 1 degre puis retour. Vers -1 a la limite haute.
void jiggle(uint16_t repetitions, unsigned long intervalMs = 150);
void armPrintStatus();
