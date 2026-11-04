#pragma once
#include <Arduino.h>

constexpr uint8_t LINE_PINS[3] = {A10, A11, A12}; // Gauche, milieu, droite.
// Valeurs approchees relevees dans les mesures de l'utilisateur (G/M/D).
constexpr uint16_t LINE_WHITE_DEFAULT[3] = {46, 32, 30};
constexpr uint16_t LINE_BLACK_DEFAULT[3] = {443, 500, 451};

constexpr float LINE_BASE_SPEED = 0.55f;
constexpr float LINE_KP = 0.14f;
constexpr float LINE_KI = 0.005f;
constexpr float LINE_KD = 0.005f;
constexpr float LINE_MAX_TURN = 0.17f;
constexpr float LINE_ERROR_FILTER_S = 0.020f;
constexpr float LINE_DERIVATIVE_FILTER_S = 0.030f;
constexpr float LINE_ACCEL_PER_S = 0.60f;
constexpr float LINE_DECEL_PER_S = 1.80f;
constexpr float LINE_STEERING_SLEW_PER_S = 1.20f;
constexpr float LINE_STEERING_RELEASE_PER_S = 3.00f;
constexpr float LINE_INTEGRAL_LIMIT = 0.15f;
constexpr uint16_t LINE_ERROR_DENOMINATOR_MIN = 700;
constexpr float LINE_MAX_SPEED = 0.48f;
constexpr unsigned long LINE_PERIOD_MS = 10UL;
constexpr uint8_t LINE_CALIBRATION_SAMPLES = 32;
constexpr int LINE_MIN_CONTRAST = 80;
constexpr uint16_t LINE_MIN_SIGNAL = 120; // Sensible au bord de ligne (~170 observe).
constexpr uint16_t LINE_WEAK_SIGNAL = 300;
constexpr float LINE_RECOVERY_BRIDGE_SPEED = 0.05f;
constexpr float LINE_RECOVERY_ARC_SPEED = 0.025f;
constexpr float LINE_RECOVERY_TURN = 0.10f;
constexpr uint8_t LINE_REACQUIRE_SAMPLES = 3;
constexpr float LINE_WEAK_SPEED_SCALE = 0.5f;

void lineBegin();
void lineUpdate();
void lineStop();
bool lineStart();
void lineCalibrate(bool black);
const char *lineEvent();
bool lineFollowing();
bool lineCalibrationReady();
void lineReadStatus(uint16_t raw[3], uint16_t darkness[3]);




const char *lineMode();
void lineReadCalibration(uint16_t white[3], uint16_t black[3]);
