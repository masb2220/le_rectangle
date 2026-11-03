#pragma once
#include <stdint.h>

// Angle par rapport a l'orientation ou le capteur gauche detecte l'objet.
// Positif = tourner a gauche ; negatif = tourner a droite.
constexpr float SCAN_OFFSET_DEG = -2.0f;
constexpr float OFFSET_STEP_DEG = 1.0f;
constexpr float SCAN_SPEED_RAD_S = 0.20f;
constexpr float LEFT_DETECTION_MAX_CM = 50.0f;

constexpr unsigned long BUMPER_DEBOUNCE_MS = 40;


// Espace cible estime entre le capteur et le verre. La pince reste relevee.
constexpr float CLAW_CLEARANCE_CM = 30.0f;



// Constantes requises par la sequence actuelle.
constexpr float OFFSET_SPEED_RAD_S = 0.20f;
constexpr float SCAN_RESTART_RIGHT_DEG = 25.0f;
constexpr float EARLY_DETECTION_DEG = 5.0f;
constexpr uint8_t MAX_SCAN_RESTARTS = 3;
constexpr float PICKUP_REVERSE_MM = 250.0f;
constexpr unsigned long MOTION_SETTLE_MS = 400UL;
constexpr unsigned long ARM_MOVE_WAIT_MS = 2000UL;
constexpr unsigned long THUMB_CLOSE_WAIT_MS = 500UL;
constexpr unsigned long DISTANCE_SAMPLE_TIMEOUT_MS = 500UL;
constexpr unsigned long SCAN_TIMEOUT_MS = 70000UL;
// Translation de prise/retour a cap fixe, en metres par seconde.
constexpr float PICKUP_STRAIGHT_SPEED_M_S = 0.12f;
