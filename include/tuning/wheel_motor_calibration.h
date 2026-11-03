/**
 * @file wheel_motor_calibration.h
 * @brief Gains et coefficients moteur retenus apres validation a vide.
 */

#ifndef WHEEL_MOTOR_CALIBRATION_H
#define WHEEL_MOTOR_CALIBRATION_H
#include "Control/motor_feedforward.h"

// Resultats roues libres : index 0 gauche, 1 droite; sens 0 positif, 1 negatif.
static constexpr float WHEEL_KP[2]={1.153f,1.058f};
static constexpr float WHEEL_KI[2]={15.328f,14.813f};
static constexpr float WHEEL_COMMAND_LIMIT=0.80f; // Meme limite totale que le banc de validation
static constexpr float WHEEL_FF_OFFSET[2][2]={{0.01360f,0.01464f},{0.01373f,0.01699f}};
static constexpr float WHEEL_FF_SLOPE[2][2]={{1.09815f,1.08511f},{1.09835f,1.04187f}};

/**
 * @brief Estimer la vitesse commune disponible a partir des quatre modeles moteur.
 * @return Minimum des vitesses (commande limite - offset) / pente, en m/s.
 * NAN si la calibration est invalide. Estimation roues libres, pas une mesure en charge.
 * @note Appeler a la configuration seulement; aucun calcul ajoute aux scans.
 */
inline float calibrated_wheel_speed_limit() {
    if (!isfinite(WHEEL_COMMAND_LIMIT) || WHEEL_COMMAND_LIMIT <= 0.0f || WHEEL_COMMAND_LIMIT > 1.0f) {
        return NAN;
    }
    float limit = INFINITY;
    for (unsigned int wheel = 0; wheel < 2; ++wheel) {
        for (unsigned int direction = 0; direction < 2; ++direction) {
            const float offset = WHEEL_FF_OFFSET[wheel][direction];
            const float slope = WHEEL_FF_SLOPE[wheel][direction];
            if (!isfinite(offset) || offset < 0.0f || offset >= WHEEL_COMMAND_LIMIT || !isfinite(slope) || slope <= 0.0f) {
                return NAN;
            }
            const float speed = (WHEEL_COMMAND_LIMIT - offset) / slope;
            if (!isfinite(speed) || speed <= 0.0f) {
                return NAN;
            }
            if (speed < limit) {
                limit = speed;
            }
        }
    }
    return limit;
}

/**
 * @brief Commande prevue selon la roue et le sens demander, nulle a consigne zero.
 *
 * @param[in] wheel Index de roue : 0 gauche, 1 droite.
 * @param[in] target Consigne de vitesse signee, en m/s.
 *
 * @return Commande moteur signee; zero a consigne zero, NAN si entree invalide.
 */
inline float calibrated_wheel_feedforward(unsigned int wheel, float target) {
    if (wheel>=2 || !isfinite(target)) { return NAN; }
    if (target==0.0f) { return 0.0f; } // Aucune commande a vitesse nulle
    const unsigned int direction=target<0.0f ? 1 : 0; // Coefficients selon le sens
    const float command=WHEEL_FF_OFFSET[wheel][direction]+WHEEL_FF_SLOPE[wheel][direction]*fabsf(target); // Modele calibrer direct
    return copysignf(command,target); // Remettre le signe demander
}
#endif
