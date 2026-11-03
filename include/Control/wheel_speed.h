/**
 * @file wheel_speed.h
 * @brief Conversion des comptes encodeur en vitesse lineaire de roue.
 */

#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include <stdint.h>

/** @brief Configuration fixe et coefficient precalculer de conversion encodeur. */
struct wheel_speed_config {
    // Parametres
    uint32_t encoder_counts_per_motor_revolution = 0; ///< Comptes mesurer pour un tour moteur avant le gearbox
    float gearbox_ratio = 0.0f; ///< Nombre de tour moteur pour un tour de roue
    float wheel_radius_m = 0.0f; ///< Rayon de la roue en metre, soit la moitier du diametre
    float period = 0.0f; ///< Secondes

    float speed_per_count = 0.0f; ///< m/s par compte, precalculer a la configuration

    // Etat
    bool configured = false; ///< True apres une configuration accepter
};

/**
 * @brief Configurer la conversion des comptes encodeur vers une vitesse de roue.
 *
 * Encodeur placer sur le moteur avant le gearbox : comptes par tour moteur, pas par tour roue.
 * gearbox_ratio = nombre de tour moteur pour un tour de roue; diametre en metre, periode en seconde.
 * Retour true si valide; sinon false et configured reste false.
 *
 * @param[in,out] wheel Parametres et coefficient de conversion de la roue.
 * @param[in] encoder_counts_per_motor_revolution Comptes par tour moteur avant le reducteur.
 * @param[in] gearbox_ratio Nombre de tours moteur par tour de roue.
 * @param[in] wheel_diameter_m Diametre de roue, en metres.
 * @param[in] period Periode de calcul, en secondes.
 *
 * @return true si valide; sinon false et configured reste false.
 */
bool configure_wheel_speed(wheel_speed_config &wheel, uint32_t encoder_counts_per_motor_revolution, float gearbox_ratio, float wheel_diameter_m, float period);

/**
 * @brief Convertir la variation signer des comptes en vitesse lineaire de roue.
 *
 * delta_count = lecture actuel moins lecture precedente sur une periode.
 * 2 * PI converti les tours en radians; on divise par la periode puis par le gearbox.
 * Ces operations sont precalculer pour utiliser moin de temps de calcul : le scan multiplie les comptes par speed_per_count.
 * Cette fonction ne lit pas l encodeur : elle recoit les comptes deja mesurer par la librairie librobus.
 *
 * @param[in] delta_count Variation signee des comptes pendant une periode.
 * @param[in] wheel Parametres et coefficient de conversion de la roue.
 *
 * @return Vitesse signee en m/s, ou NAN si configuration ou resultat invalide.
 */
float compute_wheel_speed(int32_t delta_count, const wheel_speed_config &wheel);

#endif
