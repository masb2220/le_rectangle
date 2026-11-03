/**
 * @file wheel_speed.cpp
 * @brief Conversion des comptes encodeur en vitesse lineaire de roue.
 */

#include "Control/wheel_speed.h"
#include <math.h>

static constexpr float PI_VALUE = 3.14159265358979323846f; // Valeur de pi pour les conversions angulaires.

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
bool configure_wheel_speed(wheel_speed_config &wheel, uint32_t encoder_counts_per_motor_revolution, float gearbox_ratio, float wheel_diameter_m, float period) {
    // Interdire l utilisation si la configuration echoue
    wheel.configured = false;

    // Nombre de comptes strictement positif
    if (encoder_counts_per_motor_revolution == 0) {
        return false;
    }

    // Parametres physiques finis et strictement positifs
    if (!isfinite(gearbox_ratio) || (gearbox_ratio <= 0.0f) || !isfinite(wheel_diameter_m) || (wheel_diameter_m <= 0.0f) || !isfinite(period) || (period <= 0.0f)) {
        return false;
    }
    // Precacul de la conversion du diametre en rayon
    const float wheel_radius_m = wheel_diameter_m / 2.0f;

    //validation du resultat
    if (!isfinite(wheel_radius_m) || (wheel_radius_m <= 0.0f)) {
        return false;
    }

    // Conversion pour un compte, calculer une seule fois au lieu de chaque scan car trois division par scan c est assez lourd comme calcul
    const float speed_per_count = (((2.0f * PI_VALUE) / encoder_counts_per_motor_revolution) / period / gearbox_ratio) * wheel_radius_m;

    //validation du resultat
    if (!isfinite(speed_per_count) || speed_per_count <= 0.0f) {
        return false;
    }

    wheel = wheel_speed_config{}; // Reinitialiser apres validation

    // Memoriser la configuration valide
    wheel.encoder_counts_per_motor_revolution = encoder_counts_per_motor_revolution;
    wheel.gearbox_ratio = gearbox_ratio;
    wheel.wheel_radius_m = wheel_radius_m;
    wheel.period = period;
    wheel.speed_per_count = speed_per_count; // Recalculer a chaque nouvelle configuration
    wheel.configured = true;
    return true;
}

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
float compute_wheel_speed(int32_t delta_count, const wheel_speed_config &wheel) {
    // Verifier que la configuration a ete effectuee
    if (!wheel.configured) {
        return NAN;
    }

    // Les parametres fixe sont deja regrouper dans le coefficient speed_per_count pre calculer.
    // on cast delta_count en float parce que toute les calcul suivant sont en float
    const float wheel_speed_m_s = static_cast<float>(delta_count) * wheel.speed_per_count;

    // verification du resultat avant de l utiliser.
    if (!isfinite(wheel_speed_m_s)) {
        return NAN;
    }

    return wheel_speed_m_s;
}
