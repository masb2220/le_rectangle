/**
 * @file Kinematic.cpp
 * @brief Conversion des consignes lineaire et angulaire en vitesses de roues.
 */

#include "Control/Inverse_differential_kinematics.h"
#include <math.h>

/**
 * @brief Configurer la distance entre les deux roues en metre.
 *
 * output est la structure qui garde le parametre et les deux consigne de roue.
 * Remet les consigne a zero et efface le defaut si la distance est valide.
 * Retour true si configurer; sinon false et fault memoriser.
 *
 * @param[in,out] output Parametres de cinematique et consignes de roues a memoriser.
 * @param[in] wheel_spacing Distance entre les roues, en metres.
 *
 * @return true si configurer; sinon false et fault memoriser.
 */
bool Kinematic_config(wheel_setpoint &output, float wheel_spacing, float max_wheel_speed) { //fonction de configuration
    //verification de la distance entre les roues
    if (!isfinite(wheel_spacing) || (wheel_spacing <= 0.0f) || !isfinite(max_wheel_speed) || max_wheel_speed <= 0.0f) {
        output.configured = false;
        output.fault = true;
        return false;
    }

    output = wheel_setpoint{}; // Reinitialiser apres validation

    // Parametre
    output.wheel_spacing = wheel_spacing;
    output.max_wheel_speed = max_wheel_speed;

    // Etat
    output.configured = true;

    return true;
}

/**
 * @brief Transformer la vitesse demander au robot en consigne pour chaque roue.
 *
 * Entrer : linear_speed_cmd en m/s, puis angular_speed_cmd en rad/s.
 * Retour true si calcul valide; sinon false, fault = true et les deux sortie a zero.
 *
 * @param[in,out] output Parametres de cinematique et consignes de roues a memoriser.
 * @param[in] linear_speed_cmd Consigne lineaire, en m/s.
 * @param[in] angular_speed_cmd Consigne angulaire, en rad/s.
 *
 * @return true si calcul valide; sinon false, fault=true et les deux sortie a zero.
 */
bool compute_wheel_speed_setpoints(wheel_setpoint &output, float linear_speed_cmd, float angular_speed_cmd) {
    // Verifier la configuration et les commandes
    if (!output.configured || output.fault || !isfinite(linear_speed_cmd) || !isfinite(angular_speed_cmd)) {
        output.fault = true;
        output.left = 0.0f;
        output.right = 0.0f;
        return false;
    }

    // effet de la rotation sur la vitesses des roues
    const float rotation_speed = angular_speed_cmd * (output.wheel_spacing / 2.0f);

    // Consignes en metres par seconde
    const float left = linear_speed_cmd - rotation_speed;
    const float right = linear_speed_cmd + rotation_speed;

    // Verifier avant de memoriser
    if (!isfinite(rotation_speed) || !isfinite(left) || !isfinite(right)) {
        output.fault = true;
        output.left = 0.0f;
        output.right = 0.0f;
        return false;
    }

    const float peak = fmaxf(fabsf(left), fabsf(right));
    const float scale = peak > output.max_wheel_speed ? output.max_wheel_speed / peak : 1.0f;
    output.left = left * scale;
    output.right = right * scale;

    return true;
}
