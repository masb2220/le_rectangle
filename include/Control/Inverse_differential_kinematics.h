/**
 * @file Inverse_differential_kinematics.h
 * @brief Conversion des consignes lineaire et angulaire en vitesses de roues.
 */

#ifndef INVERSE_DIFFERENTIAL_KINEMATIC_H
#define INVERSE_DIFFERENTIAL_KINEMATIC_H

/** @brief Parametres et consignes de vitesse des deux roues. */
struct wheel_setpoint {
    float left = 0.0f;  ///< Consigne roue gauche en m/s
    float right = 0.0f; ///< Consigne roue droite en m/s

    //parametre
    float wheel_spacing = 0.0f; ///< Distance entre les deux roue en metre
    float max_wheel_speed = 0.5f; ///< Limite commune en m/s, conserve le rapport des roues.

    //etat
    bool configured = false; ///< True apres une configuration accepter
    bool fault = false; ///< Defaut memoire
};


/**
 * @brief Configurer la distance entre les deux roues en metre.
 *
 * output est la structure qui garde le parametre et les deux consigne de roue.
 * Remet les consigne a zero et efface le defaut si la distance est valide.
 * Retour true si configurer; sinon false et fault memoriser.
 *
 * @param[in,out] output Parametres de cinematique et consignes de roues a memoriser.
 * @param[in] wheel_spacing Distance entre les roues, en metres.
 * @param[in] max_wheel_speed Vitesse maximale de chaque roue, en m/s.
 *
 * @return true si configurer; sinon false et fault memoriser.
 */
bool Kinematic_config(wheel_setpoint &output, float wheel_spacing, float max_wheel_speed = 0.5f);


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
bool compute_wheel_speed_setpoints(wheel_setpoint &output, float linear_speed_cmd, float angular_speed_cmd);

#endif
