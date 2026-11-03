/**
 * @file odometry.h
 * @brief Estimation de la position avec les vitesses mesurees des roues. utiliser pour mettre a jours les consigne des PID
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

/** @brief Parametres et position estimee du robot. */
struct odometry {
    // Parametre
    float wheel_spacing = 0.0f; ///< distance entre roue en metres
    //(1/x) permet de faire une multiplication a chaque cycle par une division
    float inverse_wheel_spacing = 0.0f; ///< Inverse calculer a la config, reconfigurer si entraxe change

    float period = 0.0f;  ///< Temps entre deux calcul en seconde

    // Position estime
    float x = 0.0f;             ///< Metre
    float y = 0.0f;             ///< Metre
    float theta = 0.0f;         ///< Radians

    // Vitesse estimee
    float linear_speed = 0.0f;  ///< m/s
    float angular_speed = 0.0f; ///< rad/s

    // Etat
    bool configured = false; ///< True apres une configuration accepter
    bool fault = false; ///< Defaut memoriser, reste actif jusqu a une reconfiguration valide
};

/**
 * @brief Initialiser l estimation de position du robot.
 *
 * Entrer : entraxe et X/Y initial en metre, periode en seconde, angle initial en radians.
 * Angle zero vers +X, angle positif vers +Y (rotation anti horaire).
 * Memorise la position de depart, normalise l angle et remet les vitesses a zero.
 * Retour true si valide; sinon false avec fault. Une config valide efface le defaut.
 *
 * @param[in,out] robot Position et vitesses estimees du robot.
 * @param[in] wheel_spacing Distance entre les roues, en metres.
 * @param[in] period Periode de calcul, en secondes.
 * @param[in] initial_x Position initiale X, en metres.
 * @param[in] initial_y Position initiale Y, en metres.
 * @param[in] initial_theta Orientation initiale, en radians.
 *
 * @return true si valide; sinon false avec fault. Une config valide efface le defaut.
 */
bool odometry_config(odometry &robot, float wheel_spacing, float period, float initial_x, float initial_y, float initial_theta);

/**
 * @brief Estimer la nouvelle position a partir des vitesse mesurer des deux roues en m/s.
 *
 * La moyenne donne la vitesse lineaire; la difference / entraxe donne la rotation en rad/s.
 * Multiplie par la periode pour avoir le deplacement signer et la variation d angle.
 * Met a jours l angle puis projette le deplacement sur X et Y avec ce nouvel angle.
 * Retour true si mise a jours valide. C est une estimation, le glissement des roues cree une erreur.
 *
 * @param[in,out] robot Position et vitesses estimees du robot.
 * @param[in] left_speed Vitesse mesuree gauche, en m/s.
 * @param[in] right_speed Vitesse mesuree droite, en m/s.
 *
 * @return true si mise a jours valide. C est une estimation, le glissement des roues cree une erreur.
 */
bool compute_odometry(odometry &robot, float left_speed, float right_speed);
#endif
