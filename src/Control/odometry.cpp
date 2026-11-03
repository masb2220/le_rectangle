/**
 * @file odometry.cpp
 * @brief Estimation de la position avec les vitesses mesurees des roues.
 */

#include "Control/odometry.h"
#include <math.h>

static constexpr float PI_VALUE = 3.14159265358979323846f; // Valeur de pi pour les conversions angulaires.

/**
 * @brief Ramener un angle entre -PI et +PI
 *
 * Ramener un angle en radians entre -PI et +PI sans changer son orientation.
 * Enleve les tours complet avec fmodf, puis ajuste le cote de la coupure.
 * Retour l angle normaliser; ne modifie aucune structure.
 *
 * @param[in] angle Angle a normaliser, en radians.
 *
 * @return Angle equivalent entre -PI et +PI, en radians.
 */
static float normalize_angle(float angle) {
    if ((angle >= -PI_VALUE) && (angle <= PI_VALUE)) {
        return angle; // Deja dans la plage, aucun tour a enlever
    }

    float normalized = fmodf(angle, 2.0f * PI_VALUE); //sinon on enleve des tour

    //on le ramene entre -180 et 180 degre pour trouver les chemin le plus court
    if (normalized > PI_VALUE){
        normalized -= (2.0f * PI_VALUE);
    }
    else if (normalized < -PI_VALUE){
        normalized += (2.0f * PI_VALUE);
    }

    return normalized;
}

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
bool odometry_config(odometry &robot, float wheel_spacing, float period, float initial_x, float initial_y, float initial_theta) {
    robot.configured = false;

    // Verifier les parametres
    if (!isfinite(wheel_spacing) || (wheel_spacing <= 0.0f) || !isfinite(period) || (period <= 0.0f) || !isfinite(initial_x) || !isfinite(initial_y) || !isfinite(initial_theta)) {
        robot.fault = true;
        return false;
    }

    const float inverse_wheel_spacing = 1.0f / wheel_spacing; // Une seule division a la config
    if (!isfinite(inverse_wheel_spacing) || (inverse_wheel_spacing <= 0.0f)) {
        robot.fault = true; // Inverse non representable
        return false;
    }

    robot = odometry{}; // Reinitialiser apres validation

    // Parametres
    robot.inverse_wheel_spacing = inverse_wheel_spacing; // Coefficient reutiliser a chaque scan pour sauver du temps de calcul
    robot.wheel_spacing = wheel_spacing;
    robot.period = period;

    // Position initiale
    robot.x = initial_x;
    robot.y = initial_y;
    robot.theta = normalize_angle(initial_theta);

    robot.configured = true;

    return true;
}

/**
 * @brief Estimer la nouvelle position a partir des vitesse mesurer des deux roues en m/s.
 *
 * La moyenne donne la vitesse lineaire; la difference / entraxe donne la rotation en rad/s.
 * Multiplie par la periode pour avoir le deplacement signer et la variation d angle.
 * Met a jours l angle puis projette le deplacement sur X et Y avec ce nouvel angle.
 * Retour true si mise a jours valide. C est une estimation, le glissement des roues cree une erreur.
 * Etat du robot
 * Vitesse gauche en m/s
 * Vitesse droite en m/s
 *
 * @param[in,out] robot Position et vitesses estimees du robot.
 * @param[in] left_speed Vitesse mesuree gauche, en m/s.
 * @param[in] right_speed Vitesse mesuree droite, en m/s.
 *
 * @return true si mise a jours valide.
 */
bool compute_odometry(odometry &robot, float left_speed, float right_speed) {
    // Etat invalide
    // Mesure gauche invalide
    // Mesure droite invalide
    if (!robot.configured || robot.fault || !isfinite(left_speed) || !isfinite(right_speed)) {
        robot.fault = true; // Memoriser le defaut
        return false;       // Calcul refuse
    }

    const float linear_speed = (left_speed / 2.0f) + (right_speed / 2.0f); // Vitesse moyenne en m/s

    const float angular_speed = (right_speed - left_speed) * robot.inverse_wheel_spacing; // Rotation en rad/s

    const float distance = linear_speed * robot.period; // Deplacement signe pendant la periode

    const float angle_change = angular_speed * robot.period; // Variation d'angle en radians pendant la periode

    //calcul de la variation d angle total du robot
    const float angle_before_normalization = robot.theta + angle_change; // Ancien angle + variation

    // Valider les resultats avant la normalisation et la projection
    if (!isfinite(linear_speed) || !isfinite(angular_speed) || !isfinite(distance) || !isfinite(angle_change) || !isfinite(angle_before_normalization)) {
        robot.fault = true; // Memoriser le defaut
        return false;       // Conserver la position precedente
    }

    const float new_theta = normalize_angle(angle_before_normalization); // normalisation de l angle entre -PI et +PI

    float new_x = robot.x; // Garder X si aucun deplacement
    float new_y = robot.y; // Garder Y si aucun deplacement

    if (distance != 0.0f) { // si un deplement a eu lieu
        new_x += (distance * cosf(new_theta)); // Projeter sur X
        new_y += (distance * sinf(new_theta)); // Projeter sur Y
    }

    // Verifier l'orientation, Verifier X, Verifier Y
    if (!isfinite(new_theta) || !isfinite(new_x) || !isfinite(new_y)) {
        robot.fault = true; // Memoriser le defaut
        return false;       // Refuser la mise a jour
    }

    robot.linear_speed = linear_speed;   // Memoriser la vitesse lineaire
    robot.angular_speed = angular_speed; // Memoriser la vitesse angulaire
    robot.x = new_x;                     // Memoriser X
    robot.y = new_y;                     // Memoriser Y
    robot.theta = new_theta;             // Memoriser l'orientation

    return true; // Calcul reussi
}
