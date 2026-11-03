/**
 * @file motor_feedforward.h
 * @brief Identification et calcul du feedforward moteur offset plus pente.
 */

#ifndef MOTOR_FEEDFORWARD_H
#define MOTOR_FEEDFORWARD_H
#include <math.h>
#include "tuning/robot_characterization.h"

/** @brief Modele commande = offset + pente * vitesse absolue. */
struct motor_feedforward {
    float static_command = 0.0f; ///< Offset du modele, pas un seuil de demarrage mesurer
    float command_per_speed = 0.0f; ///< Commande par m/s
    float fit_error = 0.0f; ///< Erreur RMS en unite de commande
    bool configured = false; ///< True lorsque la configuration ou le modele est accepter.
};

/**
 * @brief Ajuster CV = offset + pente * vitesse sur les sept paliers montant jusqu a CV=0.50.
 *
 * @param[in,out] model Parametres et etat du modele moteur.
 * @param[in] points Au moins sept paliers montant valides, de CV 0.05 a 0.50.
 *
 * @return True si le modele est acceptable; sinon configured reste false.
 */
inline bool configure_motor_feedforward(motor_feedforward &model, const characterization_point *points) {
    model = motor_feedforward{};
    float sx=0.0f, sy=0.0f, sxx=0.0f, sxy=0.0f;
    for (unsigned int i=0; i<7; i++) {
        const float x=fabsf(points[i].mean_speed), y=fabsf(points[i].command);
        if (!isfinite(x) || !isfinite(y) || x<=0.0f || (i>0 && x<=fabsf(points[i-1].mean_speed))) { return false; }
        sx+=x; sy+=y; sxx+=x*x; sxy+=x*y;
    }
    const float denominator=7.0f*sxx-sx*sx;
    if (!isfinite(denominator) || denominator<=0.000001f) { return false; }
    model.command_per_speed=(7.0f*sxy-sx*sy)/denominator;
    model.static_command=(sy-model.command_per_speed*sx)/7.0f;
    if (!isfinite(model.command_per_speed) || model.command_per_speed<=0.0f || !isfinite(model.static_command) || model.static_command<0.0f || model.static_command>0.20f) { return false; }
    float squared_error=0.0f;
    for (unsigned int i=0; i<7; i++) {
        const float error=model.static_command+model.command_per_speed*fabsf(points[i].mean_speed)-fabsf(points[i].command);
        squared_error+=error*error;
    }
    model.fit_error=sqrtf(squared_error/7.0f);
    model.configured=isfinite(model.fit_error) && model.fit_error<=0.03f;
    return model.configured;
}

/**
 * @brief Zero exact a consigne zero; le signe vient de la consigne, jamais du bruit encodeur.
 *
 * @param[in] model Parametres et etat du modele moteur.
 * @param[in] target Consigne de vitesse signee, en m/s.
 *
 * @return Commande moteur signee; zero a consigne zero, NAN si modele ou consigne invalide.
 */
inline float compute_motor_feedforward(const motor_feedforward &model, float target) {
    if (!model.configured || !isfinite(target)) { return NAN; }
    if (target==0.0f) { return 0.0f; }
    return copysignf(model.static_command+model.command_per_speed*fabsf(target), target);
}
#endif
