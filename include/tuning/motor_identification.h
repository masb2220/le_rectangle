/**
 * @file motor_identification.h
 * @brief Identification premier ordre avec retard et proposition de gains PI SIMC.
 */

#ifndef MOTOR_IDENTIFICATION_H
#define MOTOR_IDENTIFICATION_H
#include <stdint.h>
#include <math.h>
#include "tuning/robot_characterization.h"

/** @brief Captures normalisees et resultat du modele premier ordre avec retard. */
struct motor_identification {
    int16_t curves[2][20] = {}; ///< Reponse normalisee, resolution 0.0001
    float gains[2] = {}; ///< Gains statiques des deux echelon, en m/s par unite de commande.
    bool captured[2] = {}; ///< Validite de chacune des deux captures.
    float gain = 0.0f; ///< Gain statique en m/s par unite de commande.
    float tau = 0.0f; ///< Constante de temps du modele, en secondes.
    float delay = 0.0f; ///< Retard identifier, en secondes; zero peut etre sous la resolution.
    float error = 1.0e9f; ///< Somme des erreurs carrees pendant la recherche, puis erreur RMS normalisee.
    float kp = 0.0f; ///< Gain proportionnel proposer par SIMC.
    float ki = 0.0f; ///< Gain integral proposer, sous la forme kp / Ti.
    unsigned int trial=0; ///< Index de l hypothese tester, de 0 a 439.
    bool configured=false; ///< True lorsque la configuration ou le modele est accepter.
};

/**
 * @brief Conserver deux echelon non nul: 0.20->0.30 et 0.30->0.40, dans chaque sens.
 *
 * @param[in,out] model Parametres et etat du modele moteur.
 * @param[in] step Index du palier d identification : 0 ou 1.
 * @param[in] measure Mesures du palier de caracterisation.
 * @param[in] initial_speed Vitesse initiale, en m/s.
 * @param[in] final_speed Vitesse finale, en m/s.
 * @param[in] command_change Variation non nulle de commande moteur.
 *
 * @note step doit valoir 0 ou 1. La capture exige 100 scans et un plateau suffisamment stable.
 */
inline void capture_motor_step(motor_identification &model, unsigned int step, const characterization_measurement &measure, float initial_speed, float final_speed, float command_change) {
    const float change=final_speed-initial_speed;
    model.captured[step]=false;
    if (!isfinite(change) || fabsf(change)<0.04f || command_change==0.0f || measure.ticks!=100) { return; }
    model.gains[step]=change/command_change;
    if (!isfinite(model.gains[step]) || model.gains[step]<=0.0f) { return; }
    // Refuser un plateau encore en mouvement par rapport a l amplitude de l echelon.
    if (fabsf(measure.trace[19]-measure.trace[16])>fmaxf(0.01f,0.10f*fabsf(change))) { return; }
    for (unsigned int i=0; i<20; i++) {
        const float value=(measure.trace[i]-initial_speed)/change;
        if (!isfinite(value) || value < -0.5f || value>1.5f) { return; }
        model.curves[step][i]=(int16_t)lroundf(value*10000.0f);
    }
    model.captured[step]=true;
}

/**
 * @brief Moyenne exacte d une reponse premier ordre sur une fenetre de 50 ms.
 *
 * @param[in] index Index de l element a traiter.
 * @param[in] tau Constante de temps strictement positive, en secondes.
 * @param[in] delay Retard du modele, en secondes.
 *
 * @return Reponse normalisee moyenne sur la fenetre de 50 ms.
 *
 * @note index entre 0 et 19; le modele represente une moyenne, pas une mesure instantanee.
 */
inline float model_step_bin(unsigned int index, float tau, float delay) {
    const float a=fmaxf(0.0f,index*0.05f-delay);
    const float b=fmaxf(0.0f,(index+1)*0.05f-delay);
    return ((b-a)+tau*(expf(-b/tau)-expf(-a/tau)))/0.05f;
}

/**
 * @brief Une hypothese par appel, moteurs couper: le moniteur reste disponible entre les calcul.
 *
 * Ajustement sur le premier echelon; le second est une verification independante.
 *
 * @param[in,out] model Parametres et etat du modele moteur.
 *
 * @return True si le traitement est terminer, meme sans modele valide; consulter configured.
 *
 * @note Une hypothese par appel, uniquement moteurs couper. Reinitialiser le modele avant une nouvelle identification; arreter les appels au retour true.
 */
inline bool identify_motor_update(motor_identification &model) {
    if (!model.captured[0] || !model.captured[1]) { return true; }
    const float tau=0.01f*(1+model.trial%40);
    const float delay=0.01f*(model.trial/40);
    float error=0.0f;
    for (unsigned int i=0; i<20; i++) {
        const float difference=model_step_bin(i,tau,delay)-model.curves[0][i]*0.0001f;
        error+=difference*difference;
    }
    if (error<model.error) { model.error=error; model.tau=tau; model.delay=delay; }
    if (++model.trial<440) { return false; }
    float validation_error=0.0f;
    for (unsigned int i=0; i<20; i++) {
        const float difference=model_step_bin(i,model.tau,model.delay)-model.curves[1][i]*0.0001f;
        validation_error+=difference*difference;
    }
    model.error=sqrtf(model.error/20.0f);
    model.gain=model.gains[0];
    model.configured=model.error<=0.08f && sqrtf(validation_error/20.0f)<=0.10f && fabsf(model.gains[1]/model.gain-1.0f)<=0.25f && model.tau>0.0101f && model.tau<0.399f && model.delay<0.099f;
    if (model.configured) {
        // SIMC PI, lambda volontairement prudent; ki=kp/Ti pour notre PID parallele.
        const float lambda=fmaxf(0.05f,fmaxf(model.tau,3.0f*model.delay));
        const float integral_time=fminf(model.tau,4.0f*(lambda+model.delay));
        model.kp=model.tau/(model.gain*(lambda+model.delay));
        model.ki=model.kp/integral_time;
        model.configured=isfinite(model.kp) && isfinite(model.ki) && model.kp>0.0f && model.ki>0.0f;
    }
    return true;
}
#endif
