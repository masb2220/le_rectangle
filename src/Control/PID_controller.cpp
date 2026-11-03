/**
 * @file PID_controller.cpp
 * @brief Configuration et calcul d une boucle PID avec feedforward.
 */

#include "Control/PID_controller.h"
#include <math.h>

/**
 * @brief Valider les parametres et initialiser un controleur PID.
 *
 * Memorise les gains, les limites de sortie et la periode.
 * Remet la memoire du controleur a zero et efface le defaut
 * lorsque la configuration est valide.
 *
 * @param[in,out] controller Controleur a configurer.
 * @param[in] kp Gain proportionnel
 * @param[in] ki Gain integral
 * @param[in] kd Gain derive
 * @param[in] CV_min Limite minimale finie de la commande.
 * @param[in] CV_max Limite maximale finie, superieure a CV_min.
 * @param[in] periode Temps entre deux calculs, fini et positif, en secondes.
 *
 * @retval true Configuration accepter.
 * @retval false Configuration refuser; configured = false et fault = true.
 *
 * @note Les champs restent modifiables. Utiliser cette fonction pour
 * changer les reglages sans contourner leur validation.
 */
bool PID_config(PID_controller &controller, float kp, float ki, float kd, float CV_min, float CV_max, float periode) {
    // Verifier les gains avant de les memoriser
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) || (kp < 0.0f) || (ki < 0.0f) || (kd < 0.0f)) {
        controller.configured = false; // Configuration refuser
        controller.fault = true; // Memoriser le defaut
        return false;
    }

    // Verifier les limites et la periode
    if (!isfinite(CV_min) || !isfinite(CV_max) || (CV_min >= CV_max) || !isfinite(periode) || (periode <= 0.0f)) {
        controller.configured = false; // Configuration refuser
        controller.fault = true; // Memoriser le defaut
        return false;
    }

    controller = PID_controller{}; // Reinitialiser apres validation

    // Memoriser les parametres valider
    controller.kp = kp; // Gain proportionnel
    controller.ki = ki; // Gain integral
    controller.kd = kd; // Gain derive
    controller.CV_min = CV_min; // Commande minimale
    controller.CV_max = CV_max; // Commande maximale
    controller.periode = periode; // Periode en secondes

    // Autoriser le controleur une fois la configuration complete
    controller.configured = true; // Autoriser les calculs

    return true;
}

/**
 * @brief Calculer la commande PID et ajouter le feedforward.
 *
 * La commande combine trois contributions :
 * - P reagit a l erreur entre la consigne et la mesure.
 * - I accumule erreur * periode pour corriger un ecart persistant.
 * - D reagit a la variation de l erreur entre deux calculs.
 *
 * Le feedforward ajoute une commande estimee a partir d un modele
 * ou d un profil. Le PID corrige l ecart restant.
 *
 * @par Anti-windup
 * La saturation est evaluer avec l ancienne integrale.
 * L accumulation est bloquer seulement si l erreur pousse davantage
 * vers la limite atteinte. Une erreur qui aide a sortir de saturation
 * peut donc encore modifier l integrale.
 *
 * @par Validation
 * L etat et les entrees sont verifier avant les calculs.
 * Les resultats restent temporaires jusqu a leur validation.
 * Un defaut active fault et retourne zero sans memoriser les nouveaux calculs.
 *
 * @param[in,out] controller Parametres et memoire du controleur.
 * @param[in] setpoint Consigne demander.
 * @param[in] PV Valeur mesurer, dans la meme unite que setpoint.
 * @param[in] feedforward Commande anticipee, dans la meme unite que la sortie.
 * Mettre zero pour ne pas utiliser d anticipation.
 *
 * @return Commande limitee entre CV_min et CV_max si le calcul est valide.
 * Retourne zero en cas de defaut, meme si zero est hors de ces limites.
 *
 * @post Un calcul valide met a jour integrale, last_error et init.
 *
 * @warning Le code appelant doit verifier fault avant d utiliser la commande.
 * Cette fonction ne commande pas directement les moteurs.
 *
 * @note Appeler a la periode configuree; la fonction ne mesure pas le temps.
 * @note La derivee est nulle au premier calcul et lorsque kd vaut zero.
 * @note Mettre kd a zero donne un PI; mettre ki et kd a zero donne un P.
 */
float compute_PID(PID_controller &controller, float setpoint, float PV, float feedforward) {
    // Verifier l etat et les entree
    if (!controller.configured || controller.fault || !isfinite(setpoint) || !isfinite(PV) || !isfinite(feedforward)) {
        controller.fault = true; // Memoriser le defaut
        return 0.0f; // Refuser la commande
    }
    // Reutiliser les calculs intermediaires sans modifier la memoire avant validation.

    const float error = setpoint - PV; // Ecart entre consigne et mesure
    const float P = controller.kp * error; //Valeur de l action proportionelle

    float deriver = 0.0f; // Variation de l erreur par seconde

    // Calculer la derivee si un historique est disponible (la premiere scan a pas de last error)
    if (controller.init && (controller.kd > 0.0f)) {
        deriver = (error - controller.last_error) / controller.periode;
    }

    const float D = controller.kd * deriver; // Valeur de l action de la derivee

    // Evaluer la saturation avec l ancienne integrale et le feedforward, cette etape est un peu complexe.
    //en gros on veut savoir si avec les valeur P+D+Feefforward et l integral de cycle d avant.
    //le but est de savoir si la sorti du controleur sature si oui alors l integral dois cesser de cumuler l erreurs.
    //car il y a rien a faire de plus pour corriger et sa vas cause des problemes de depassement rendu au sp

    const float CV_non_saturee = P + (controller.ki * controller.integrale) + D + feedforward; //valeur caluler de CV avec l action de l integral du cyle d avant

    const bool CV_saturee_MAX = (CV_non_saturee >= controller.CV_max) && (error > 0.0f); // bool saturation max
    const bool CV_saturee_MIN = (CV_non_saturee <= controller.CV_min) && (error < 0.0f); // bool saturation min

    // on veut pas overwrite directement controller.integrale on dois verifier le resultat alors on utilise une variable temporaire
    float nouvelle_integrale = controller.integrale; //integral actuel

    if ((controller.ki > 0.0f) && !CV_saturee_MAX && !CV_saturee_MIN) { //si aucune saturation et on veut de l integral dans le PID
        nouvelle_integrale = nouvelle_integrale + (error * controller.periode); // calcul de l integral actuel
    }

    // par default on definie que cv = cv_non_saturee (reste le cas si la nouvelle integral est pareil que l ancienne)
    float CV = CV_non_saturee;

    if (nouvelle_integrale != controller.integrale) { //si les deux integral sont differente on recalcul CV avec la nouvelle
        const float I = controller.ki * nouvelle_integrale; // Valeur de l action de l integral
        CV = P + I + D + feedforward; // Commande totale
    }

    // Valider avant de memoriser ou de limiter la commande (check de CV)
    if (!isfinite(error) || !isfinite(CV_non_saturee) || !isfinite(nouvelle_integrale) || !isfinite(CV)) {
        controller.fault = true; // Memoriser le defaut
        return 0.0f; // Refuser les resultats invalides
    }

    // Memoriser les resultats valider
    controller.integrale = nouvelle_integrale; // Accumulation pour le prochain calcul
    controller.last_error = error; // Historique pour la derivee
    controller.init = true; // Premier calcul valide effectuer (utile seulement pour le preimer scan)

    // Limiter la commande finale selon CV min et max
    if (CV > controller.CV_max) {
        return controller.CV_max;
    }

    if (CV < controller.CV_min) {
        return controller.CV_min;
    }

    return CV;
}
