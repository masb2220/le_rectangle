/**
 * @file PID_controller.h
 * @brief Controleur PID avec feedforward et anti-windup.
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/**
 * @brief Parametres et memoire d un controleur PID.
 *
 * Configurer avec PID_config() avant le premier calcul.
 */
struct PID_controller {
    // Gains du controleur
    float kp = 0.0f; ///< Gain proportionnel : reagit a l erreur actuelle.
    float ki = 0.0f; ///< Gain integral : corrige l erreur accumuler.
    float kd = 0.0f; ///< Gain derive : reagit a la variation de l erreur.

    // Parametres
    float CV_min = 0.0f; ///< Limite minimale de la commande.
    float CV_max = 0.0f; ///< Limite maximale de la commande.
    float periode = 0.0f; ///< Periode entre deux calculs, en secondes.

    // Memoire du controleur
    float integrale = 0.0f; ///< Accumulation de erreur * periode, avant multiplication par ki.
    float last_error = 0.0f; ///< Erreur du calcul precedent.

    // Etat
    bool init = false; ///< True apres un calcul valide; autorise ensuite la derivee.
    bool configured = false; ///< True apres une configuration valide.
    bool fault = false; ///< Defaut memoriser; une configuration valide permet de l effacer.
};

/**
 * @brief Valider les parametres et initialiser un PID.
 *
 * Memorise les gains, les limites et la periode.
 * Remet l integrale, la derniere erreur et l etat du premier calcul a zero.
 *
 * @param[in,out] controller Controleur a configurer.
 * @param[in] kp Gain proportionnel, fini et positif ou nul.
 * @param[in] ki Gain integral, fini et positif ou nul; zero desactive I.
 * @param[in] kd Gain derive, fini et positif ou nul; zero desactive D.
 * @param[in] CV_min Limite minimale finie de la commande.
 * @param[in] CV_max Limite maximale finie, strictement superieure a CV_min.
 * @param[in] periode Periode de calcul finie et strictement positive, en secondes.
 *
 * @retval true Configuration accepter et defaut effacer.
 * @retval false Parametres invalides; configured = false et fault = true.
 *
 * @note Utiliser cette fonction pour reconfigurer le PID.
 * Modifier directement ses champs contourne la validation.
 */
bool PID_config(PID_controller &controller, float kp, float ki, float kd, float CV_min, float CV_max, float periode);

/**
 * @brief Calculer une commande PID avec une anticipation optionnelle.
 *
 * L erreur est la difference entre la consigne et la mesure :
 * - P reagit a l erreur actuelle.
 * - I accumule erreur * periode.
 * - D utilise la variation de l erreur divisee par la periode.
 *
 * Le feedforward fournit une commande estimee a l avance.
 * Le PID ajoute sa correction, puis la somme est limiter entre CV_min et CV_max.
 *
 * L anti-windup bloque l accumulation si la sortie calculee avec l ancienne
 * integrale est saturee et que l erreur pousse davantage dans cette direction.
 * La derivee reste nulle au premier calcul et lorsque kd vaut zero.
 *
 * @param[in,out] controller Gains, configuration et memoire du PID.
 * @param[in] setpoint Consigne demander, dans la meme unite que PV.
 * @param[in] PV Valeur mesurer du procede.
 * @param[in] feedforward Commande anticipee, dans la meme unite que CV.
 * Zero desactive cette contribution.
 *
 * @return Commande limitee entre CV_min et CV_max en fonctionnement normal.
 * Retourne zero en cas de defaut, meme si zero est hors de ces limites.
 *
 * @post En cas de succes, met a jour integrale, last_error et init.
 * @post En cas de defaut, active fault sans memoriser les nouveaux calculs.
 *
 * @warning Verifier fault apres l appel : une sortie nulle peut aussi etre
 * une commande normale. Cette fonction ne commande pas directement le moteur.
 *
 * @note Appeler a la periode configuree : cette fonction ne mesure pas le temps.
 * @note Mettre kd a zero donne un PI; mettre ki et kd a zero donne un P.
 */
float compute_PID(PID_controller &controller, float setpoint, float PV, float feedforward = 0.0f);

#endif
