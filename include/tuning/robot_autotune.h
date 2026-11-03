/**
 * @file robot_autotune.h
 * @brief Evaluation et classement des candidats PI sur le banc roues libres.
 */

#ifndef ROBOT_AUTOTUNE_H
#define ROBOT_AUTOTUNE_H

/** @brief Mesures accumulees pendant les 200 scans d un essai. */
struct tuning_metrics {
    float target = 0.0f; ///< Consigne pendant les 1.5 premiere seconde
    float error_sum = 0.0f; ///< Erreur absolue accumuler pendant le suivi
    float end_error_sum = 0.0f; ///< Erreur absolue sur les 300 derniere ms du palier
    float zero_error_sum = 0.0f; ///< Vitesse absolue sur les 200 derniere ms a zero
    float overshoot = 0.0f; ///< Depassement relatif de la consigne
    float command_variation = 0.0f; ///< Variation totale de commande
    float previous_command = 0.0f; ///< Commande du scan precedent pour calculer sa variation.
    unsigned int saturated = 0; ///< Nombre de scans a la limite de commande.
    unsigned int stable_ticks = 0; ///< Nombre de scans consecutifs dans la bande de tolerance.
    unsigned int final_stable_ticks = 0; ///< Derniere duree stable observee pendant le suivi, en scans.
    unsigned int ticks = 0; ///< Nombre de scans accumules.
};

/** @brief Score et criteres de validation d un candidat PI. */
struct tuning_result {
    float score = 0.0f; ///< Plus petit est meilleur
    float mean_error_ratio = 0.0f; ///< Erreur absolue moyenne rapportee a la consigne.
    float final_error_ratio = 0.0f; ///< Erreur absolue finale rapportee a la consigne.
    float stop_speed = 0.0f; ///< Vitesse absolue moyenne sur les 200 dernieres ms, en m/s.
    float overshoot = 0.0f; ///< Depassement relatif maximal dans le sens de la consigne.
    float saturation_ratio = 0.0f; ///< Fraction des 200 scans a la limite de commande.
    float settling_time = -1.0f; ///< Derniere entree durable dans la bande avant la fin du palier
    bool acceptable = false; ///< True si tous les criteres de validation sont respecter.
};

static constexpr unsigned int TUNING_CANDIDATES = 25; ///< Cinq facteurs de kp fois cinq facteurs de ki.

/**
 * @brief Calculer un couple de gains dans la grille locale autour du reglage SIMC.
 *
 * Grille locale de -30 a +30 pourcent autour du calcul SIMC.
 *
 * @param[in] index Index de l element a traiter.
 * @param[out] kp Gain proportionnel du candidat.
 * @param[out] ki Gain integral du candidat.
 * @param[in] base_kp Gain proportionnel issu du modele SIMC.
 * @param[in] base_ki Gain integral issu du modele SIMC.
 *
 * @note index entre 0 et 24. Les gains proposes ne sont pas automatiquement appliquer au programme normal.
 */
void tuning_candidate(unsigned int index, float &kp, float &ki, float base_kp, float base_ki);

/**
 * @brief Reinitialiser les mesures pour un essai de suivi puis d arret.
 *
 * @param[in,out] metrics Mesures accumulees pendant l essai.
 * @param[in] target Consigne de vitesse signee, en m/s.
 */
void tuning_begin(tuning_metrics &metrics, float target);

/**
 * @brief Accumuler les mesures d un scan de 10 ms pour evaluer un candidat.
 *
 * Scan 0..149 : suivi; scan 150..199 : consigne zero pour verifier le ralentissement.
 *
 * @param[in,out] metrics Mesures accumulees pendant l essai.
 * @param[in] speed Vitesse mesuree signee, en m/s.
 * @param[in] command Commande moteur normalisee.
 * @param[in] command_limit Limite positive de commande moteur.
 *
 * @note Consigne non nulle, mesures finies et limite positive. Appeler 200 fois a 10 ms : suivi 150 scans, puis arret 50 scans.
 */
void tuning_sample(tuning_metrics &metrics, float speed, float command, float command_limit);

/**
 * @brief Calculer le score et les criteres d acceptation d un essai complet.
 *
 * @param[in] metrics Mesures accumulees pendant l essai.
 *
 * @return Mesures et score; acceptable reste false si l essai est incomplet ou refuse.
 */
tuning_result tuning_finish(const tuning_metrics &metrics);
#endif
