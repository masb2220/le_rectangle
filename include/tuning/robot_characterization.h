/**
 * @file robot_characterization.h
 * @brief Accumulation et bilan des mesures de reponse moteur.
 */

#ifndef ROBOT_CHARACTERIZATION_H
#define ROBOT_CHARACTERIZATION_H

/** @brief Bilan d un palier de commande moteur. */
struct characterization_point {
    float command = 0.0f; ///< Commande normaliser
    float mean_speed = 0.0f; ///< Moyenne signer des 400 derniere ms
    float speed_noise = 0.0f; ///< Ecart type des mesure sur le plateau
    float rise_time = -1.0f; ///< Temps a 90 pourcent du changement, -1 si non mesurable
    float peak_acceleration = 0.0f; ///< Acceleration moyenne sur 50 ms, pas un pic instantane
};

/** @brief Memoire d acquisition des 100 scans d un palier. */
struct characterization_measurement {
    float trace[20] = {}; ///< Moyennes de 50 ms pendant une seconde
    float group_sum = 0.0f; ///< Somme des cinq vitesses du groupe courant, en m/s.
    float tail_sum = 0.0f; ///< Somme des vitesses des 40 derniers scans, en m/s.
    float tail_square_sum = 0.0f; ///< Somme des carres des vitesses des 40 derniers scans.
    float initial_speed = 0.0f; ///< Vitesse avant le palier, en m/s.
    unsigned int ticks = 0; ///< Nombre de scans accumules.
};

/**
 * @brief Reinitialiser la mesure pour un palier de commande moteur.
 *
 * Recommencer la mesure pour un palier de commande moteur.
 *
 * @param[in,out] measure Mesures du palier de caracterisation.
 * @param[in] initial_speed Vitesse initiale, en m/s.
 */
void characterization_begin(characterization_measurement &measure, float initial_speed);

/**
 * @brief Accumuler une vitesse mesuree par scan de 10 ms pendant une seconde.
 *
 * Ajouter une vitesse mesuree a chaque scan de 10 ms, pendant 100 scan.
 *
 * @param[in,out] measure Mesures du palier de caracterisation.
 * @param[in] speed Vitesse mesuree signee, en m/s.
 *
 * @note Les appels au dela de 100 scans sont ignores.
 */
void characterization_sample(characterization_measurement &measure, float speed);

/**
 * @brief Calculer le bilan du palier a partir des mesures accumulees.
 *
 * Produire le bilan d un palier; les temps sont des estimations sur les fenetres de 50 ms.
 *
 * @param[in] measure Mesures du palier de caracterisation.
 * @param[in] command Commande moteur normalisee.
 *
 * @return Commande, vitesse moyenne, bruit, temps a 90 pourcent et acceleration sur 50 ms.
 *
 * @note Appeler apres 100 scans. Les 40 derniers servent au plateau; les temps utilisent des moyennes de 50 ms.
 */
characterization_point characterization_finish(const characterization_measurement &measure, float command);
#endif
