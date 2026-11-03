/**
 * @file speed_profile.h
 * @brief Profil de deplacement theorique : acceleration, vitesse constante et freinage.
 */

#ifndef SPEED_PROFILE_H
#define SPEED_PROFILE_H

/**
 * @brief Parametres et memoire d un profil trapezoidal de vitesse.
 *
 * Le profil fournit une position, une vitesse et une acceleration
 * theoriques a chaque instant.
 *
 * Pour un deplacement court, le profil devient triangulaire :
 * acceleration puis freinage, sans phase a vitesse constante.
 *
 * Configurer avec profile_config(), puis charger un deplacement
 * avec profile_start() avant d appeler compute_profile().
 *
 * @note En translation, utiliser les metres et les secondes.
 * En rotation, utiliser les radians et les secondes.
 */
struct speed_profile {
    // Limites de creation du profil
    float V_MAX = 0.0f; ///< Limite positive de vitesse, en m/s ou rad/s.
    float A_MAX = 0.0f; ///< Limite positive d acceleration et de freinage, en m/s² ou rad/s².
    float period = 0.0f; ///< Periode entre deux calculs, en secondes.

    // Deplacement demander
    float deplacement = 0.0f; ///< Deplacement total signer depuis le depart, en m ou rad.

    // Consignes theoriques
    float position = 0.0f; ///< Deplacement prevu depuis le depart a cet instant, en m ou rad.
    float vitesse = 0.0f; ///< Vitesse theorique signee a cet instant, en m/s ou rad/s.
    float acceleration = 0.0f; ///< Acceleration theorique signee de la phase actuelle, en m/s² ou rad/s².

    // Parametres calculer au depart
    float direction = 1.0f; ///< Sens du profil : +1 ou -1 selon le deplacement.
    float peak_speed = 0.0f; ///< Pointe de vitesse positive prevue pour ce trajet, au plus V_MAX.
    float acceleration_time = 0.0f; ///< Duree de chaque phase d acceleration et de freinage, en secondes.
    float cruise_time = 0.0f; ///< Duree a vitesse constante, nulle pour un profil triangulaire.
    float move_time = 0.0f; ///< Duree theorique totale du deplacement, en secondes.
    float elapsed = 0.0f; ///< Temps du profil deja parcouru, en secondes.

    // Etat
    bool configured = false; ///< True apres une configuration valide.
    bool started = false; ///< True lorsqu un deplacement a ete charger.
    bool finished = false; ///< Fin theorique du profil; ne confirme pas l arrivee du robot.
    bool fault = false; ///< Defaut memoriser; une configuration valide permet de l effacer.
};

/**
 * @brief Valider les limites et initialiser le profil de vitesse.
 *
 * Valide les parametres, efface l ancien profil, puis memorise les limites de vitesse,
 * d acceleration et la periode de calcul.
 * Aucun deplacement n est charger par cette fonction.
 *
 * @param[in,out] profile Profil a configurer.
 * @param[in] max_speed Vitesse maximale finie et strictement positive, en m/s ou rad/s.
 * @param[in] max_acceleration Acceleration maximale finie et strictement positive, en m/s² ou rad/s².
 * @param[in] period Periode de calcul finie et strictement positive, en secondes.
 *
 * @retval true Configuration accepter et defaut effacer.
 * @retval false Parametres invalides; configured vaut false et fault vaut true.
 *
 * @note Utiliser cette fonction pour modifier les limites.
 * Modifier directement les champs contourne leur validation.
 */
bool profile_config(speed_profile &profile, float max_speed, float max_acceleration, float period);

/**
 * @brief Preparer un profil pour un deplacement signer.
 *
 * Calcule le sens, la pointe de vitesse et la duree des phases.
 * Si la distance est trop courte pour atteindre V_MAX, prepare
 * un profil triangulaire avec une pointe de vitesse reduite.
 *
 * Remet le temps ecoule, la position, la vitesse et l acceleration
 * a zero. Un deplacement nul est immediatement considere terminer.
 *
 * @param[in,out] profile Profil deja configurer, sans defaut.
 * @param[in] displacement Deplacement total fini et signer, en m ou rad.
 * Une valeur negative demande le sens inverse.
 *
 * @retval true Profil preparer et started vaut true.
 * @retval false Preparation refuser; fault vaut true et les sorties
 * de vitesse et d acceleration sont annulees.
 *
 * @pre profile_config() doit avoir reussi.
 *
 * @note Le profil suppose une vitesse initiale et finale nulle.
 * @note Un nouvel appel remplace le profil precedent.
 */
bool profile_start(speed_profile &profile, float displacement);

/**
 * @brief Avancer le profil d une periode et calculer ses consignes.
 *
 * Avance elapsed de period, sans depasser move_time.
 * Calcule ensuite position, vitesse et acceleration selon
 * la phase actuelle : acceleration, plateau ou freinage.
 *
 * La position est calculer avec les equations du profil,
 * sans accumuler vitesse * periode a chaque appel.
 *
 * A la fin, position atteint le deplacement demander,
 * vitesse et acceleration valent zero, et finished devient true.
 *
 * @param[in,out] profile Profil preparer dont les consignes et le temps sont mis a jour.
 *
 * @retval true Calcul valide, ou profil deja terminer.
 * @retval false Profil indisponible ou calcul invalide; fault vaut true
 * et les sorties de vitesse et d acceleration sont annulees.
 *
 * @pre profile_config() et profile_start() doivent avoir reussi.
 *
 * @note Appeler a la periode configuree : la fonction ne mesure pas le temps reel.
 * @note Aucune position ni vitesse mesuree du robot n est utiliser ici.
 * Le PID corrige ensuite l ecart entre ces consignes et les mesures.
 * @note finished indique la fin du profil, pas la stabilisation physique du robot.
 */
bool compute_profile(speed_profile &profile);

#endif
