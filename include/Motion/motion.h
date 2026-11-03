/**
 * @file motion.h
 * @brief Execution de deux mouvements elementaires : rotation et translation.
 *
 * Ce module suit un angle cible ou une distance signee avec les profils et PID.
 * Il ne connait pas les cibles X/Y du sequenceur et ne commande pas les moteurs.
 */
#ifndef MOTION_H
#define MOTION_H

#include "Control/PID_controller.h"
#include "Control/odometry.h"
#include "Control/speed_profile.h"

enum class MotionFault { NONE, INVALID_STATE, TIMEOUT, POSITION_MISSED };

/** @brief Profils, tolerances et memoire du mouvement elementaire actif. */
struct motion_controller {
    // Profil de chaque type de mouvement
    speed_profile linear_profile; ///< Position en metre et vitesse en m/s
    speed_profile angular_profile; ///< Angle en radian et vitesse en rad/s
    bool profile_initialized = false; ///< Depart de l etape deja memoriser
    float start_x = 0.0f; ///< Position au debut de translation
    float start_y = 0.0f; ///< Position au debut de translation
    float start_angle = 0.0f; ///< Axe fixe de translation ou angle de depart
    float start_cos = 1.0f; ///< Cosinus memoriser au debut de chaque translation
    float start_sin = 0.0f; ///< Sinus memoriser au debut de chaque translation

    float forward_lateral_correction_m_per_m = 0.0f;
    float reverse_lateral_correction_m_per_m = 0.0f;
    float lateral_target = 0.0f; // Decalage final dans la normale gauche du cap initial.

    // Commandes vers inverse differential kinematic
    float linear_speed_cmd = 0.0f;  ///< m/s
    float angular_speed_cmd = 0.0f; ///< rad/s

    // Tolerances (l erreur sera jamais de 0 alors on dois definir des tolerance pour definir si le mouvevent est stabiliser et arriver au SP)
    float position_tolerance = 0.01f;      ///< Metres
    float angle_tolerance = 0.01f;         ///< Radians
    float linear_speed_tolerance = 0.01f;  ///< m/s
    float angular_speed_tolerance = 0.01f; ///< rad/s

    // Correction laterale pendant la translation
    float lateral_gain = 2.0f; ///< Correction de cap en rad/m; zero conserve seulement le maintien de cap.
    float lateral_max_angle = 0.174533f; ///< Correction maximale en radians (10 degres).
    float lateral_fade_distance = 0.10f; ///< Distance restante en metres sur laquelle revenir au cap initial.

    // Temporisation (temps de stabilisation idealement un multiple du tempd de cylce)
    float required_stable_time = 0.1f; ///< Secondes
    float stable_time = 0.0f;          ///< Temps stable accumule
    float period = 0.0f;               ///< Periode en secondes
    float timeout_margin = 3.0f; ///< Secondes ajoutees a deux fois la duree du profil.
    float elapsed = 0.0f; ///< Temps reel de commande compte en cycles.

    // Etat
    bool configured = false; ///< True apres une configuration accepter
    bool fault = false; ///< Defaut memoriser, reste actif jusqu a une reconfiguration valide
    MotionFault fault_reason = MotionFault::NONE;
};

/**
 * @brief Valider les profils et tolerances, puis preparer le controleur.
 * @param[in,out] motion Profils et tolerances deja configurer; memoire de suivi a reinitialiser.
 * @param[in] period Periode commune de calcul, en secondes.
 * @return True si accepter; sinon configured=false, fault=true et commandes nulles.
 * @note Conserver les profils et tolerances : ils sont des parametres d entree.
 */
bool configure_motion(motion_controller &motion, float period);

/**
 * @brief Preparer un nouveau mouvement sans changer les reglages ni effacer un defaut.
 * @param[in,out] motion Memoire de suivi; les commandes et le temps stable sont remis a zero.
 * @param[in,out] position_pid Historique du PID de position a effacer.
 * @param[in,out] angle_pid Historique du PID d angle a effacer.
 * @note Appeler avant de changer le type ou la cible du mouvement.
 */
void reset_motion(motion_controller &motion, PID_controller &position_pid, PID_controller &angle_pid);

/**
 * @brief Executer un scan de rotation vers un angle absolu par le chemin court.
 * @param[in,out] motion Profil angulaire, tolerances et memoire du suivi.
 * @param[in] robot Position et vitesses estimees du robot.
 * @param[in,out] angle_pid PID d angle vers vitesse angulaire.
 * @param[in] target_angle Angle absolu demander, en radians; lu au premier scan seulement.
 * @param[in] relative_rotation Si true, target_angle est un deplacement signe depuis le premier scan, sans chemin court.
 * @param[out] finished True lorsque le profil est fini et le robot stabiliser assez longtemps.
 * @return True si le scan est valide; false avec fault et commandes nulles sinon.
 * @note Appeler a chaque periode. La commande lineaire reste nulle.
 * @note reset_motion() prepare une nouvelle cible; un appel suivant poursuit le profil courant.
 */
bool compute_rotation(motion_controller &motion, const odometry &robot, PID_controller &angle_pid, float target_angle, bool &finished, bool relative_rotation = false);

/**
 * @brief Executer un scan de translation dans l axe du robot au depart.
 * @param[in,out] motion Profil lineaire, tolerances et memoire du suivi.
 * @param[in] robot Position et vitesses estimees du robot.
 * @param[in,out] position_pid PID de position vers vitesse lineaire.
 * @param[in] displacement Distance relative signee en metres; lue au premier scan seulement.
 * @param[in] relative_rotation Si true, target_angle est un deplacement signe depuis le premier scan, sans chemin court.
 * @param[out] finished True lorsque le profil est fini et le robot stabiliser assez longtemps.
 * @return True si le scan est valide; false avec fault et commandes nulles sinon.
 * @param[in,out] angle_pid PID de suivi du cap corrige pendant la translation.
 * @note Le robot doit deja etre oriente dans l axe voulu. Cap corrige selon l ecart lateral a la ligne de depart, en avance et en recul.
 * @note Appeler a chaque periode; reset_motion() prepare un nouveau deplacement.
 */
bool compute_translation(motion_controller &motion, const odometry &robot, PID_controller &position_pid, PID_controller &angle_pid, float displacement, bool &finished);

#endif
