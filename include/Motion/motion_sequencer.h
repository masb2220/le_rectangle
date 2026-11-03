/**
 * @file motion_sequencer.h
 * @brief Preparation d une cible X/Y/angle et enchainement de trois mouvements.
 */
#ifndef MOTION_SEQUENCER_H
#define MOTION_SEQUENCER_H

#include "Motion/motion.h"

/** @brief Etapes de la sequence; les deux rotations utilisent la meme fonction. */
enum class MotionState {
    INITIAL_ROTATION, ///< Orientation vers la cible.
    TRANSLATION, ///< Avance ou recul dans l axe atteint.
    FINAL_ROTATION, ///< Orientation finale demander.
    FINISHED ///< Sequence terminee; verifier aussi le defaut.
};

/** @brief Cible absolue et etape active; les calculs de suivi sont dans movement. */
struct motion_sequencer {
    motion_controller movement; ///< Execution du mouvement elementaire courant.
    float target_x = 0.0f; ///< Position cible absolue X, en metres.
    float target_y = 0.0f; ///< Position cible absolue Y, en metres.
    float target_angle = 0.0f; ///< Orientation finale absolue, en radians.
    float rotation_displacement = 0.0f; ///< Rotation seule : deplacement signe demande.
    bool signed_rotation = false; ///< Conserver le sens de la rotation relative seule.
    int translation_direction = 1; ///< +1 pour avancer, -1 pour reculer.
    MotionState state = MotionState::FINISHED; ///< Etape actuelle de la sequence.
};

/**
 * @brief Valider le controleur de mouvement et placer le sequenceur au repos.
 * @param[in,out] motion Sequenceur avec profils et tolerances deja configurer dans movement.
 * @param[in] period Periode de calcul, en secondes.
 * @return True si accepter; false avec movement.fault sinon.
 */
bool motion_config(motion_sequencer &motion, float period);

/**
 * @brief Enregistrer une cible absolue sans commencer les calculs de suivi.
 * @param[in,out] motion Sequenceur configurer et au repos.
 * @param[in] target_x Position cible X, en metres.
 * @param[in] target_y Position cible Y, en metres.
 * @param[in] target_angle Orientation finale, en radians.
 * @param[in] translation_direction Sens : +1 en avance, -1 en recul.
 * @return True si accepter; false si occuper ou invalide. Un refus pour occupation ne cree pas de defaut.
 */
bool motion_start(motion_sequencer &motion, float target_x, float target_y, float target_angle, int translation_direction);

/** @brief Avancer/reculer dans le cap actuel, puis tourner d un angle relatif.
 * Distances en metres, angles en radians (positif = antihoraire).
 * distance=0 : rotation seule, signe et amplitude conserves (+ gauche, - droite).
 * Sinon, l orientation finale est atteinte par le chemin court. delta_angle=0 : translation seule.
 * La pose et les cibles sont memorisees a l acceptation. Refus si occupe.
 */
bool motion_start_relative(motion_sequencer &motion, const odometry &robot, float distance, float delta_angle);

/**
 * @brief Executer un scan et passer a l etape suivante lorsque le mouvement est terminer.
 * @param[in,out] motion Sequence et commandes disponibles dans movement.
 * @param[in] robot Position et vitesses estimees du robot.
 * @param[in,out] position_pid PID utilise par la translation.
 * @param[in,out] angle_pid PID utilise par les deux rotations.
 * @return True si le scan est valide; false avec commandes nulles et defaut sinon.
 * @note FINISHED indique la fin de la sequence; true ne signifie pas mouvement terminer.
 */
bool compute_motion(motion_sequencer &motion, const odometry &robot, PID_controller &position_pid, PID_controller &angle_pid);

#endif
