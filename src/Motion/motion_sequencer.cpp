/**
 * @file motion_sequencer.cpp
 * @brief Convertir une cible absolue en rotation, translation, puis rotation finale.
 *
 * Aucun calcul de profil ou de PID ici : motion.cpp execute les mouvements.
 */
#include "Motion/motion_sequencer.h"
#include <math.h>

static constexpr float PI_VALUE = 3.14159265358979323846f;

/** @brief Memoriser le defaut et annuler les commandes; retourne toujours false. */
static bool sequence_failure(motion_sequencer &motion) {
    motion.movement.fault = true;
    if (motion.movement.fault_reason == MotionFault::NONE) { motion.movement.fault_reason = MotionFault::INVALID_STATE; }
    motion.movement.linear_speed_cmd = 0.0f;
    motion.movement.angular_speed_cmd = 0.0f;
    return false;
}

/** @brief Changer d etape et preparer le suivi du prochain mouvement. */
static void change_motion_state(motion_sequencer &motion, MotionState next_state, PID_controller &position_pid, PID_controller &angle_pid) {
    motion.state = next_state;
    reset_motion(motion.movement, position_pid, angle_pid);
}

/** @copydoc motion_config */
bool motion_config(motion_sequencer &motion, float period) {
    motion.state = MotionState::FINISHED;
    return configure_motion(motion.movement, period);
}

/** @copydoc motion_start */
bool motion_start(motion_sequencer &motion, float target_x, float target_y, float target_angle, int translation_direction) {
    if (!motion.movement.configured || motion.movement.fault) {
        return sequence_failure(motion); // Sequenceur indisponible
    }

    if (motion.state != MotionState::FINISHED) {
        return false; // Conserver le mouvement en cours
    }

    if (!isfinite(target_x) || !isfinite(target_y) || !isfinite(target_angle) || ((translation_direction != 1) && (translation_direction != -1))) {
        return sequence_failure(motion); // Commande invalide
    }

    motion.movement.lateral_target = 0.0f;
    motion.signed_rotation = false; // Cible absolue : chemin court.
    motion.rotation_displacement = 0.0f;
    motion.target_x = target_x; // Cible X en metres
    motion.target_y = target_y; // Cible Y en metres
    motion.target_angle = target_angle; // Orientation finale en radians
    motion.translation_direction = translation_direction; // Avance ou recul

    motion.movement.linear_speed_cmd = 0.0f; // Commande initiale nulle
    motion.movement.angular_speed_cmd = 0.0f;
    motion.movement.stable_time = 0.0f; // Nouvelle temporisation
    motion.movement.elapsed = 0.0f;
    motion.movement.profile_initialized = false; // Charger le profil au prochain scan
    motion.state = MotionState::INITIAL_ROTATION; // Premiere etape

    return true;
}

/** @copydoc motion_start_relative */
bool motion_start_relative(motion_sequencer &motion, const odometry &robot, float distance, float delta_angle) {
    if (motion.state != MotionState::FINISHED) { return false; }
    if (!robot.configured || robot.fault || !isfinite(robot.x) || !isfinite(robot.y) || !isfinite(robot.theta)) {
        return sequence_failure(motion);
    }
    if (!motion_start(motion, robot.x + distance * cosf(robot.theta),
                      robot.y + distance * sinf(robot.theta), robot.theta + delta_angle,
                      distance < 0.0f ? -1 : 1)) {
        return false;
    }
    const float ratio = distance < 0.0f ? motion.movement.reverse_lateral_correction_m_per_m
                                       : motion.movement.forward_lateral_correction_m_per_m;
    motion.movement.lateral_target = fabsf(distance) * ratio;
    motion.target_x -= motion.movement.lateral_target * sinf(robot.theta);
    motion.target_y += motion.movement.lateral_target * cosf(robot.theta);
    // Une commande relative commence deja dans son axe; ne pas viser la diagonale.
    motion.state = MotionState::TRANSLATION;
    if (distance == 0.0f) {
        motion.signed_rotation = true;
        motion.rotation_displacement = delta_angle;
        motion.state = MotionState::FINAL_ROTATION; // Aucune translation.
    }
    return true;
}

/** @copydoc compute_motion */
bool compute_motion(motion_sequencer &motion, const odometry &robot, PID_controller &position_pid, PID_controller &angle_pid) {
    motion_controller &movement = motion.movement; // Reference, aucune copie
    movement.linear_speed_cmd = 0.0f;
    movement.angular_speed_cmd = 0.0f;

    if (!movement.configured || movement.fault || !robot.configured || robot.fault || !position_pid.configured || position_pid.fault || !angle_pid.configured || angle_pid.fault) {
        return sequence_failure(motion);
    }
    if (!isfinite(robot.x) || !isfinite(robot.y) || !isfinite(robot.theta) || !isfinite(robot.linear_speed) || !isfinite(robot.angular_speed)) {
        return sequence_failure(motion);
    }

    bool finished = false; // Fin physique du mouvement courant
    switch (motion.state) {
        case MotionState::INITIAL_ROTATION: {
            float target_angle = 0.0f; // Lu seulement au chargement du profil
            if (!movement.profile_initialized) {
                const float error_x = motion.target_x - robot.x;
                const float error_y = motion.target_y - robot.y;
                const float distance = hypotf(error_x, error_y);
                if (!isfinite(error_x) || !isfinite(error_y) || !isfinite(distance)) {
                    return sequence_failure(motion);
                }
                if (distance <= movement.position_tolerance) {
                    change_motion_state(motion, MotionState::FINAL_ROTATION, position_pid, angle_pid);
                    return true; // Aucune translation necessaire
                }
                target_angle = atan2f(error_y, error_x);
                if (motion.translation_direction == -1) { target_angle += PI_VALUE; }
            }
            if (!compute_rotation(movement, robot, angle_pid, target_angle, finished)) { return false; }
            if (finished) { change_motion_state(motion, MotionState::TRANSLATION, position_pid, angle_pid); }
            return true;
        }

        case MotionState::TRANSLATION: {
            float displacement = 0.0f; // Lu seulement au chargement du profil
            if (!movement.profile_initialized) {
                // Conserver la projection historique pour tenir compte de la pose apres rotation.
                // Ces calculs ont lieu au depart seulement, jamais pendant le suivi du profil.
                displacement = ((motion.target_x - robot.x) * cosf(robot.theta)) + ((motion.target_y - robot.y) * sinf(robot.theta));
            }
            if (!compute_translation(movement, robot, position_pid, angle_pid, displacement, finished)) { return false; }
            if (finished) {
                if (hypotf(motion.target_x - robot.x, motion.target_y - robot.y) > movement.position_tolerance) {
                    movement.fault_reason = MotionFault::POSITION_MISSED;
                    return sequence_failure(motion); // Distance projetee atteinte, mais cible XY manquee.
                }
                change_motion_state(motion, MotionState::FINAL_ROTATION, position_pid, angle_pid);
            }
            return true;
        }

        case MotionState::FINAL_ROTATION:
            if (!compute_rotation(movement, robot, angle_pid, motion.signed_rotation ? motion.rotation_displacement : motion.target_angle, finished, motion.signed_rotation)) { return false; }
            if (finished) {
                if (hypotf(motion.target_x - robot.x, motion.target_y - robot.y) > movement.position_tolerance) {
                    movement.fault_reason = MotionFault::POSITION_MISSED;
                    return sequence_failure(motion);
                }
                change_motion_state(motion, MotionState::FINISHED, position_pid, angle_pid);
            }
            return true;

        case MotionState::FINISHED:
            return true;

        default:
            return sequence_failure(motion);
    }
}
