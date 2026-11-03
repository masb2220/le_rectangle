/**
 * @file motion.cpp
 * @brief Suivi des profils de rotation et de translation avec correction PID.
 */
#include "Motion/motion.h"
#include <math.h>

static constexpr float PI_VALUE = 3.14159265358979323846f;

/** @brief Reduire un angle sur un tour, sans modulo si la valeur tient deja dans la plage. */
static float wrap_turn(float angle) {
    const float full_turn = 2.0f * PI_VALUE; // Un tour complet
    if ((angle > -full_turn) && (angle < full_turn)) {
        return angle; // Meme resultat que fmodf dans cette plage
    }
    return fmodf(angle, full_turn); // Garder les grands angles et la gestion des valeurs invalides
}

/** @brief Calculer l erreur angulaire courte entre -PI et +PI. */
static float angle_error(float target, float measured) {
    const float full_turn = 2.0f * PI_VALUE; // Un tour en radians

    const float target_wrapped =
        wrap_turn(target); // Reduire la cible

    const float measured_wrapped =
        wrap_turn(measured); // Reduire la mesure

    float error =
        wrap_turn(target_wrapped - measured_wrapped); // Reduire l'ecart

    if (error > PI_VALUE) {
        error -= full_turn; // Chemin court
    }
    else if (error < -PI_VALUE) {
        error += full_turn; // Chemin court
    }

    return error; // Entre -PI et +PI
}

/** @brief Memoriser le defaut et annuler les deux commandes; retourne toujours false. */
static bool motion_failure(motion_controller &motion) {
    motion.fault = true; // Memoriser le defaut
    if (motion.fault_reason == MotionFault::NONE) { motion.fault_reason = MotionFault::INVALID_STATE; }
    motion.linear_speed_cmd = 0.0f; // Annuler la translation
    motion.angular_speed_cmd = 0.0f; // Annuler la rotation

    return false; // Signaler l'echec
}

/** @brief Accumuler le temps stable et indiquer si la duree requise est atteinte. */
static bool update_stability(motion_controller &motion, bool stable) {
    if (!stable) {
        motion.stable_time = 0.0f; // Recommencer la temporisation
        return false;
    }

    motion.stable_time += motion.period; // Ajouter une periode stable

    return motion.stable_time >= motion.required_stable_time; // Duree atteinte
}

/** @brief Limiter le temps d execution, y compris si le robot reste bloque. */
static bool check_deadline(motion_controller &motion, const speed_profile &profile) {
    motion.elapsed += motion.period;
    const float deadline = 2.0f * profile.move_time + motion.timeout_margin;
    if (!isfinite(motion.elapsed) || !isfinite(deadline) || motion.elapsed > deadline) {
        motion.fault_reason = MotionFault::TIMEOUT;
        return false;
    }
    return true;
}

/** @brief Effacer l integrale et l historique du PID sans changer ses gains ni son defaut. */
static void reset_tracking(PID_controller &controller) {
    controller.integrale = 0.0f; // Ancienne accumulation effacer
    controller.last_error = 0.0f; // Nouvelle erreur de depart
    controller.init = false; // Pas de deriver au premier scan
}

/** @copydoc configure_motion */
bool configure_motion(motion_controller &motion, float period) {
    motion.configured = false; // Bloquer avant validation
    motion.linear_speed_cmd = 0.0f; // Annuler les commandes
    motion.angular_speed_cmd = 0.0f;
    motion.stable_time = 0.0f; // Reinitialiser la temporisation
    motion.elapsed = 0.0f;

    if (!isfinite(period) || (period <= 0.0f)) {
        return motion_failure(motion); // Periode invalide
    }

    if (!isfinite(motion.position_tolerance) || (motion.position_tolerance <= 0.0f) || !isfinite(motion.angle_tolerance) || (motion.angle_tolerance <= 0.0f)) {
        return motion_failure(motion); // Tolerances de position invalides
    }

    if (!isfinite(motion.linear_speed_tolerance) || (motion.linear_speed_tolerance <= 0.0f) || !isfinite(motion.angular_speed_tolerance) || (motion.angular_speed_tolerance <= 0.0f)) {
        return motion_failure(motion); // Tolerances de vitesse invalides
    }

    if (!isfinite(motion.required_stable_time) || (motion.required_stable_time <= 0.0f) || !isfinite(motion.timeout_margin) || motion.timeout_margin <= motion.required_stable_time) {
        return motion_failure(motion); // Duree stable invalide
    }

    if (!isfinite(motion.lateral_gain) || motion.lateral_gain < 0.0f || !isfinite(motion.lateral_max_angle) || motion.lateral_max_angle <= 0.0f || motion.lateral_max_angle >= PI_VALUE * 0.5f || !isfinite(motion.lateral_fade_distance) || motion.lateral_fade_distance <= 0.0f) {
        return motion_failure(motion); // Correction laterale invalide
    }

    if (!isfinite(motion.forward_lateral_correction_m_per_m) || !isfinite(motion.reverse_lateral_correction_m_per_m)
        || fabsf(motion.forward_lateral_correction_m_per_m) > 0.1f || fabsf(motion.reverse_lateral_correction_m_per_m) > 0.1f)
        return motion_failure(motion);

    // Les limites des profils sont configurer dans robot_control avant le sequenceur.
    if (!motion.linear_profile.configured || motion.linear_profile.fault || !motion.angular_profile.configured || motion.angular_profile.fault || motion.linear_profile.period != period || motion.angular_profile.period != period) {
        return motion_failure(motion); // Profils indisponible ou periode differente
    }
    motion.profile_initialized = false; // Aucun profil en cours
    motion.period = period; // Periode en secondes
    motion.fault = false; // Effacer le defaut
    motion.fault_reason = MotionFault::NONE;
    motion.configured = true; // Configuration acceptee

    return true;
}

/** @copydoc reset_motion */
void reset_motion(motion_controller &motion, PID_controller &position_pid, PID_controller &angle_pid) {
    motion.elapsed = 0.0f;
    motion.profile_initialized = false; // Charger le prochain profil au premier scan
    motion.stable_time = 0.0f; // Recommencer la stabilisation
    motion.linear_speed_cmd = 0.0f; // Annuler la translation
    motion.angular_speed_cmd = 0.0f; // Annuler la rotation
    reset_tracking(position_pid); // Conserver les reglages
    reset_tracking(angle_pid); // Meme remise a zero pour les deux PID
}

/** @copydoc compute_rotation */
bool compute_rotation(motion_controller &motion, const odometry &robot, PID_controller &angle_pid, float target_angle, bool &finished, bool relative_rotation) {
    finished = false; // Etape en cours
    motion.linear_speed_cmd = 0.0f; // Rotation seule
    motion.angular_speed_cmd = 0.0f; // Commande nulle avant validation
    if (!motion.configured || motion.fault || !robot.configured || robot.fault || !isfinite(robot.theta) || !isfinite(robot.angular_speed) || !isfinite(robot.linear_speed)) {
        return motion_failure(motion); // Etat ou mesure invalide
    }
    if (!motion.profile_initialized) {
        motion.start_angle = robot.theta; // Reference fixe de depart
        const float displacement = relative_rotation ? target_angle : angle_error(target_angle, robot.theta);
        // Ne pas temporiser une rotation inutile. Une rotation deja commencee garde sa stabilisation.
        if (fabsf(displacement) <= motion.angle_tolerance &&
            fabsf(robot.angular_speed) <= motion.angular_speed_tolerance &&
            fabsf(robot.linear_speed) <= motion.linear_speed_tolerance) {
            finished = true;
            return true;
        }
        if (!profile_start(motion.angular_profile, displacement)) {
            return motion_failure(motion);
        }
        reset_tracking(angle_pid); // Nouveau suivi
        motion.profile_initialized = true; // Ne pas redemarrer a chaque scan
    }
    if (!check_deadline(motion, motion.angular_profile) || !compute_profile(motion.angular_profile)) {
        return motion_failure(motion);
    }
    const float reference_angle = motion.start_angle + motion.angular_profile.position; // Consigne intermediaire
    const float tracking_error = angle_error(reference_angle, robot.theta); // Erreur courte sur le profil
    const float final_error = motion.angular_profile.finished ? tracking_error : angle_error(motion.start_angle + motion.angular_profile.deplacement, robot.theta); // Profil fini : meme cible, meme erreur
    const float command = compute_PID(angle_pid, tracking_error, 0.0f, motion.angular_profile.vitesse); // Feedforward + PID
    if (angle_pid.fault || !isfinite(final_error)) {
        return motion_failure(motion);
    }
    const bool stable = motion.angular_profile.finished && fabsf(final_error) <= motion.angle_tolerance && fabsf(robot.angular_speed) <= motion.angular_speed_tolerance && fabsf(robot.linear_speed) <= motion.linear_speed_tolerance; // Profil fini ET robot stable
    motion.angular_speed_cmd = stable ? 0.0f : command; // Rotation seulement
    finished = update_stability(motion, stable); // Attendre le temps stable
    return true;
}

/** @copydoc compute_translation */
bool compute_translation(motion_controller &motion, const odometry &robot, PID_controller &position_pid, PID_controller &angle_pid, float displacement, bool &finished) {
    finished = false; // Etape en cours
    motion.linear_speed_cmd = 0.0f; // Commande nulle avant validation
    motion.angular_speed_cmd = 0.0f; // Commande angulaire nulle avant validation
    if (!motion.configured || motion.fault || !robot.configured || robot.fault || !isfinite(robot.x) || !isfinite(robot.y) || !isfinite(robot.theta) || !isfinite(robot.linear_speed) || !isfinite(robot.angular_speed)) {
        return motion_failure(motion); // Etat ou mesure invalide
    }
    if (!motion.profile_initialized) {
        motion.start_x = robot.x; // Origine de la translation
        motion.start_y = robot.y; // Origine de la translation
        motion.start_angle = robot.theta; // Axe fixe pendant tout le profil
        motion.start_cos = cosf(motion.start_angle); // Calculer une seule fois pour cette translation
        motion.start_sin = sinf(motion.start_angle); // Recalculer quand une nouvelle translation commence
        if (!profile_start(motion.linear_profile, displacement)) {
            return motion_failure(motion);
        }
        reset_tracking(position_pid); // Nouveau suivi
        reset_tracking(angle_pid);
        motion.profile_initialized = true; // Depart deja memoriser
    }
    if (!check_deadline(motion, motion.linear_profile) || !compute_profile(motion.linear_profile)) {
        return motion_failure(motion);
    }
    const float measured_position = ((robot.x - motion.start_x) * motion.start_cos) + ((robot.y - motion.start_y) * motion.start_sin); // Deplacement mesurer dans l axe
    const float tracking_error = motion.linear_profile.position - measured_position; // Erreur sur la consigne intermediaire
    const float final_error = motion.linear_profile.deplacement - measured_position; // Distance restante
    const float command = compute_PID(position_pid, tracking_error, 0.0f, motion.linear_profile.vitesse); // Feedforward + PID
    // Projection sur la normale gauche de la ligne memorisee; aucune trigonometrie supplementaire.
    const float lateral_offset = -(robot.x - motion.start_x) * motion.start_sin + (robot.y - motion.start_y) * motion.start_cos;
    const float length = motion.linear_profile.deplacement;
    float u = fabsf(length) > 1e-6f ? measured_position / length : 1.0f;
    u = fmaxf(0.0f, fminf(1.0f, u));
    const float shape = u*u*u*(10.0f + u*(-15.0f + 6.0f*u));
    const float slope = fabsf(length) > 1e-6f
        ? motion.lateral_target / length * 30.0f*u*u*(1.0f-u)*(1.0f-u) : 0.0f;
    const float lateral_error = motion.lateral_target * shape - lateral_offset;
    float heading_offset = motion.lateral_gain * lateral_error;
    if (command < 0.0f) heading_offset = -heading_offset;
    else if (command == 0.0f) heading_offset = 0.0f;
    heading_offset = fmaxf(-motion.lateral_max_angle, fminf(motion.lateral_max_angle, heading_offset));
    const float remaining = fabsf(final_error);
    if (remaining < motion.lateral_fade_distance)
        heading_offset *= remaining / motion.lateral_fade_distance;
    const float heading_error = angle_error(motion.start_angle + atanf(slope) + heading_offset, robot.theta);
    const float heading_difference = angle_error(robot.theta, motion.start_angle);
    const float projection = cosf(heading_difference);
    if (!isfinite(lateral_error) || !isfinite(slope) || projection < 0.5f)
        return motion_failure(motion);
    const float curvature_rate = fabsf(length) > 1e-6f
        ? motion.lateral_target / (length*length) * 60.0f*u*(1.0f-u)*(1.0f-2.0f*u)
          * command / (1.0f+slope*slope) : 0.0f;
    const float angular_command = compute_PID(angle_pid, heading_error, 0.0f, curvature_rate);
    if (position_pid.fault || angle_pid.fault || !isfinite(final_error)) {
        return motion_failure(motion);
    }
    const bool stable = motion.linear_profile.finished && fabsf(final_error) <= motion.position_tolerance && fabsf(robot.linear_speed) <= motion.linear_speed_tolerance && fabsf(heading_error) <= motion.angle_tolerance && fabsf(robot.angular_speed) <= motion.angular_speed_tolerance;
    motion.linear_speed_cmd = stable ? 0.0f : command / projection; // Avance avec correction de direction
    motion.angular_speed_cmd = stable ? 0.0f : angular_command;
    finished = update_stability(motion, stable); // Attendre le temps stable
    return true;
}
