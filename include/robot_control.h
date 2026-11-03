/**
 * @file robot_control.h
 * @brief Configuration, acquisition et execution de la chaine de controle.
 */

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <Print.h>
#include "Motion/motion.h"

/**
 * @brief Point d entrer appeler une fois depuis setup pour preparer tout le controle.
 *
 * Configure les module et initialise LibRobus seulement si HARDWARE_CONNECTED vaut true.
 * Demarre la reference de temps et retourne true si l initialisation est accepter.
 * Retour false avec defaut si la config echoue. Ne lance pas de mouvement vers une cible.
 *
 * @return false avec defaut si la config echoue. Ne lance pas de mouvement vers une cible.
 */
bool robot_control_init();

/**
 * @brief Point d entrer appeler souvent depuis loop, sans delay.
 *
 * Retourne directement si pas pret, en defaut ou materiel desactiver.
 * Compare micros a la derniere echeance : attend 10 ms puis execute un cycle.
 * Si le retard atteint deux periode, memorise un defaut et demande l arret.
 * Avance l echeance d une periode fixe : ce code ne mesure pas le dt reel pour les calcul.
 * Ce n est pas une interruption; la frequence des appels depend aussi du reste de loop.
 */
void robot_control_update();

/**
 * @brief Demander un mouvement vers une cible absolue si le controle est disponible.
 *
 * @param[in] target_x Position cible absolue X, en metres.
 * @param[in] target_y Position cible absolue Y, en metres.
 * @param[in] target_angle Orientation cible, en radians.
 * @param[in] direction Sens de translation : +1 ou -1.
 *
 * @return True si accepter, false si indisponible, occuper ou invalide.
 */
bool robot_control_start(float target_x, float target_y, float target_angle, int direction);

/** @brief Translation relative signee (m), suivie d une rotation relative (rad).
 * Distance nulle : rotation signee (+ gauche, - droite), sans chemin court.
 * Un seul mouvement en cours. Retour false = demande non acceptee.
 */
bool robot_control_start_relative(float distance, float delta_angle);

/** Translation signee a cap fixe, sans correction ni cible laterale.
 * distance en metres, vitesse maximale en m/s. PID roues et cap conserves.
 * Les reglages normaux sont restaures a la fin, sur STOP ou en cas de defaut.
 */
bool robot_control_start_straight(float distance, float speed_limit);

/** @brief Rotation seule avec vitesse de profil temporairement reduite (rad/s).
 * Refuse si occupe; la vitesse normale est restauree a la fin ou en cas de defaut.
 */
bool robot_control_start_rotation(float delta_angle, float speed_limit);

/** @brief Microsecondes disponibles avant la prochaine echeance du controle. */
unsigned long robot_control_background_time_us();

/** @brief Copier la pose estimee. False si controle non initialise ou en defaut. */
bool robot_control_get_pose(float &x, float &y, float &theta);

/** @brief Cause du defaut de mouvement; NONE si le defaut global vient d ailleurs. */
MotionFault robot_control_motion_fault();

/** @brief Premiere cause globale memorisee, conservee meme apres STOP. */
enum class ControlFault { NONE, CONFIGURATION, ACQUISITION, CALCULATION, MOTOR_COMMAND, MISSED_PERIOD, STOP_REQUESTED };
ControlFault robot_control_fault_reason();
/** @brief Intervalle observe lors d une periode manquee, en microsecondes. */
unsigned long robot_control_fault_interval_us();

/**
 * @brief Annuler le mouvement et verrouiller le controle avec des commandes moteur nulles.
 */
void robot_control_stop();

/**
 * @brief Lire si le sequenceur est dans son etat termine.
 *
 * @return True si initialise, sans defaut et au repos (aussi avant la premiere demande).
 */
bool robot_control_finished();

/**
 * @brief Lire le defaut global du controle.
 *
 * @return True si le controle est verrouiller en defaut.
 */
bool robot_control_fault();

/**
 * @brief Afficher la position estimee et l etat du controle au repos.
 *
 * @param[in,out] output Flux recevant le statut; utiliser au repos pour eviter le formatage pendant le controle.
 */
void robot_control_print_status(Print &output);


// Annuler la trajectoire et couper les roues sans verrouiller le controle.
// Attendre l'immobilisation mecanique avant de lancer la trajectoire suivante.
void robot_control_cancel();

#endif
