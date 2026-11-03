/**
 * @file robot_serial.h
 * @brief Reception des commandes et envoi serie borne sans attente.
 */

#ifndef ROBOT_SERIAL_H
#define ROBOT_SERIAL_H

/**
 * @brief Lire les commandes, surveiller le mouvement et transmettre sans attente.
 * @param[in] allow_move Autoriser MOVE; false pendant une sequence geree par main.
 */
void robot_serial_update(bool allow_move = true);

/**
 * @brief Ajouter un message complet a la file serie si la place le permet.
 *
 * @param[in] message Message termine par un caractere nul.
 */
void robot_serial_message(const char *message);

#endif
