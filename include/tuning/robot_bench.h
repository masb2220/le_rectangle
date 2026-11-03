/**
 * @file robot_bench.h
 * @brief Banc de caracterisation et de tuning roues libres, independant du controle normal.
 */

#ifndef ROBOT_BENCH_H
#define ROBOT_BENCH_H

/**
 * @brief Initialiser LibRobus et le port serie du banc avec les moteurs a zero.
 */
void robot_bench_init();

/**
 * @brief Traiter STOP, executer les essais a leur echeance et envoyer les rapports hors acquisition.
 */
void robot_bench_update();
#endif
