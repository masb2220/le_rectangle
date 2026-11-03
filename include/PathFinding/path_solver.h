/** @file path_solver.h
 * @brief Interface du parcours de Ben, execute sans attente bloquante.
 */
#ifndef PATH_SOLVER_H
#define PATH_SOLVER_H
#include <Print.h>
void path_solver_init();
void path_solver_update();
void path_solver_print_status(Print &output);
#endif
