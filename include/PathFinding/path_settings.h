/** @file path_settings.h
 * @brief Grille connue du parcours. Les lignes interdites sont distinctes des murs IR.
 */
#ifndef PATH_SETTINGS_H
#define PATH_SETTINGS_H
#include <stdint.h>
namespace path_settings {
constexpr bool INTERIOR_LINES_CONFIRMED = true; ///< Confirmer apres saisie des lignes du parcours reel.
constexpr int ROWS = 10;
constexpr int COLUMNS = 3;
constexpr int START_X = 1; ///1
constexpr int START_Y = 9; ///9
constexpr float CELL_M = 0.5f;
constexpr int FORWARD_CELLS = 2; // Maximum de cases par avance vers le nord.
static_assert(FORWARD_CELLS >= 1 && FORWARD_CELLS < ROWS, "FORWARD_CELLS doit etre entre 1 et ROWS-1");
// Une entree par case, rangees 0..9. Bits N=1 E=2 S=4 W=8.
// Renseigner les lignes interieures connues ici; zero signifie aucune ligne renseignee.
// Une ligne peut etre indiquee d un seul cote : la lecture verifie les deux cases.
static const uint8_t FORBIDDEN[ROWS][COLUMNS] = {
    {2,14,8}, // Excel 1 : A-B et B-C interdites.
    {0,5,0},  // Excel 2.
    {2,15,8}, // Excel 3.
    {0,1,0},  // Excel 4.
    {2,10,8}, // Excel 5.
    {0,4,0},  // Excel 6 : passages lateraux libres.
    {2,15,8}, // Excel 7.
    {0,5,0},  // Excel 8.
    {2,15,8}, // Excel 9.
    {0,1,0}   // Excel 10 : depart B10.
};
}
#endif
