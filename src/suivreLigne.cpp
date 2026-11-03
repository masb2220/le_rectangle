#include "declarations.h"

void suivre_ligne() {
  int position = position_robot();

  if (position == 0) {
    vitesse_droite = vitesse;
    vitesse_gauche = vitesse;
    last_position = 0;
    return;
  }
  else if (position == 1) {
    vitesse_droite = vitesse;
    vitesse_gauche = vitesse_gauche + correction;
    last_position = 1;
  }
  else if (position == 2) {
    vitesse_droite = vitesse_droite + correction;
    vitesse_gauche = vitesse;
    last_position = 2;
  }
  else if (position == 3) {
    if (last_position == 1) {
      vitesse_droite = vitesse;
      vitesse_gauche = vitesse_gauche + correction;
    }
    else if (last_position == 2) {
      vitesse_droite = vitesse_droite + correction;
      vitesse_gauche = vitesse;
    }
  }
}