#include "declarations.h"

int position_robot() {
  int intensity_gauche = analogRead(gauche);
  int intensity_milieu = analogRead(milieu);
  int intensity_droite = analogRead(droite);

  if (intensity_milieu >= threshold_milieu) {
    return 0;
  } else if (intensity_droite >= threshold_droite) {
    return 1;
  } else if (intensity_gauche >= threshold_gauche) {
    return 2;
  } else {
    return 3;
  }
}