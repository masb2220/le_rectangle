#include "declarations.h"

int receiveIR() {
  if (irrecv.decode(&results)) {
    if (results.value != 0xFFFFFFFF) {      // ignore les trames de répétition
      Serial.println(results.value, HEX);   // une seule fois par appui
    }
    irrecv.resume();                        // prêt pour le prochain signal
  }
  return results.value;
}