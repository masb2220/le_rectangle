#include "declarations.h"

uint32_t receiveIR() {
  if (!irrecv.decode(&results)) {
    return 0;
  }

  const uint32_t signal = results.value;
  irrecv.resume();
  if (signal == 0xFFFFFFFFUL) {
    return 0;
  }

  Serial.println(signal, HEX);
  return signal;
}