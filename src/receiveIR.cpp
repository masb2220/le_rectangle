#include "declarations.h"

IRrecv irrecv(37);

uint32_t receiveIR() {
  decode_results results;
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
