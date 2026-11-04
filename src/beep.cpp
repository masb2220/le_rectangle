#include "declarations.h"

namespace {
constexpr unsigned long PULSE_MS = 100;
uint8_t remaining = 0;
bool sounding = false;
unsigned long changedAt = 0;
}

void beep(int count) {
    if (count <= 0 || count > 255 || remaining) return;
    remaining = count;
    sounding = true;
    changedAt = millis();
    AX_BuzzerON();
}

bool updateBeep() {
    if (!remaining) return false;
    const unsigned long now = millis();
    if (now - changedAt < PULSE_MS) return true;
    changedAt = now;
    if (sounding) {
        AX_BuzzerOFF();
        sounding = false;
        --remaining;
    } else {
        AX_BuzzerON();
        sounding = true;
    }
    return remaining != 0;
}
