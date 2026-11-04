#include <LibRobus.h>
#include "arm.h"
#include "scan_config.h"

constexpr unsigned long OPEN_DURING_DESCENT_MS = 100UL;
constexpr unsigned long RELEASE_WAIT_MS = 250UL;///200

namespace {
struct BumperState {
    bool raw;
    bool stable;
    unsigned long changedAt;
};

BumperState bumpers[4];

void resetBumpers() {
    for (uint8_t id = 0; id < 4; ++id) {
        const bool pressed = ROBUS_IsBumper(id);
        bumpers[id] = {pressed, pressed, millis()};
    }
}

bool bumperPressed() {
    bool triggered = false;
    const unsigned long now = millis();
    for (uint8_t id = 0; id < 4; ++id) {
        auto &bumper = bumpers[id];
        const bool pressed = ROBUS_IsBumper(id);
        if (pressed != bumper.raw) {
            bumper.raw = pressed;
            bumper.changedAt = now;
        }
        if (bumper.raw != bumper.stable && now - bumper.changedAt >= BUMPER_DEBOUNCE_MS) {
            bumper.stable = bumper.raw;
            if (bumper.stable) triggered = true;
        }
    }
    return triggered;
}
}

void setup() {
    armBegin();
    BoardInit();
    MOTOR_SetSpeed(LEFT, 0.0f);
    MOTOR_SetSpeed(RIGHT, 0.0f);
    Serial.begin(9600);
    flipDown();
    openThumb();
    resetBumpers();
    Serial.println(F("FLIP TEST : placer le verre puis appuyer sur un bumper ou envoyer G."));
}

void loop() {
    const bool pressed = bumperPressed();
    const int command = Serial.read();
    if (!pressed && command != 'G' && command != 'g') return;
    closeThumb();
    delay(THUMB_CLOSE_WAIT_MS);
    flipUp();
    delay(ARM_MOVE_WAIT_MS);
    flipDown();
    delay(OPEN_DURING_DESCENT_MS);
    openThumb();
    delay(RELEASE_WAIT_MS);
    flipUp();
    delay(ARM_MOVE_WAIT_MS);

    flipDown();
    delay(ARM_MOVE_WAIT_MS);
    resetBumpers();
    Serial.println(F("Pince ouverte en bas. Placer le verre puis appuyer sur un bumper ou envoyer G."));
}
