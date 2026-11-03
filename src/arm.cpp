#include "arm.h"
#include <LibRobus.h>

namespace {
uint8_t position[2] = {90, 90};
bool active[2] = {false, false};
uint16_t remaining = 0;
uint8_t origin = 90;
bool offsetPhase = false;
unsigned long lastStep = 0;
unsigned long stepInterval = 150;

void moveServo(uint8_t id, int target)
{
    const int low = id == FLIP_SERVO ? FLIP_MIN : THUMB_MIN;
    const int high = id == FLIP_SERVO ? FLIP_MAX : THUMB_MAX;
    position[id] = constrain(target, low, high);
    SERVO_SetAngle(id, position[id]);
    if (!active[id]) {
        SERVO_Enable(id);
        active[id] = true;
    }
}
}

void armBegin()
{
    armDisable();
}

void armDisable()
{
    remaining = 0;
    SERVO_Disable(FLIP_SERVO);
    SERVO_Disable(THUMB_SERVO);
    // Broches de signal de la copie locale Robus.h : servo 0 = 4, servo 1 = 7.
    const uint8_t pins[2] = {4, 7};
    for (uint8_t pin : pins) {
        digitalWrite(pin, LOW);
        pinMode(pin, OUTPUT);
    }
    active[0] = active[1] = false;
}
void closeThumb() { moveServo(THUMB_SERVO, THUMB_CLOSED_ANGLE); }
void openThumb() { moveServo(THUMB_SERVO, THUMB_OPEN_ANGLE); }
void flipUp()
{
    remaining = 0;
    moveServo(FLIP_SERVO, FLIP_UP_ANGLE);
}
void flipDown()
{
    remaining = 0;
    moveServo(FLIP_SERVO, FLIP_DOWN_ANGLE);
}
void jiggle(uint16_t repetitions, unsigned long intervalMs)
{
    // Ne pas activer un servo dont la position physique est inconnue.
    if (!active[FLIP_SERVO]) {
        Serial.println(F("Commander up ou down avant jiggle."));
        return;
    }
    // Un nouvel appel repart de la consigne courante.
    origin = position[FLIP_SERVO];
    remaining = repetitions;
    offsetPhase = false;
    stepInterval = intervalMs ? intervalMs : 1;
    lastStep = millis();
}
void armUpdate()
{
    if (!remaining || millis() - lastStep < stepInterval) return;
    lastStep = millis();
    if (!offsetPhase) {
        const int offset = origin < FLIP_MAX ? 1 : -1;
        moveServo(FLIP_SERVO, int(origin) + offset);
        offsetPhase = true;
    } else {
        moveServo(FLIP_SERVO, origin);
        offsetPhase = false;
        --remaining;
    }
}
void armPrintStatus()
{
    for (uint8_t i = 0; i < 2; ++i) {
        Serial.print(F("Servo ")); Serial.print(i);
        Serial.print(F(" | consigne=")); Serial.print(position[i]);
        Serial.print(F(" | actif=")); Serial.println(active[i]);
    }
    Serial.print(F("Cycles jiggle restants=")); Serial.println(remaining);
}
