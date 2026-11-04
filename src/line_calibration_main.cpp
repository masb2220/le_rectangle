#include <LibRobus.h>
#include "LineFollower/line_follower.h"

namespace {
bool capturing = false;
bool captureBlack = false;
bool whiteCaptured = false;
bool blackCaptured = false;

void printValues(const __FlashStringHelper *name, const uint16_t values[3]) {
    Serial.print(F("constexpr uint16_t "));
    Serial.print(name);
    Serial.print(F("[3] = {"));
    Serial.print(values[0]); Serial.print(F(", "));
    Serial.print(values[1]); Serial.print(F(", "));
    Serial.print(values[2]); Serial.println(F("};"));
}
}

void setup() {
    BoardInit();
    lineBegin();
    Serial.begin(9600);
    Serial.println(F("Calibration ligne (A10 gauche, A11 milieu, A12 droite)."));
    Serial.println(F("Placer les trois capteurs sur du blanc puis envoyer B."));
    Serial.println(F("Placer les trois capteurs sur du noir puis envoyer N."));
}

void loop() {
    if (Serial.available()) {
        const char key = Serial.read();
        if (!capturing && (key == 'B' || key == 'b' || key == 'N' || key == 'n')) {
            captureBlack = key == 'N' || key == 'n';
            capturing = true;
            lineCalibrate(captureBlack);
            Serial.println(F("Capture en cours. Garder le robot immobile."));
        }
    }

    lineUpdate();
    if (const char *event = lineEvent()) {
        Serial.println(event);
        capturing = false;
        if (captureBlack) blackCaptured = true;
        else whiteCaptured = true;

        uint16_t white[3], black[3];
        lineReadCalibration(white, black);
        if (whiteCaptured) printValues(F("LINE_WHITE_DEFAULT"), white);
        if (blackCaptured) printValues(F("LINE_BLACK_DEFAULT"), black);

        if (whiteCaptured && blackCaptured) {
            Serial.println(lineCalibrationReady()
                ? F("Contraste OK. Copier les deux lignes dans line_follower.h.")
                : F("Contraste insuffisant. Refaire les mesures avant de copier."));
        }
    }
}
