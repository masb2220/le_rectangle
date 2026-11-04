#include "declarations.h"

void setup() {
    BoardInit();
    MOTOR_SetSpeed(LEFT, 0.0f);
    MOTOR_SetSpeed(RIGHT, 0.0f);
    Serial.begin(9600);
    if (initColorSensor()) {
        Serial.println(F("COLOR TEST : presenter une couleur devant le capteur."));
        Serial.println(F("DEL : rose D45, turquoise D42, jaune D43, orange D44, bleu D41."));
    } else {
        Serial.println(F("Capteur TCS34725 absent : verifier le branchement I2C puis redemarrer."));
    }
}

void loop() {
    updateLeds();
    colorReaction(true);
}
