#pragma once
#include <Arduino.h>

// Distances en centimetres : formules d origine de l utilisateur.
// A3 : formule puissance ; A0 : formule rationnelle. Pas de courbe typique substituee.
constexpr uint8_t PIN_DISTANCE_DROIT = A3;
constexpr uint8_t PIN_DISTANCE_GAUCHE = A0; // Sharp GP2D12F confirme par l utilisateur.
struct DistanceIR {
    float cm;
    bool valide;
    unsigned long dateMs;
    uint16_t brut;
};
void initDistance();
void updateDistance();
DistanceIR lireDistance(uint8_t cote); // 0=droit, 1=gauche
