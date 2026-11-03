#include "distance_ir.h"
#include <math.h>

namespace {
DistanceIR mesures[2] = {};
uint8_t cote = 0;
unsigned long derniereLecture = 0;
constexpr unsigned long INTERVALLE_ALTERNE_MS = 25; // 50 ms par capteur.
// Formules fournies et validees par l'utilisateur sur son montage.
float convertir(uint8_t index, uint16_t brut)
{
    float cm = NAN;
    if (index == 0 && brut > 0) {
        cm = 30.0f * powf(brut * 0.0048828125f, -1.173f);
    } else if (index == 1 && brut > 3) { // Gauche A0 : GP2D12F, formule empirique fournie.
        cm = 6787.0f / (brut - 3.0f) - 4.0f;
    }
    // Meme plage utile que le programme d'origine, sans arrondir a l'entier.
    return isfinite(cm) && cm > 10.0f && cm < 80.0f ? cm : NAN;
}
uint16_t lireADC(uint8_t pin)
{
    analogRead(pin); // Laisser le multiplexeur se stabiliser apres changement de voie.
    uint16_t a = analogRead(pin), b = analogRead(pin), c = analogRead(pin);
    if (a > b) { const uint16_t t=a; a=b; b=t; }
    if (b > c) { const uint16_t t=b; b=c; c=t; }
    if (a > b) { const uint16_t t=a; a=b; b=t; }
    // Mediane de lectures ADC d'une meme sortie ; pas trois mesures IR independantes.
    return b;
}
}
void initDistance()
{
    pinMode(PIN_DISTANCE_DROIT, INPUT);
    pinMode(PIN_DISTANCE_GAUCHE, INPUT);
}
void updateDistance()
{
    if (millis() - derniereLecture < INTERVALLE_ALTERNE_MS) return;
    derniereLecture = millis();
    const uint16_t brut = lireADC(cote == 0 ? PIN_DISTANCE_DROIT : PIN_DISTANCE_GAUCHE);
    const float cm = convertir(cote, brut);
    mesures[cote] = {cm, isfinite(cm), millis(), brut};
    cote ^= 1;
}
DistanceIR lireDistance(uint8_t index)
{
    if (index > 1) return {NAN, false, 0, 0};
    DistanceIR resultat = mesures[index];
    if (!resultat.dateMs || millis() - resultat.dateMs > 200) resultat.valide = false;
    return resultat;
}
