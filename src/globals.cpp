#include "declarations.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
State state = reading;

//LES ANCIENNES VALEURS POUR LE SUIVEUR DE LIGNE A OLI
int threshold = 600;
int threshold_gauche = 40;
int threshold_milieu = 80;
int threshold_droite = 30;

int gauche = 10;
int milieu = 11;
int droite = 12;

float vitesse = 0.2;
float vitesse_gauche = vitesse;
float vitesse_droite = vitesse;
float correction = 0.01;
int last_position = 0;

IRrecv irrecv(37);
decode_results results;