#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <LibRobus.h>   // already brings in IRremote (2.x)

//============ Types ==========//
enum State {
  reading,
  pink,
  blue,
  cyan,
  orange,
  yellow
};

//============ Variables globales (définies dans globals.cpp) ==========//
extern Adafruit_TCS34725 tcs;
extern State state;

extern int threshold;
extern int threshold_gauche;
extern int threshold_milieu;
extern int threshold_droite;

extern int gauche;
extern int milieu;
extern int droite;

extern float vitesse;
extern float vitesse_gauche;
extern float vitesse_droite;
extern float correction;
extern int last_position;

extern IRrecv irrecv;
extern decode_results results;

//============ Déclaration de fonctions ==========//
void colorReaction();
int position_robot();
void suivre_ligne();
uint32_t receiveIR();
void beep(int count);