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

constexpr unsigned long COLOR_SAMPLE_MS = 160UL;
extern Adafruit_TCS34725 tcs;
extern State state;




extern IRrecv irrecv;

//============ Déclaration de fonctions ==========//
void colorReaction(bool diagnostics = false);
void initLeds();
int position_robot();
void suivre_ligne();
// Trames mesurees avec la telecommande et IRremote 2.x : depart et reveil.
constexpr uint32_t IR_START_SIGNAL = 0x906FC03FUL;
constexpr uint32_t IR_WAKE_SIGNAL = 0x906F08F7UL;
uint32_t receiveIR();
void beep(int count);
bool updateBeep();
void updateLeds();
bool initColorSensor();
bool colorBlueHold();
