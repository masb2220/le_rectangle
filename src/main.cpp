#include "declarations.h"

void setup(void) {
  Serial.begin(9600);
  pinMode(gauche, INPUT);
  pinMode(milieu, INPUT);
  pinMode(droite, INPUT);
  //BoardInit();
  pinMode(45, OUTPUT);
  pinMode(43, OUTPUT);
  pinMode(41, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(0, INPUT);
  pinMode(1, INPUT);
  pinMode(2, INPUT);

  irrecv.enableIRIn();   // initialise le récepteur IR

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  digitalWrite(41, HIGH);
  digitalWrite(42, HIGH);
  digitalWrite(43, HIGH);
  digitalWrite(44, HIGH);
  digitalWrite(45, HIGH);
}

void loop(void) {
  receiveIR();
}