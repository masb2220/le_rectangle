#include "declarations.h"
#include <Arduino.h>
#include <LibRobus.h>
#include "scan_config.h"
#include "robot_control.h"
#include "robot_serial.h"
#include "distance_ir.h"
#include "cup.h"
#include "arm.h"

namespace {
struct BumperState {
    bool raw;
    bool stable;
    unsigned long changedMs;
};
BumperState bumpers[4] = {};
void initBumpers() {
    for (uint8_t id = 0; id < 4; ++id) {
        const bool pressed = ROBUS_IsBumper(id);
        // Un bumper deja tenu au demarrage doit etre relache puis presse.
        bumpers[id] = {pressed, pressed, millis()};
    }
}
void updateBumpers() {
    const unsigned long now = millis();
    for (uint8_t id = 0; id < 4; ++id) {
        BumperState &b = bumpers[id];
        const bool pressed = ROBUS_IsBumper(id);
        if (pressed != b.raw) {
            b.raw = pressed;
            b.changedMs = now;
        }
        if (b.stable != b.raw && now - b.changedMs >= BUMPER_DEBOUNCE_MS) {
            b.stable = b.raw;
            if (b.stable) {
                if (cupStart()) robot_serial_message("Bumper : prise puis retour distance et orientation initiale.");
                else robot_serial_message("Bumper ignore : sequence en cours ou controle indisponible.");
            }
        }
    }
}
}

bool ready = false;
bool faultReported = false;
void setup() {
    armBegin(); // Signaux servo a LOW avant toute initialisation du robot.
    Serial.begin(9600);
    ready = robot_control_init();
    cupBegin();
    flipDown(); // Position basse commandee des le demarrage, sans attendre le bumper.
    initBumpers();
    robot_serial_message("Demarrage : flipDown commande, pince en position basse.");
    robot_serial_message("BUMPER PRISE : scan A0, offset, prise, retour et orientation initiale.");
    robot_serial_message("Appuyer puis relacher un bumper pour lancer. STOP arrete. SCAN reste disponible.");
    pinMode(45, OUTPUT);
    pinMode(43, OUTPUT);
    pinMode(41, OUTPUT);
    pinMode(42, OUTPUT);
    pinMode(44, OUTPUT);
    
  irrecv.enableIRIn();   // initialise le récepteur IR

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    //while (1);
  }
  digitalWrite(41, HIGH);
  digitalWrite(42, HIGH);
  digitalWrite(43, HIGH);
  digitalWrite(44, HIGH);
  digitalWrite(45, HIGH);
}
void loop() {
    if (ready) robot_control_update();
    robot_serial_update(false); // Toujours disponible, meme en defaut PID.
    const bool fault = !ready || robot_control_fault();
    if (fault) {
        // Roues verrouillees par le controle : l'acquisition IR reste disponible.
        updateDistance();
        cupUpdate(); // Signaler le defaut et sortir de l'etat de balayage.
        if (!faultReported) {
            faultReported = true;
            robot_serial_message("Controle indisponible : roues arretees. IR et LIVE restent disponibles.");
            robot_serial_message("STATUS affiche la cause ; redemarrage necessaire pour relancer le PID.");
        }
        return;
    }
    updateBumpers();
    if (robot_control_background_time_us() > 3000) updateDistance();
    robot_control_update();
    if (robot_control_background_time_us() > 3000) cupUpdate();
}
