// Main independant : environnement PlatformIO line_test.
#include <Arduino.h>
#include <LibRobus.h>
#include <stdio.h>
#include <string.h>
#include "LineFollower/line_follower.h"
#include "LineFollower/line_steering.h"
#include "declarations.h"
namespace {
char output[384];
uint16_t head = 0, tail = 0, used = 0;
char input[24];
uint8_t length = 0;
bool rejected = false;
bool live = true;
unsigned long lastStatus = 0;
constexpr unsigned long BUMPER_DEBOUNCE_MS = 40UL;
struct BumperState {
    bool raw;
    bool stable;
    unsigned long changedMs;
};
BumperState bumpers[4] = {};

void message(const char *text)
{
    if (strlen(text) + 2 > sizeof(output) - used) return;
    while (*text) {
        output[head] = *text++;
        head = (head + 1) % sizeof(output);
        ++used;
    }
    for (char c : "\r\n") {
        if (!c) break;
        output[head] = c;
        head = (head + 1) % sizeof(output);
        ++used;
    }
}

void initBumpers()
{
    for (uint8_t id = 0; id < 4; ++id) {
        const bool pressed = ROBUS_IsBumper(id);
        // Un bouton tenu au reset doit etre relache puis presse.
        bumpers[id] = {pressed, pressed, millis()};
    }
}

void updateBumpers()
{
    const unsigned long now = millis();
    for (uint8_t id = 0; id < 4; ++id) {
        BumperState &b = bumpers[id];
        const bool pressed = ROBUS_IsBumper(id);
        if (pressed != b.raw) {
            b.raw = pressed;
            b.changedMs = now;
        }
        if (b.raw != b.stable && now - b.changedMs >= BUMPER_DEBOUNCE_MS) {
            b.stable = b.raw;
            if (b.stable && !lineFollowing()) {
                if (lineStart()) message("Bumper : suivi de ligne actif.");
                else message("Bumper refuse : calibration incomplete ou en cours.");
            }
        }
    }
}
void printStatus()
{
    uint16_t raw[3], darkness[3];
    lineReadStatus(raw, darkness);
    char text[160];
    snprintf(text, sizeof(text), "G/M/D brut=%u/%u/%u noir=%u/%u/%u calibre=%u suivi=%u mode=%s",
        raw[0], raw[1], raw[2], darkness[0], darkness[1], darkness[2],
        unsigned(lineCalibrationReady()), unsigned(lineFollowing()), lineMode());
    message(text);
}

void command()
{
    if (!strcmp(input, "STOP")) { lineStop(); message("STOP : roues arretees."); }
    else if (!strcmp(input, "BLANC")) { lineCalibrate(false); message("Calibration BLANC : garder les trois detecteurs au-dessus du blanc."); }
    else if (!strcmp(input, "NOIR")) { lineCalibrate(true); message("Calibration NOIR : garder les trois detecteurs au-dessus du noir."); }
    else if (!strcmp(input, "GO")) {
        if (lineStart()) message("Suivi de ligne actif.");
        else message("GO refuse : terminer BLANC et NOIR avec assez de contraste.");
    } else if (!strcmp(input, "STATUS")) printStatus();
    else if (!strcmp(input, "LIVE")) live = true;
    else if (!strcmp(input, "PAUSE")) live = false;
    else message("BLANC | NOIR | GO | STOP | STATUS | LIVE | PAUSE");
}

void serialUpdate()
{
    for (uint8_t n = 0; n < 16 && Serial.available(); ++n) {
        const char c = Serial.read();
        if (c == '\r' || c == '\n') {
            if (rejected) message("Ligne trop longue, ignoree.");
            else if (length) { input[length] = 0; command(); }
            length = 0;
            rejected = false;
        } else if (!rejected) {
            if (length < sizeof(input) - 1) input[length++] = c;
            else rejected = true;
        }
    }
    int available = min(Serial.availableForWrite(), 16);
    while (available-- > 0 && used) {
        Serial.write(output[tail]);
        tail = (tail + 1) % sizeof(output);
        --used;
    }
}
}

void setup()
{
    // Initialiser le robot sans PID de navigation ni sequence de ramassage.
    BoardInit();
    MOTOR_SetSpeed(LEFT, 0.0f);
    MOTOR_SetSpeed(RIGHT, 0.0f);
    SERVO_Disable(0);
    SERVO_Disable(1);
    lineBegin();
    initBumpers();
    char speedMessage[100];
    snprintf(speedMessage, sizeof(speedMessage), "Vitesse demandee=%d/1000, croisiere avec marge PID=%d/1000.",
        int(LINE_BASE_SPEED*1000), int(lineCruiseLimit(LINE_BASE_SPEED, LINE_MAX_SPEED, LINE_MAX_TURN)*1000));
    message(speedMessage);
    message("LINE TEST : analogiques A10=gauche A11=milieu A12=droite, 9600 bauds.");
    message("Calibration chargee. Placer la ligne puis appuyer sur un bumper (ou GO).");
    message("STOP arrete. Aucun depart automatique. Servos desactives.");
    irrecv.enableIRIn();   // initialise le récepteur IR

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
    while (true) {
        uint32_t signal = receiveIR();
        if (signal == 0x906FC03FUL) {
        Serial.println("start signal received");
        break;
        }
    }
}

void loop()
{
    colorReaction();
    updateBumpers();
    //lineUpdate();
    serialUpdate();
    if (const char *event = lineEvent()) message(event);
    if (live && millis() - lastStatus >= 500UL && used < 100) {
        lastStatus = millis();
        printStatus();
    }

}
