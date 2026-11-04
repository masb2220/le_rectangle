#include <Arduino.h>
#include <string.h>
#include "robot_serial.h"
#include "robot_control.h"
#include "cup.h"

namespace {
char output[512];
uint16_t head=0, tail=0, used=0;
char input[48];
uint8_t length=0;
bool rejected=false;
bool live=false;
unsigned long lastLive=0;
void command() {
    if (!strcmp(input,"START") || !strcmp(input,"SCAN")) {
        if (!cupStart()) robot_serial_message("Depart refuse : controle occupe ou en defaut.");
    } else if (!strcmp(input,"TUNE")) {
        if (!cupStart(false)) robot_serial_message("TUNE refuse : sequence en cours ou defaut PID.");
    } else if (!strcmp(input,"STOP")) cupAbort();
    else if (!strcmp(input,"STATUS")) cupPrintStatus();
    else if (!strcmp(input,"OFFSET")) cupPrintOffset();
    else if (!strcmp(input,"LIVE")) { live=true; robot_serial_message("Affichage auto actif, une fois par seconde."); }
    else if (!strcmp(input,"PAUSE")) { live=false; robot_serial_message("Affichage auto suspendu."); }
    else if (!strcmp(input,"PING")) robot_serial_message("PONG : programme et liaison serie actifs.");
    else robot_serial_message("SCAN | TUNE | + | - | OFFSET | STOP | STATUS | LIVE | PAUSE | PING");
}
}
void robot_serial_message(const char *message) {
    const size_t n=strlen(message);
    if(n+2 > sizeof(output)-used) return;
    for(size_t i=0;i<n;++i) { output[head]=message[i]; head=(head+1)%sizeof(output); ++used; }
    output[head]='\r'; head=(head+1)%sizeof(output); ++used;
    output[head]='\n'; head=(head+1)%sizeof(output); ++used;
}
void robot_serial_update(bool) {
    // Budget borne, aucune attente sur la liaison serie.
    for(uint8_t n=0;n<16 && Serial.available();++n) {
        const char c=Serial.read();
        // + et - fonctionnent immediatement, avec ou sans fin de ligne.
        if (!length && !rejected && (c=='+' || c=='-')) {
            if (!cupAdjustOffset(c=='+' ? 1 : -1))
                robot_serial_message("Pas refuse : attendre la fin du mouvement et la detection avant +/-.");
            continue;
        }
        if(c=='\n'||c=='\r') {
            if(rejected) robot_serial_message("Ligne trop longue.");
            else if(length) { input[length]=0; command(); }
            length=0; rejected=false;
        } else if(!rejected) {
            if(length<sizeof(input)-1) input[length++]=c;
            else rejected=true;
        }
    }
    if (live && millis()-lastLive >= 1000UL && used < 100 &&
        (robot_control_fault() || robot_control_background_time_us() > 4000)) {
        lastLive=millis();
        cupPrintStatus();
    }
    robot_serial_flush();
}
void robot_serial_flush() {
    int available=Serial.availableForWrite();
    if(available>16) available=16;
    while(available-- > 0 && used) {
        Serial.write(output[tail]); tail=(tail+1)%sizeof(output); --used;
    }
}
