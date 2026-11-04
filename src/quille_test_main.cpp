#include "declarations.h"
#include "cup.h"
#include "LineFollower/line_follower.h"
#include "distance_ir.h"
#include "robot_control.h"
#include "robot_serial.h"

namespace {
bool ready = false;
bool running = false;
bool colorReady = false;
bool markerHandled = false;
}

void setup() {
    Serial.begin(9600);
    ready = robot_control_init();
    cupBegin();
    lineBegin();
    colorReady = initColorSensor();
    irrecv.enableIRIn();
    Serial.println(ready ? F("TEST QUILLE : START IR ou G suit la ligne jusqu au rose, X arrete.") :
                           F("Initialisation du controle moteur impossible."));
}

void loop() {
    if (ready) robot_control_update(!lineFollowing());
    robot_serial_flush();
    const bool remoteStart = ready && !running && !lineFollowing() && receiveIR() == IR_START_SIGNAL;
    char command = 0;
    if (Serial.available()) {
        command = Serial.read();
    }
    if (command == 'x' || command == 'X') {
        lineStop();
        cupAbort();
        running = false;
    } else if (!running && !lineFollowing() && (remoteStart || command == 'g' || command == 'G')) {
        markerHandled = false;
        state = reading;
        if (!ready || !colorReady || robot_control_fault() || !lineStart()) {
            robot_serial_message("Depart refuse : moteur, capteur couleur ou calibration de ligne.");
        }

    }
    if (lineFollowing()) {
        if (robot_control_fault()) {
            lineStop();
            return;
        }
        if (!markerHandled && robot_control_background_time_us() > 4000) {
            colorReaction();
            if (state == pink) {
                lineStop();
                markerHandled = true;
                running = pinStart();
                if (!running) robot_serial_message("Scan quille refuse.");
                return;
            }
        }
        lineUpdate();
        if (const char *event = lineEvent()) robot_serial_message(event);
        return;
    }
    if (!running) return;
    if (!robot_control_fault()) {
        if (robot_control_background_time_us() <= 3000) return;
        updateDistance();
        if (robot_control_background_time_us() <= 3000) return;
    }
    cupUpdate();
    const int result = cupResult();
    if (result == 0) return;
    running = false;
    robot_serial_message(result > 0 && lineStart() ? "Quille terminee. Reprise du suivi de ligne." : "Test interrompu. Robot arrete.");
}
