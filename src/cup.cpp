#include "cup.h"
#include "cup_return.h"
#include "scan_config.h"
#include "distance_ir.h"
#include "arm.h"
#include "robot_control.h"
#include "robot_serial.h"
#include <math.h>
#include <stdio.h>

namespace {
constexpr float PI_F = 3.14159265359f;
enum class State { IDLE, LOWER_BEFORE_SCAN, SCANNING, SETTLING, ROTATING_OFFSET,
    TUNING, NOT_FOUND, FAILED, REVERSING, REVERSE_SETTLE, CLOSING, LIFTING, DONE, INITIAL_CHECK, ESCAPE_SETTLE, ESCAPE_RIGHT, RESTART_SETTLE, RETURNING, RETURN_SETTLE, RESTORING_HEADING };
State state = State::IDLE;
bool pinModeActive = false;
unsigned long phaseStart = 0;
unsigned long sampleDate = 0;
float referenceTheta = 0;
float initialTheta = 0; // Avant le scan et avant tous les detours.
float pickupDisplacementM = 0;
float requestedOffsetDeg = SCAN_OFFSET_DEG;
float detectionCm = NAN;
bool pickupAfter = true;
uint8_t scanRestarts = 0;
float scanPreviousTheta = 0;
float scanSweepRad = 0;

float wrap(float a) {
    while (a > PI_F) a -= 2*PI_F;
    while (a < -PI_F) a += 2*PI_F;
    return a;
}
const char *stateName() {
    switch (state) {
        case State::IDLE: return "repos";
        case State::LOWER_BEFORE_SCAN: return "descente";
        case State::SCANNING: return "scan";
        case State::SETTLING: return "arret detection";
        case State::ROTATING_OFFSET: return "offset";
        case State::TUNING: return "reglage manuel";
        case State::NOT_FOUND: return "aucun objet";
        case State::FAILED: return "defaut";
        case State::REVERSING: return "recul";
        case State::REVERSE_SETTLE: return "arret recul";
        case State::CLOSING: return "fermeture";
        case State::LIFTING: return "flip";
        case State::DONE: return "termine";
        case State::INITIAL_CHECK: return "verification A0";
        case State::ESCAPE_SETTLE: return "arret avant detour";
        case State::ESCAPE_RIGHT: return "detour droite";
        case State::RESTART_SETTLE: return "arret avant reprise";
        case State::RETURNING: return "retour distance";
        case State::RETURN_SETTLE: return "arret retour";
        case State::RESTORING_HEADING: return "orientation initiale";
    }
    return "inconnu";
}
bool thetaNow(float &theta) {
    float x, y;
    return robot_control_get_pose(x, y, theta);
}
void fail(const char *message) {
    robot_control_cancel();
    state = State::FAILED;
    robot_serial_message(message);
}
void initialCheck() {
    sampleDate = lireDistance(1).dateMs;
    phaseStart = millis();
    state = State::INITIAL_CHECK;
}
void escapeInitialObject() {
    robot_control_cancel();
    if (scanRestarts >= MAX_SCAN_RESTARTS) {
        fail("Objet toujours present au depart : nombre maximal de reprises atteint.");
        return;
    }
    ++scanRestarts;
    state = State::ESCAPE_SETTLE;
    phaseStart = millis();
    char message[90];
    snprintf(message, sizeof(message), "Detection initiale : detour droite %d deg, reprise %u/%u.",
        int(SCAN_RESTART_RIGHT_DEG), scanRestarts, MAX_SCAN_RESTARTS);
    robot_serial_message(message);
}
bool startLeftScan() {
    if (!thetaNow(scanPreviousTheta) ||
        !robot_control_start_rotation(2*PI_F, SCAN_SPEED_RAD_S)) return false;
    scanSweepRad = 0;
    state = State::SCANNING;
    phaseStart = millis();
    char message[90];
    snprintf(message, sizeof(message), "Scan gauche : vitesse=%d mrad/s, seuil A0=%d cm.",
        int(SCAN_SPEED_RAD_S*1000), int(LEFT_DETECTION_MAX_CM));
    robot_serial_message(message);
    return true;
}
bool finishOffset() {
    if (pinModeActive) {
        const float approach = wrap(referenceTheta + requestedOffsetDeg*PI_F/180.0f - initialTheta);
        const bool right = sinf(approach) > 0.001f;
        pickupDisplacementM = -(detectionCm/100.0f + PIN_CONTACT_ADJUST_M);
        if (pickupDisplacementM >= 0 ||
            !robot_control_start_straight(pickupDisplacementM,
                right ? PIN_RIGHT_SPEED_M_S : PIN_LEFT_SPEED_M_S)) return false;
        state = State::REVERSING;
        robot_serial_message(right ? "Quille a droite : recul rapide." :
                                     "Quille a gauche ou au centre : recul lent.");
        return true;
    }
    if (!pickupAfter) {
        state = State::TUNING;
        robot_serial_message("Reglage pret : + = 1 deg gauche, - = 1 deg droite.");
        cupPrintOffset();
        return true;
    }
    // Metres signes : negatif = recul du robot, vers la pince arriere.
    pickupDisplacementM = -PICKUP_REVERSE_MM/1000.0f;
    if (!isfinite(pickupDisplacementM) || pickupDisplacementM >= 0 ||
        !robot_control_start_straight(pickupDisplacementM, PICKUP_STRAIGHT_SPEED_M_S)) return false;
    state = State::REVERSING;
    char message[80];
    snprintf(message, sizeof(message), "Offset termine. Recul a cap fixe de %d mm.",int(PICKUP_REVERSE_MM));
    robot_serial_message(message);
    return true;
}
bool goToOffset(float degrees) {
    float theta;
    if (!robot_control_finished() || !thetaNow(theta)) return false;
    const float delta = wrap(referenceTheta + degrees*PI_F/180.0f - theta);
    if (fabsf(delta) < 0.001f) {
        requestedOffsetDeg = degrees;
        return finishOffset();
    }
    if (!robot_control_start_rotation(delta, OFFSET_SPEED_RAD_S)) return false;
    requestedOffsetDeg = degrees;
    state = State::ROTATING_OFFSET;
    return true;
}
}
int cupResult() {
    if (state == State::DONE) return 1;
    if (state == State::FAILED || state == State::NOT_FOUND) return -1;
    return 0;
}

void cupBegin() {
    initDistance();
}
static bool startScan(bool pickupAfterScan, bool forPin) {
    const bool available = state == State::IDLE || state == State::TUNING ||
        state == State::NOT_FOUND || state == State::FAILED || state == State::DONE;
    if (!available || !robot_control_finished()) return false;
    if (!thetaNow(initialTheta)) return false;
    pinModeActive = forPin;
    pickupDisplacementM = 0;
    scanRestarts = 0;
    pickupAfter = pickupAfterScan;
    requestedOffsetDeg = forPin ? PIN_OFFSET_DEG : SCAN_OFFSET_DEG;
    detectionCm = NAN;
    // Reouvrir pour permettre un nouveau cycle apres une prise precedente.
    if (forPin) {
        initialCheck();
        robot_serial_message("Rose : recherche de la quille avec A0.");
        return true;
    }
    if (pickupAfter) openThumb();
    flipDown();
    state = State::LOWER_BEFORE_SCAN;
    phaseStart = millis();
    robot_serial_message(pickupAfter
        ? "Prise : ouverture et descente, puis scan gauche."
        : "TUNE : descente puis scan gauche, sans ramassage.");
    return true;
}
bool cupStart(bool pickupAfterScan) {
    return startScan(pickupAfterScan, false);
}

bool pinStart() {
    return startScan(false, true);
}

void cupAbort() {
    robot_control_cancel();
    state = State::IDLE;
    robot_serial_message("STOP : sequence annulee, roues arretees, servos maintenus.");
}
bool cupAdjustOffset(int direction) {
    if (state != State::TUNING || pickupAfter || (direction != 1 && direction != -1)) return false;
    const float next = requestedOffsetDeg + direction*OFFSET_STEP_DEG;
    if (next < -180.0f || next > 180.0f) return false;
    return goToOffset(next);
}
void cupPrintOffset() {
    char message[110];
    const long value = lroundf(requestedOffsetDeg*1000.0f);
    snprintf(message, sizeof(message), "Offset cible=%ld millidegres (+gauche / -droite).",value);
    robot_serial_message(message);
    const unsigned long magnitude = value < 0 ? -value : value;
    snprintf(message, sizeof(message), "A copier dans scan_config.h : SCAN_OFFSET_DEG = %s%lu.%03luf;",
        value < 0 ? "-" : "",magnitude/1000UL,magnitude%1000UL);
    robot_serial_message(message);
}
void cupUpdate() {
    if (state == State::IDLE || state == State::TUNING || state == State::NOT_FOUND ||
        state == State::FAILED || state == State::DONE) return;
    if (robot_control_fault()) { state = State::FAILED; robot_serial_message("Defaut PID : sequence interrompue."); return; }
    if (state == State::LOWER_BEFORE_SCAN) {
        if (millis() - phaseStart < ARM_MOVE_WAIT_MS) return;
        initialCheck();
    } else if (state == State::INITIAL_CHECK) {
        const DistanceIR left = lireDistance(1);
        if (left.dateMs != sampleDate) {
            sampleDate = left.dateMs;
            if (left.valide && left.cm <= LEFT_DETECTION_MAX_CM) escapeInitialObject();
            else if (!startLeftScan()) fail("Rotation gauche refusee.");
        } else if (millis() - phaseStart > DISTANCE_SAMPLE_TIMEOUT_MS) fail("Pas de nouvelle mesure A0 : scan refuse.");
    } else if (state == State::ESCAPE_SETTLE && millis() - phaseStart >= MOTION_SETTLE_MS) {
        if (!robot_control_start_rotation(-SCAN_RESTART_RIGHT_DEG*PI_F/180.0f, SCAN_SPEED_RAD_S)) {
            fail("Detour a droite refuse."); return;
        }
        state = State::ESCAPE_RIGHT;
    } else if (state == State::ESCAPE_RIGHT && robot_control_finished()) {
        state = State::RESTART_SETTLE;
        phaseStart = millis();
    } else if (state == State::RESTART_SETTLE && millis() - phaseStart >= MOTION_SETTLE_MS) {
        initialCheck();
    } else if (state == State::SCANNING) {
        float theta;
        if (!thetaNow(theta)) { fail("Pose scan indisponible."); return; }
        scanSweepRad += wrap(theta-scanPreviousTheta);
        scanPreviousTheta = theta;
        const DistanceIR left = lireDistance(1);
        if (left.dateMs != sampleDate) {
            sampleDate = left.dateMs;
            if (left.valide && left.cm <= LEFT_DETECTION_MAX_CM) {
                if (scanSweepRad <= EARLY_DETECTION_DEG*PI_F/180.0f) {
                    escapeInitialObject();
                    return;
                }
                if (!thetaNow(referenceTheta)) { fail("Pose indisponible."); return; }
                detectionCm = left.cm;
                robot_control_cancel();
                state = State::SETTLING;
                phaseStart = millis();
                char message[100];
                const unsigned int dixiemesCm = lroundf(detectionCm*10.0f);
                snprintf(message, sizeof(message), "Objet detecte par le gauche : distance=%u.%u cm. Roues arretees.",
                    dixiemesCm/10,dixiemesCm%10);
                robot_serial_message(message);
                return;
            }
        }
        if (robot_control_finished()) {
            state = State::NOT_FOUND;
            robot_serial_message("Tour termine sans detection. Aucun recul ni fermeture.");
        } else if (millis() - phaseStart > SCAN_TIMEOUT_MS) fail("Delai SCAN depasse.");
    } else if (state == State::SETTLING && millis() - phaseStart >= MOTION_SETTLE_MS) {
        if (!goToOffset(requestedOffsetDeg)) fail("Offset ou demarrage recul refuse.");
    } else if (state == State::ROTATING_OFFSET && robot_control_finished()) {
        if (!finishOffset()) fail("Recul refuse.");
    } else if (state == State::REVERSING && robot_control_finished()) {
        state = State::REVERSE_SETTLE;
        phaseStart = millis();
        robot_serial_message(pinModeActive ? "Approche quille terminee." : "Recul termine, roues arretees.");
    } else if (state == State::REVERSE_SETTLE && !pinModeActive && millis() - phaseStart >= MOTION_SETTLE_MS) {
        closeThumb();
        state = State::CLOSING;
        phaseStart = millis();
        robot_serial_message("Fermeture du pouce.");
    } else if (state == State::CLOSING && millis() - phaseStart >= THUMB_CLOSE_WAIT_MS) {
        flipUp();
        state = State::LIFTING;
        phaseStart = millis();
        robot_serial_message("Flip : relevage de la pince.");
    } else if ((state == State::LIFTING && millis() - phaseStart >= ARM_MOVE_WAIT_MS) ||
               (state == State::REVERSE_SETTLE && pinModeActive && millis() - phaseStart >= MOTION_SETTLE_MS)) {
        const float distance = cupReturnDistance(pickupDisplacementM);
        if (!isfinite(distance) || distance == 0 ||
            !robot_control_start_straight(distance, pinModeActive ? PIN_RETURN_SPEED_M_S : PICKUP_STRAIGHT_SPEED_M_S)) {
            fail("Retour en ligne droite refuse."); return;
        }
        state = State::RETURNING;
        char message[90];
        snprintf(message, sizeof(message), "Retour de %d mm en sens inverse.", int(fabsf(distance)*1000));
        robot_serial_message(message);
    } else if (state == State::RETURNING && robot_control_finished()) {
        state = State::RETURN_SETTLE;
        phaseStart = millis();
        robot_serial_message("Distance de retour terminee. Reprise de l'orientation initiale.");
    } else if (state == State::RETURN_SETTLE && millis() - phaseStart >= MOTION_SETTLE_MS) {
        float theta;
        if (!thetaNow(theta)) { fail("Pose retour indisponible."); return; }
        const float delta = cupReturnAngle(initialTheta, theta);
        if (fabsf(delta) < 0.001f) {
            state = State::DONE;
            robot_serial_message("Sequence terminee : distance et orientation retablies (odometrie).");
        } else if (!robot_control_start_rotation(delta, RETURN_HEADING_SPEED_RAD_S)) {
            fail("Retour a l'orientation initiale refuse.");
        } else state = State::RESTORING_HEADING;
    } else if (state == State::RESTORING_HEADING && robot_control_finished()) {
        state = State::DONE;
        robot_serial_message("Sequence terminee : distance et orientation retablies (odometrie).");
    }
}
void cupPrintStatus() {
    char message[120];
    snprintf(message, sizeof(message), "Etat=%s reprises=%u/%u", stateName(), scanRestarts, MAX_SCAN_RESTARTS);
    robot_serial_message(message);
    snprintf(message, sizeof(message), "Temps=%lu s PID defaut=%u cause=%u retard=%lu us offset=%ld mdeg",
        millis()/1000UL,unsigned(robot_control_fault()),unsigned(robot_control_fault_reason()),
        robot_control_fault_interval_us(),lroundf(requestedOffsetDeg*1000));
    robot_serial_message(message);
    const DistanceIR left=lireDistance(1);
    snprintf(message, sizeof(message), "Gauche A0 brut=%u distance=%d mm valide=%u age=%lu ms detectee=%d mm",
        left.brut,left.valide?int(left.cm*10):-1,unsigned(left.valide),millis()-left.dateMs,
        isfinite(detectionCm)?int(detectionCm*10):-1);
    robot_serial_message(message);
}
