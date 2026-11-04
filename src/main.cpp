#include "declarations.h"
#include "robot_control.h"
#include "robot_serial.h"
#include "wall_detour.h"
#include "cup.h"
#include "arm.h"
#include "distance_ir.h"
#include "LineFollower/line_follower.h"

namespace {
static_assert(IR_WAKE_SIGNAL != IR_START_SIGNAL, "Codes IR distincts requis");
enum class GapPhase { Idle, BeforeApproach, ApproachingBar, Settling, Squaring, Crossing, CheckingExit, FindingExit };
GapPhase gapPhase = GapPhase::Idle;
constexpr float HALF_TURN = 3.14159265f;
constexpr float APPROACH_LIMIT_M = 0.50f;
constexpr float APPROACH_SPEED_M_S = 0.08f;
constexpr float GAP_DISTANCE_M = 0.70f;
constexpr float GAP_SPEED_M_S = 0.12f;
constexpr float EXIT_SEARCH_LIMIT_M = 0.40f;
constexpr float EXIT_SEARCH_SPEED_M_S = 0.06f;
constexpr float TURN_SPEED_RAD_S = 0.20f;
constexpr unsigned long SETTLE_MS = 250UL;
constexpr unsigned long BLUE_INHIBIT_MS = 2000UL;
bool blueInhibited = false;
unsigned long wokeAt = 0;
constexpr float OUTER_SENSOR_SPACING_M = 0.040f; // Mesurer entre les centres gauche et droit.
constexpr float MAX_SQUARE_ANGLE = 35.0f * HALF_TURN / 180.0f;
constexpr unsigned long ALIGN_SAMPLE_MS = 2 * LINE_PERIOD_MS;
constexpr uint8_t ALIGN_SAMPLES = 3;
float approachX, approachY, approachHeading;
float edgePosition[2];
bool edgeSeen[2];
bool wasBlack[2];
unsigned long lastAlignmentSample = 0;
uint8_t matchingSamples = 0;
bool ignoreGreenUntilClear = false;
bool ignoreYellowUntilClear = false;
unsigned long stoppedAt = 0;
bool colorReady = false;
constexpr unsigned long COLOR_COOLDOWN_MS = 2000UL;
bool pickupRunning = false;
bool colorCooldown = false;
bool ignoreOrangeUntilClear = false;
unsigned long pickupFinishedAt = 0;

void readLineDarkness(uint16_t darkness[3]) {
    uint16_t raw[3];
    lineUpdate(); // Lecture seule, les roues sont gerees par le controle de mouvement.
    lineReadStatus(raw, darkness);
}

void abortGap(const char *message) {
    robot_control_cancel();
    gapPhase = GapPhase::Idle;
    robot_serial_message(message);
}

void updateGap() {
    const unsigned long now = millis();
    switch (gapPhase) {
        case GapPhase::BeforeApproach: {
            if (now - stoppedAt < SETTLE_MS) return;
            if (!lineCalibrationReady() || robot_control_background_time_us() <= 2500) return;
            if (!robot_control_get_pose(approachX, approachY, approachHeading)) break;
            uint16_t darkness[3];
            readLineDarkness(darkness);
            for (uint8_t side = 0; side < 2; ++side) {
                wasBlack[side] = darkness[side * 2] >= LINE_WEAK_SIGNAL;
                edgeSeen[side] = false;
            }
            if (robot_control_start_straight(APPROACH_LIMIT_M, APPROACH_SPEED_M_S)) {
                matchingSamples = 0;
                lastAlignmentSample = now;
                gapPhase = GapPhase::ApproachingBar;
                return;
            }
            break;
        }

        case GapPhase::ApproachingBar:
            if (now - lastAlignmentSample >= ALIGN_SAMPLE_MS &&
                robot_control_background_time_us() > 2500) {
                lastAlignmentSample = now;
                uint16_t darkness[3];
                readLineDarkness(darkness);
                float x, y, heading;
                if (!robot_control_get_pose(x, y, heading)) break;
                const float along = (x-approachX)*cosf(approachHeading) +
                                    (y-approachY)*sinf(approachHeading);
                for (uint8_t side = 0; side < 2; ++side) {
                    const bool black = darkness[side * 2] >= LINE_WEAK_SIGNAL;
                    if (black && !wasBlack[side]) {
                        edgePosition[side] = along;
                        edgeSeen[side] = true;
                    }
                    wasBlack[side] = black;
                }
                const bool allBlack = darkness[0] >= LINE_WEAK_SIGNAL &&
                    darkness[1] >= LINE_WEAK_SIGNAL &&
                    darkness[2] >= LINE_WEAK_SIGNAL;
                if (allBlack) ++matchingSamples;
                else matchingSamples = 0;
                if (matchingSamples >= ALIGN_SAMPLES) {
                    robot_control_cancel();
                    stoppedAt = millis();
                    matchingSamples = 0;
                    gapPhase = GapPhase::Settling;
                    robot_serial_message("Noir/noir/noir confirme. Mesure de l'equerrage.");
                    return;
                }
            }
            if (robot_control_finished()) {
                abortGap("Barre noire non detectee dans les 50 cm. Robot arrete.");
            }
            return;

        case GapPhase::Settling: {
            if (now - stoppedAt < SETTLE_MS) return;
            if (!edgeSeen[0] || !edgeSeen[1]) {
                abortGap("Equerrage : bord non observe par les deux capteurs.");
                return;
            }
            float x, y, heading;
            if (!robot_control_get_pose(x, y, heading)) break;
            const float edgeGap = edgePosition[1] - edgePosition[0];
            const float correction = atan2f(edgeGap, OUTER_SENSOR_SPACING_M);
            float delta = approachHeading + correction - heading;
            while (delta > HALF_TURN) delta -= 2 * HALF_TURN;
            while (delta < -HALF_TURN) delta += 2 * HALF_TURN;
            char message[85];
            snprintf(message, sizeof(message), "Equerrage : ecart de bord=%ld mm, correction=%ld deg.",
                lroundf(edgeGap * 1000.0f), lroundf(delta * 180.0f / HALF_TURN));
            robot_serial_message(message);
            if (fabsf(delta) > MAX_SQUARE_ANGLE) {
                abortGap("Equerrage : angle mesure trop grand.");
                return;
            }
            if (fabsf(delta) >= HALF_TURN / 180.0f &&
                !robot_control_start_rotation(delta, TURN_SPEED_RAD_S)) {
                abortGap("Equerrage : rotation refusee par le controle moteur.");
                return;
            }
            gapPhase = GapPhase::Squaring;
            return;
        }

        case GapPhase::Squaring:
            if (!robot_control_finished()) return;
            if (robot_control_start_straight(GAP_DISTANCE_M, GAP_SPEED_M_S)) {
                gapPhase = GapPhase::Crossing;
                robot_serial_message("Correction angulaire terminee. Traversee en cours.");
                return;
            }
            break;

        case GapPhase::Crossing:
            if (!robot_control_finished()) return;
            matchingSamples = 0;
            lastAlignmentSample = now;
            gapPhase = GapPhase::CheckingExit;
            robot_serial_message("Traversee terminee. Verification de la ligne a la sortie.");
            return;

        case GapPhase::CheckingExit:
        case GapPhase::FindingExit: {
            if (now - lastAlignmentSample >= ALIGN_SAMPLE_MS &&
                robot_control_background_time_us() > 2500) {
                lastAlignmentSample = now;
                uint16_t darkness[3];
                readLineDarkness(darkness);
                const bool anyBlack = max(darkness[0], max(darkness[1], darkness[2])) >= LINE_MIN_SIGNAL;
                const bool anyWhite = min(darkness[0], min(darkness[1], darkness[2])) < LINE_MIN_SIGNAL;
                if (anyBlack && anyWhite) {
                    if (++matchingSamples >= ALIGN_SAMPLES) {
                        robot_control_cancel();
                        if (!lineStart()) break;
                        ignoreGreenUntilClear = true;
                        gapPhase = GapPhase::Idle;
                        robot_serial_message("Ligne de sortie confirmee. Reprise du PID.");
                        return;
                    }
                } else {
                    matchingSamples = 0;
                    if (gapPhase == GapPhase::CheckingExit) {
                        if (!robot_control_start_straight(EXIT_SEARCH_LIMIT_M, EXIT_SEARCH_SPEED_M_S)) break;
                        gapPhase = GapPhase::FindingExit;
                        robot_serial_message("Recherche de la ligne a basse vitesse.");
                        return;
                    }
                }
            }
            if (gapPhase == GapPhase::FindingExit && robot_control_finished() && matchingSamples == 0) {
                abortGap("Ligne de sortie introuvable. Robot arrete.");
            }
            return;
        }

        case GapPhase::Idle:
            return;
    }
    abortGap("Manoeuvre verte interrompue.");
}
bool waitingForWake = false;
bool startAfterBeep = false;
}

void setup() {
    armBegin();

    Serial.begin(9600);
    colorReady = initColorSensor();
    irrecv.enableIRIn();
    if (colorReady) {
        Serial.print(F("Attente IR sur D37. Code de depart attendu : "));
        Serial.println(IR_START_SIGNAL, HEX);
    } else {
        robot_serial_message("TCS34725 introuvable. Verifier le branchement et redemarrer.");
    }
    robot_control_init();
    lineBegin();
    wallDetourInit();
    cupBegin();
}

void loop() {
    robot_control_update(!lineFollowing());
    robot_serial_flush();
    updateLeds();
    if (robot_control_fault()) {
        if (lineFollowing()) lineStop();
        return;
    }
    if (pickupRunning) {
        if (robot_control_background_time_us() <= 3000) return;
        updateDistance();
        if (robot_control_background_time_us() <= 3000) return;
        cupUpdate();
        const int result = cupResult();
        if (result == 0) return;
        pickupRunning = false;
        if (result > 0 && lineStart()) {
            state = reading;
            colorCooldown = true;
            pickupFinishedAt = millis();
            robot_serial_message("Ramassage termine. Reprise du suivi de ligne.");
        } else {
            robot_serial_message("Ramassage ou reprise impossible. Robot arrete.");
        }
        return;
    }
    if (gapPhase != GapPhase::Idle) {
        updateGap();
        return;
    }
    if (wallDetourActive()) {
        wallDetourUpdate();
        return;
    }
    if (updateBeep()) return;

    if (lineFollowing()) {
        if (colorCooldown && millis() - pickupFinishedAt < COLOR_COOLDOWN_MS) {
            lineUpdate();
            if (const char *event = lineEvent()) robot_serial_message(event);
            return;
        }
        colorCooldown = false;
        if (robot_control_background_time_us() > 4000) {
            colorReaction();
            if (state != orange) ignoreOrangeUntilClear = false;
        }
        if (state == orange && !ignoreOrangeUntilClear) {
            lineStop();
            ignoreOrangeUntilClear = true;
            pickupRunning = cupStart();
            robot_serial_message(pickupRunning ? "Orange : recherche du verre." :
                                 "Ramassage refuse. Robot arrete.");
            return;
        }
        if (state != yellow) ignoreYellowUntilClear = false;
        if (state == yellow && !ignoreYellowUntilClear) {
            ignoreYellowUntilClear = true;
            wallDetourStart();
            return;
        }
        if (state != cyan) ignoreGreenUntilClear = false;
        if (state == cyan && !ignoreGreenUntilClear) {
            lineStop();
            stoppedAt = millis();
            gapPhase = GapPhase::BeforeApproach;
            robot_serial_message("Vert : avance jusqu au noir/noir/noir.");
            return;
        }
        if (blueInhibited && millis() - wokeAt >= BLUE_INHIBIT_MS) blueInhibited = false;
        if (state != reading && state != cyan && state != yellow && state != orange && !(state == blue && blueInhibited)) {
            lineStop();
            waitingForWake = state == blue;
            const char *const names[] = {"inconnue", "rose", "bleu", "turquoise", "orange", "jaune"};
            robot_serial_message(names[state]);
        } else {
            lineUpdate();
        }
        if (const char *event = lineEvent()) robot_serial_message(event);
        return;
    }

    if (startAfterBeep) {
        startAfterBeep = false;
        if (lineStart()) {
            blueInhibited = waitingForWake;
            wokeAt = millis();
            waitingForWake = false;
            state = reading;
            robot_serial_message("Suivi de ligne actif.");
        } else {
            robot_serial_message("Depart refuse : calibration de ligne invalide.");
        }
        return;
    }

    const uint32_t signal = receiveIR();
    if (!signal) return;
    beep(waitingForWake ? 3 : 2);
    const uint32_t expected = waitingForWake ? IR_WAKE_SIGNAL : IR_START_SIGNAL;
    if (signal != expected) {
        robot_serial_message("Message IR confirme par le buzzer, code refuse.");
        return;
    }
    if (!colorReady) {
        robot_serial_message("Message IR accepte, capteur couleur indisponible.");
        return;
    }
    startAfterBeep = true;
}
