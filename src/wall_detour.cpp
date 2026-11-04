#include <Arduino.h>
#include "wall_detour.h"
#include "PathFinding/obstacle_sensor.h"
#include "LineFollower/line_follower.h"
#include "robot_control.h"
#include "robot_serial.h"

namespace {
enum class Phase { Idle, LookingForWall, WallStop, RightTurn,
    FirstLeg, FirstLeft, SecondLeg, SecondLeft, LookingForLine, LineStop, FinalTurn };

constexpr float SEARCH_LIMIT_M = 1.0f;
constexpr float FIRST_LEG_DISTANCE_M = 0.50f;
constexpr float SECOND_LEG_DISTANCE_M = 0.650f;
constexpr float SEARCH_SPEED_M_S = 0.2f;
constexpr float LEG_SPEED_M_S = 0.8f;
constexpr float TURN_SPEED_RAD_S = 4.40f;
// Angles signes en degres : positif a gauche, negatif a droite.
constexpr float WALL_TURN_DEG = -90.0f;
constexpr float FIRST_CORNER_DEG = 90.0f;
constexpr float SECOND_CORNER_DEG = 90.0f;
constexpr float LINE_REJOIN_TURN_DEG = -30.0f;
constexpr float DEG_TO_RADIANS = 0.01745329252f;
constexpr unsigned long SETTLE_MS = 250UL;
constexpr unsigned long WALL_SEARCH_TIMEOUT_MS = 15000UL;
constexpr uint8_t LINE_SAMPLES = 3;
const obstacle_sensor_pins wallSensors;

Phase phase = Phase::Idle;
unsigned long stoppedAt = 0;
unsigned long wallSearchStartedAt = 0;
float wallStartX = 0;
float wallStartY = 0;
unsigned long lastSample = 0;
uint8_t matchingSamples = 0;

void stopAt(Phase next) {
    robot_control_cancel();
    stoppedAt = millis();
    phase = next;
}

bool advance(float distance, float speed, Phase next) {
    if (!robot_control_start_straight(distance, speed)) return false;
    phase = next;
    return true;
}

bool rotate(float angleDegrees, Phase next) {
    if (!robot_control_start_rotation(angleDegrees * DEG_TO_RADIANS, TURN_SPEED_RAD_S)) return false;
    phase = next;
    return true;
}

void abortDetour(const char *message) {
    stopAt(Phase::Idle);
    robot_serial_message(message);
}

bool wallDetected() {
    bool left, right;
    obstacle_sensor_read(wallSensors, left, right);
    return left || right;
}
}

void wallDetourInit() {
    obstacle_sensor_init(wallSensors);
}

void wallDetourStart() {
    float heading;
    if (!lineFollowing() || !robot_control_get_pose(wallStartX, wallStartY, heading)) {
        lineStop();
        abortDetour("Jaune : approche du mur indisponible.");
        return;
    }
    wallSearchStartedAt = millis();
    phase = Phase::LookingForWall;
    robot_serial_message("Jaune : suivi de ligne jusqu'au mur (D49/D48).");
}

bool wallDetourActive() {
    return phase != Phase::Idle;
}

void wallDetourUpdate() {
    const unsigned long now = millis();
    switch (phase) {
        case Phase::Idle:
            return;
        case Phase::LookingForWall: {
            if (wallDetected()) {
                lineStop();
                stopAt(Phase::WallStop);
                robot_serial_message("Mur detecte. Arret avant virage a droite.");
                return;
            }

            float x, y, heading;
            if (!robot_control_get_pose(x, y, heading)) {
                lineStop();
                abortDetour("Jaune : odometrie indisponible pendant l'approche.");
                return;
            }
            const float dx = x - wallStartX;
            const float dy = y - wallStartY;
            if (dx*dx + dy*dy >= SEARCH_LIMIT_M*SEARCH_LIMIT_M ||
                now - wallSearchStartedAt >= WALL_SEARCH_TIMEOUT_MS) {
                lineStop();
                abortDetour("Jaune : limite de recherche du mur atteinte.");
                return;
            }
            lineUpdate();
            if (const char *event = lineEvent()) robot_serial_message(event);
            if (!lineFollowing()) abortDetour("Jaune : ligne perdue avant le mur.");
            return;
        }
        case Phase::WallStop:
            if (now - stoppedAt < SETTLE_MS) return;
            if (rotate(WALL_TURN_DEG, Phase::RightTurn)) return;
            break;
        case Phase::RightTurn:
            if (!robot_control_finished()) return;
            if (advance(FIRST_LEG_DISTANCE_M, LEG_SPEED_M_S, Phase::FirstLeg)) return;
            break;
        case Phase::FirstLeg:
            if (!robot_control_finished()) return;
            if (rotate(FIRST_CORNER_DEG, Phase::FirstLeft)) return;
            break;
        case Phase::FirstLeft:
            if (!robot_control_finished()) return;
            if (advance(SECOND_LEG_DISTANCE_M, LEG_SPEED_M_S, Phase::SecondLeg)) return;
            break;
        case Phase::SecondLeg:
            if (!robot_control_finished()) return;
            if (rotate(SECOND_CORNER_DEG, Phase::SecondLeft)) return;
            break;
        case Phase::SecondLeft:
            if (!robot_control_finished()) return;
            lastSample = now;
            matchingSamples = 0;
            if (advance(SEARCH_LIMIT_M, SEARCH_SPEED_M_S, Phase::LookingForLine)) return;
            break;
        case Phase::LookingForLine: {
            if (now - lastSample >= 2 * LINE_PERIOD_MS &&
                robot_control_background_time_us() > 2500) {
                lastSample = now;
                uint16_t raw[3], darkness[3];
                lineUpdate();
                lineReadStatus(raw, darkness);
                const bool black = max(darkness[0], max(darkness[1], darkness[2])) >= LINE_MIN_SIGNAL;
                matchingSamples = black ? matchingSamples + 1 : 0;
                if (matchingSamples >= LINE_SAMPLES) {
                    stopAt(Phase::LineStop);
                    robot_serial_message("Ligne retrouvee. Virage de reprise.");
                    return;
                }
            }
            if (robot_control_finished()) abortDetour("Jaune : ligne introuvable dans le metre de recherche.");
            return;
        }
        case Phase::LineStop:
            if (now - stoppedAt < SETTLE_MS) return;
            if (rotate(LINE_REJOIN_TURN_DEG, Phase::FinalTurn)) return;
            break;
        case Phase::FinalTurn:
            if (!robot_control_finished()) return;
            if (lineStart()) {
                phase = Phase::Idle;
                robot_serial_message("Detour jaune termine. Reprise du suivi de ligne.");
                return;
            }
            break;
    }
    abortDetour("Jaune : mouvement refuse. Robot arrete.");
}
