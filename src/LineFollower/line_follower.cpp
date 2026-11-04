#include "line_follower.h"
#include "line_recovery.h"
#include "line_steering.h"
#include <LibRobus.h>
#include <math.h>
#include "Control/PID_controller.h"

namespace {
uint16_t rawValues[3] = {};
uint16_t blackness[3] = {};
uint16_t whiteValues[3] = {};
uint16_t blackValues[3] = {};
uint32_t calibrationSum[3] = {};
bool haveWhite = false;
bool haveBlack = false;
bool following = false;
bool losingLine = false;
unsigned long lostSince = 0;
float lastError = 0;
int preferredDirection = -1; // Gauche par defaut si aucune direction n'a ete observee.
uint8_t reacquireCount = 0;
LineRecoveryStage recoveryStage = LineRecoveryStage::BRIDGE;
float filteredError = 0;
float previousFilteredError = 0;
float filteredDerivative = 0;
bool derivativeReady = false;
float forwardCommand = 0;
float steeringCommand = 0;
float lastTurn = 0;
float leftCommand = 0;
float rightCommand = 0;
PID_controller linePid;

void resetSteering()
{
    lastError = filteredError = lastTurn = 0;
    previousFilteredError = filteredDerivative = 0;
    derivativeReady = false;
    forwardCommand = steeringCommand = 0;
    reacquireCount = 0;
    preferredDirection = -1;
    leftCommand = rightCommand = 0;
    PID_config(linePid, LINE_KP, LINE_KI, 0.0f, -LINE_MAX_TURN,
        LINE_MAX_TURN, LINE_PERIOD_MS / 1000.0f);
}
float approach(float current, float target, float maximumStep)
{
    return current + constrain(target-current, -maximumStep, maximumStep);
}
bool calibrating = false;
bool calibratingBlack = false;
uint8_t calibrationCount = 0;
unsigned long lastSample = 0;
const char *pendingEvent = nullptr;

void commandWheels(float base, float turn, float dt, bool recovery)
{
    // Ramper la vitesse et la direction separement : une correction terminee
    // doit pouvoir se relacher vite, sans prolonger le virage par inertie logicielle.
    const float speedRate = base < forwardCommand ? LINE_DECEL_PER_S : LINE_ACCEL_PER_S;
    const bool releasing = steeringCommand*turn <= 0 || fabsf(turn) < fabsf(steeringCommand);
    const float steeringRate = releasing ? LINE_STEERING_RELEASE_PER_S : LINE_STEERING_SLEW_PER_S;
    forwardCommand = approach(forwardCommand, base, speedRate*dt);
    steeringCommand = approach(steeringCommand, turn, steeringRate*dt);
    // Conserver le differentiel de direction plutot que saturer les deux roues.
    forwardCommand = fminf(forwardCommand, fmaxf(0.0f, LINE_MAX_SPEED-fabsf(steeringCommand)));
    const float minimum = recovery ? -LINE_RECOVERY_TURN : 0.0f;
    leftCommand = constrain(forwardCommand+steeringCommand, minimum, LINE_MAX_SPEED);
    rightCommand = constrain(forwardCommand-steeringCommand, minimum, LINE_MAX_SPEED);
    MOTOR_SetSpeed(LEFT, leftCommand);
    MOTOR_SetSpeed(RIGHT, rightCommand);
}

void recoverLine(unsigned long now, float dt)
{
    const LineRecoveryStage nextStage = lineRecoveryStage(now-lostSince);
    if (nextStage == LineRecoveryStage::EXPIRED) {
        lineStop();
        pendingEvent = "Ligne non retrouvee apres 3 s : arret. Bumper ou GO pour repartir.";
        return;
    }
    if (nextStage != recoveryStage) {
        recoveryStage = nextStage;
        pendingEvent = nextStage == LineRecoveryStage::ARC ? "Recherche : virage serre dans le dernier sens." :
            nextStage == LineRecoveryStage::REVERSE_SWEEP ? "Recherche : balayage dans le sens oppose." :
            "Recherche : pivot limite pour retrouver la ligne.";
    }
    float base = 0.0f;
    float turn = lineRecoveryDirection(nextStage, preferredDirection) * LINE_RECOVERY_TURN;
    if (nextStage == LineRecoveryStage::BRIDGE) {
        base = LINE_RECOVERY_BRIDGE_SPEED;
        turn = lastTurn;
    } else if (nextStage == LineRecoveryStage::ARC) {
        base = LINE_RECOVERY_ARC_SPEED;
    }
    // Recherche bornee : autoriser un pivot, plutot que continuer a sortir de piste.
    commandWheels(base, turn, dt, true);
}
void stopWheels()
{
    MOTOR_SetSpeed(LEFT, 0.0f);
    MOTOR_SetSpeed(RIGHT, 0.0f);
}
}

void lineBegin()
{
    for (uint8_t i = 0; i < 3; ++i) {
        pinMode(LINE_PINS[i], INPUT);
        whiteValues[i] = LINE_WHITE_DEFAULT[i];
        blackValues[i] = LINE_BLACK_DEFAULT[i];
    }
    haveWhite = haveBlack = true;
    lineStop();
}

void lineStop()
{
    following = false;
    losingLine = false;
    calibrating = false;
    resetSteering();
    stopWheels();
}

bool lineCalibrationReady()
{
    if (!haveWhite || !haveBlack) return false;
    for (uint8_t i = 0; i < 3; ++i) {
        if (abs(int(blackValues[i]) - int(whiteValues[i])) < LINE_MIN_CONTRAST) return false;
    }
    return true;
}

bool lineStart()
{
    if (calibrating || !lineCalibrationReady()) return false;
    losingLine = false;
    resetSteering();
    if (linePid.fault) return false;
    lastSample = millis();
    following = true;
    return true;
}

void lineCalibrate(bool black)
{
    lineStop();
    calibratingBlack = black;
    if (black) haveBlack = false;
    else haveWhite = false;
    calibrationCount = 0;
    for (uint8_t i = 0; i < 3; ++i) calibrationSum[i] = 0;
    calibrating = true;
}

void lineUpdate()
{
    const unsigned long now = millis();
    const unsigned long elapsed = now - lastSample;
    if (elapsed < LINE_PERIOD_MS) return;
    lastSample = now;
    const float dt = elapsed / 1000.0f;
    if (following && elapsed > 100UL) {
        lineStop();
        pendingEvent = "Delai de controle depasse : roues arretees.";
        return;
    }
    for (uint8_t i = 0; i < 3; ++i) {
        analogRead(LINE_PINS[i]); // Stabilisation du multiplexeur ADC.
        rawValues[i] = analogRead(LINE_PINS[i]);
    }

    if (calibrating) {
        for (uint8_t i = 0; i < 3; ++i) calibrationSum[i] += rawValues[i];
        if (++calibrationCount >= LINE_CALIBRATION_SAMPLES) {
            for (uint8_t i = 0; i < 3; ++i) {
                const uint16_t value = calibrationSum[i] / LINE_CALIBRATION_SAMPLES;
                if (calibratingBlack) blackValues[i] = value;
                else whiteValues[i] = value;
            }
            if (calibratingBlack) haveBlack = true;
            else haveWhite = true;
            calibrating = false;
            if (haveWhite && haveBlack && !lineCalibrationReady())
                pendingEvent = "Contraste insuffisant : verifier hauteur/cablage et refaire BLANC/NOIR.";
            else pendingEvent = calibratingBlack ? "Calibration NOIR terminee." : "Calibration BLANC terminee.";
        }
        return;
    }

    if (!lineCalibrationReady()) return;
    for (uint8_t i = 0; i < 3; ++i) {
        // 0=blanc, 1000=noir, quelle que soit la polarite du capteur.
        const float normalized = (int(rawValues[i]) - int(whiteValues[i])) /
            float(int(blackValues[i]) - int(whiteValues[i]));
        blackness[i] = lroundf(constrain(normalized, 0.0f, 1.0f) * 1000.0f);
    }
    if (!following) return;

    const uint16_t total = blackness[0] + blackness[1] + blackness[2];
    const uint16_t peak = max(blackness[0], max(blackness[1], blackness[2]));
    const bool visible = peak >= LINE_MIN_SIGNAL;
    if (!visible && !losingLine) {
        losingLine = true;
        lostSince = now;
        reacquireCount = 0;
        recoveryStage = LineRecoveryStage::BRIDGE;
        linePid.integrale = 0;
        linePid.init = false;
        derivativeReady = false;
        filteredDerivative = 0;
        pendingEvent = "Ligne entre les detecteurs : recuperation active (maximum 3 s).";
    }
    if (losingLine) {
        if (visible) ++reacquireCount;
        else reacquireCount = 0;
        if (reacquireCount < LINE_REACQUIRE_SAMPLES) {
            recoverLine(now, dt);
            return;
        }
        losingLine = false;
        linePid.integrale = 0;
        linePid.init = false;
        filteredError = lineShapeError((int(blackness[2])-int(blackness[0])) /
            float(max(total, LINE_ERROR_DENOMINATOR_MIN)));
        derivativeReady = false;
        filteredDerivative = 0;
        pendingEvent = "Ligne retrouvee : reprise progressive du PID.";
    }

    const float error = (int(blackness[2])-int(blackness[0])) /
        float(max(total, LINE_ERROR_DENOMINATOR_MIN));
    // Supprimer le residu integral au croisement du centre pour eviter le zigzag.
    if (error*lastError < 0 || fabsf(error) < 0.04f) linePid.integrale = 0;
    lastError = error;
    if (fabsf(error) >= 0.04f) preferredDirection = error > 0 ? 1 : -1;
    const float alpha = dt / (LINE_ERROR_FILTER_S + dt);
    const float shapedError = lineShapeError(error);
    filteredError += alpha * (shapedError-filteredError);
    const float derivative = derivativeReady ? (filteredError-previousFilteredError)/dt : 0.0f;
    const float derivativeAlpha = dt / (LINE_DERIVATIVE_FILTER_S+dt);
    filteredDerivative += derivativeAlpha*(derivative-filteredDerivative);
    previousFilteredError = filteredError;
    derivativeReady = true;
    linePid.periode = dt;
    // P+I via le controleur commun, D filtre via son entree additive :
    // la saturation et l'anti-windup voient bien la somme P+I+D.
    const float turn = compute_PID(linePid, 0.0f, -filteredError, LINE_KD*filteredDerivative);
    linePid.integrale = constrain(linePid.integrale, -LINE_INTEGRAL_LIMIT, LINE_INTEGRAL_LIMIT);
    if (linePid.fault) {
        lineStop();
        pendingEvent = "Defaut PID ligne : roues arretees.";
        return;
    }
    lastTurn = turn;
    const float quality = constrain((float(peak)-LINE_MIN_SIGNAL) /
        (LINE_WEAK_SIGNAL-LINE_MIN_SIGNAL), 0.0f, 1.0f);
    // Ralentir aussi en courbe, sans changement brutal de commande.
    const float severity = fminf(1.0f, fmaxf(fabsf(error), fabsf(filteredDerivative)*0.06f));
    const float curveScale = 1.0f - 0.65f*severity;
    const float cruise = lineCruiseLimit(LINE_BASE_SPEED, LINE_MAX_SPEED, LINE_MAX_TURN);
    const float base = cruise * curveScale *
        (LINE_WEAK_SPEED_SCALE + quality*(1.0f-LINE_WEAK_SPEED_SCALE));
    commandWheels(base, turn, dt, false);
}

const char *lineEvent()
{
    const char *event = pendingEvent;
    pendingEvent = nullptr;
    return event;
}

bool lineFollowing() { return following; }

void lineReadStatus(uint16_t raw[3], uint16_t darkness[3])
{
    for (uint8_t i = 0; i < 3; ++i) {
        raw[i] = rawValues[i];
        darkness[i] = blackness[i];
    }
}




const char *lineMode()
{
    if (!following) return "ARRET";
    if (!losingLine) return "PID";
    switch (recoveryStage) {
        case LineRecoveryStage::BRIDGE: return "PASSAGE";
        case LineRecoveryStage::ARC: return "VIRAGE";
        case LineRecoveryStage::SWEEP: return "RECHERCHE";
        case LineRecoveryStage::REVERSE_SWEEP: return "RECHERCHE_INVERSE";
        case LineRecoveryStage::FINAL_SWEEP: return "DERNIERE_RECHERCHE";
        default: return "ARRET";
    }
}

void lineReadCalibration(uint16_t white[3], uint16_t black[3])
{
    for (uint8_t i = 0; i < 3; ++i) {
        white[i] = whiteValues[i];
        black[i] = blackValues[i];
    }
}
