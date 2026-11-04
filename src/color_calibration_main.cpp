#include "declarations.h"

namespace {
constexpr uint8_t SAMPLE_COUNT = 8;
const char *const COLORS[] = {"rose", "turquoise", "jaune", "orange", "bleu"};
bool ready = false;
int selected = -1;
uint8_t count = 0;
unsigned long lastSample = 0;
uint16_t low[4], high[4];
uint32_t sum[4];

void printRanges() {
    Serial.print(F("Calibration "));
    Serial.println(COLORS[selected]);
    const char *const channels[] = {"R", "V", "B", "Clair"};
    for (uint8_t i = 0; i < 4; ++i) {
        Serial.print(channels[i]);
        Serial.print(F(" min=")); Serial.print(low[i]);
        Serial.print(F(" max=")); Serial.print(high[i]);
        Serial.print(F(" moyenne=")); Serial.println(sum[i] / count);
    }
}
}

void setup() {
    BoardInit();
    MOTOR_SetSpeed(LEFT, 0.0f);
    MOTOR_SetSpeed(RIGHT, 0.0f);
    Serial.begin(9600);
    ready = tcs.begin();
    if (ready) {
        Serial.println(F("CALIBRATION COULEUR"));
        Serial.println(F("Placer l'echantillon a la hauteur de travail."));
        Serial.println(F("1=rose 2=turquoise 3=jaune 4=orange 5=bleu X=annuler"));
    } else {
        Serial.println(F("TCS34725 absent. Verifier le branchement I2C et redemarrer."));
    }
}

void loop() {
    if (!ready) return;
    if (Serial.available()) {
        const char key = Serial.read();
        if (key == 'x' || key == 'X') {
            selected = -1;
            Serial.println(F("Capture annulee."));
        } else if (key >= '1' && key <= '5') {
            selected = key - '1';
            count = 0;
            for (uint8_t i = 0; i < 4; ++i) {
                low[i] = UINT16_MAX;
                high[i] = 0;
                sum[i] = 0;
            }
            lastSample = millis();
            Serial.print(F("Capture en cours : "));
            Serial.println(COLORS[selected]);
        }
    }

    const unsigned long now = millis();
    if (selected < 0 || now - lastSample < COLOR_SAMPLE_MS) return;
    lastSample = now;
    const uint16_t values[] = {
        tcs.read16(TCS34725_RDATAL), tcs.read16(TCS34725_GDATAL),
        tcs.read16(TCS34725_BDATAL), tcs.read16(TCS34725_CDATAL)
    };
    for (uint8_t i = 0; i < 4; ++i) {
        low[i] = min(low[i], values[i]);
        high[i] = max(high[i], values[i]);
        sum[i] += values[i];
    }
    if (++count == SAMPLE_COUNT) {
        printRanges();
        selected = -1;
    }
}
