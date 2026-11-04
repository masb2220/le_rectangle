#include "declarations.h"

State state = reading;

namespace {
constexpr unsigned long LED_MS = 10000;
constexpr uint8_t LED_PINS[] = {45, 44, 43, 42, 41};
bool ledActive[5] = {};
unsigned long ledSince[5] = {};
unsigned long lastSample = 0;
bool sensorReady = false;

constexpr uint8_t COLOR_ERROR_PERCENT = 10;
static_assert(COLOR_ERROR_PERCENT <= 100, "Pourcentage hors plage");

struct ChannelRange {
  uint16_t minValue;
  uint16_t maxValue;

  bool contains(uint16_t value) const {
    const uint32_t lower = uint32_t(minValue) * (100 - COLOR_ERROR_PERCENT) / 100;
    const uint32_t upper = (uint32_t(maxValue) * (100 + COLOR_ERROR_PERCENT) + 99) / 100;
    return value >= lower && value <= upper;
  }
};

struct ColorRange {
  State color;
  ChannelRange red;
  ChannelRange green;
  ChannelRange blue;
};

// Bornes RVB mesurees. La marge est appliquee par contains().
// Pour l'orange, la seconde capture est la reference.
constexpr ColorRange COLOR_RANGES[] = {
  {pink,   {960, 985}, {717, 732}, {628, 641}},
  {cyan,   {609, 610}, {906, 907}, {738, 739}},
  {yellow, {848, 848}, {974, 975}, {528, 528}},
  {orange, {836, 838}, {711, 712}, {445, 446}},
  {blue,   {357, 361}, {657, 664}, {658, 664}},
};

State classify(uint16_t r, uint16_t g, uint16_t b) {
  for (const ColorRange &range : COLOR_RANGES) {
    if (range.red.contains(r) && range.green.contains(g) && range.blue.contains(b)) {
      return range.color;
    }
  }
  return reading;
}
}

bool initColorSensor() {
  initLeds();
  sensorReady = tcs.begin();
  lastSample = millis();
  return sensorReady;
}

void colorReaction(bool diagnostics) {
  const unsigned long now = millis();
  if (!sensorReady || now - lastSample < COLOR_SAMPLE_MS) return;
  lastSample = now;

  // Le capteur integre en continu. Lire ses registres sans le delay de getRawData().
  const uint16_t r = tcs.read16(TCS34725_RDATAL);
  const uint16_t g = tcs.read16(TCS34725_GDATAL);
  const uint16_t b = tcs.read16(TCS34725_BDATAL);
  const State detected = classify(r, g, b);
  if (diagnostics) {
    static const char *const names[] = {"inconnue", "rose", "bleu", "turquoise", "orange", "jaune"};
    Serial.print(F("R=")); Serial.print(r);
    Serial.print(F(" V=")); Serial.print(g);
    Serial.print(F(" B=")); Serial.print(b);
    Serial.print(F(" couleur=")); Serial.println(names[detected]);
  }
  if (detected == state) return;
  state = detected;

  uint8_t led;
  switch (detected) {
    case pink: led = 0; break;
    case orange: led = 1; break;
    case yellow: led = 2; break;
    case cyan: led = 3; break;
    case blue: led = 4; break;
    default: return;
  }
  digitalWrite(LED_PINS[led], LOW);
  ledActive[led] = true;
  ledSince[led] = now;
}

bool colorBlueHold() {
  static bool waiting = false;
  static bool sawBlue = false;
  const bool blueNow = state == blue;
  if (blueNow && !sawBlue) waiting = true;
  sawBlue = blueNow;
  if (waiting && receiveIR() == IR_WAKE_SIGNAL) waiting = false;
  return waiting;
}

void initLeds() {
  for (uint8_t pin : LED_PINS) {
    digitalWrite(pin, HIGH);
    pinMode(pin, OUTPUT);
  }
}

void updateLeds() {
  const unsigned long now = millis();
  for (uint8_t i = 0; i < 5; ++i) {
    if (ledActive[i] && now - ledSince[i] >= LED_MS) {
      digitalWrite(LED_PINS[i], HIGH);
      ledActive[i] = false;
    }
  }
}
