#include "declarations.h"

// 64 cycles x 4 conservent l'exposition nominale de la calibration.
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_4X);
