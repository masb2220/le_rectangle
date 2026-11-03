#include "declarations.h"

void colorReaction() {
  switch (state) {
    case reading: {
      uint16_t r, g, b, c, colorTemp, lux;
      tcs.getRawData(&r, &g, &b, &c);
      colorTemp = tcs.calculateColorTemperature(r, g, b);
      lux = tcs.calculateLux(r, g, b);

      Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
      Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
      Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
      Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
      Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
      Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
      Serial.println(" ");

      if (r >= 800 && g >= 650 && g <= 950 && b >= 400 && b <= 800) {
        Serial.println("pink detected");
        state = pink;
      }
      if (r >= 500 && r <= 800 && g >= 900 && b >= 600 && b <= 1000) {
        Serial.println("cyan detected");
        state = cyan;
      }
      if (r >= 900 && r <= 1400 && g >= 900 && g <= 1400 && b >= 500 && b <= 800) {
        Serial.println("yellow detected");
        state = yellow;
      }
      if (r >= 200 && r <= 600 && g >= 600 && g <= 1100 && b >= 750) {
        Serial.println("blue detected");
        state = blue;
      }
      if (r >= 200 && r <= 600 && g >= 600 && g <= 1100 && b >= 750) {
        Serial.println("blue detected");   // TODO: orange needs its own RGB ranges
        state = orange;
      }
      break;
    }

    case pink:
    case blue:
    case cyan:
    case orange:
    case yellow:
      state = reading;
      break;
  }
}