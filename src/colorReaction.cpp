#include "declarations.h"

namespace {
constexpr uint32_t BLUE_RESUME_SIGNAL = 0x906FC03FUL;
}
void bleu();

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

      if (r >= 800 && g >= 650 && g <= 850 && b >= 500 && b <= 700) {
        Serial.println("pink detected");
        state = pink;
      }
      if (r >= 475 && r <= 550 && g >= 675 && b >= 575 && b <= 700) {
        Serial.println("cyan detected");
        state = cyan;
      }
      if (r >= 725 && r <= 850 && g >= 850 && g <= 1000 && b >= 425 && b <= 550) {
        Serial.println("yellow detected");
        state = yellow;
      }
      if (r >= 250 && r <= 400 && g >= 500 && g <= 700 && b >= 500 && b <= 700) {
        Serial.println("blue detected");
        state = blue;
      }
      if (r >= 650 && r <= 775 && g >= 550 && g <= 675 && b >= 300 && b <= 450) {
        Serial.println("orange detected");   // TODO: orange needs its own RGB ranges
        state = orange;
      }
      break;
    }

    case blue:
        bleu();
        state = reading;
        break;

    case pink:
    case cyan:
    case orange:
    case yellow:
      state = reading;
      break;
  }
}

void bleu() {
  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);            // stop robot
  Serial.println("Blue: waiting for IR...");

  while (true) {                        
    int signal = receiveIR();
    if (signal == BLUE_RESUME_SIGNAL) {
      Serial.println("Resume signal received");
      return;
    }
  }
}