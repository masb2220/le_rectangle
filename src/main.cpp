#include <Wire.h>
#include "Adafruit_TCS34725.h"

/* Example code for the Adafruit TCS34725 breakout library */

/* Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground */
   
/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific int time and gain values */
enum State {
  reading,
  pink,
  blue,
  cyan,
  orange,
  yellow
};
State state = reading;
int threshold = 600;

void colorReaction();
int position_robot(int threshold);

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

void setup(void) {
  Serial.begin(9600);
  pinMode(45, OUTPUT);
  pinMode(43, OUTPUT);
  pinMode(41, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(0, INPUT);
  pinMode(1, INPUT);
  pinMode(2, INPUT);

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  digitalWrite(41, HIGH);
  digitalWrite(42, HIGH);
  digitalWrite(45, HIGH);
  digitalWrite(41, HIGH);
  digitalWrite(45, HIGH);
}

void loop(void) {
  colorReaction();
}

void colorReaction(){
switch(state){
    case reading:
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
    if (r >= 800 && g >= 650 && g <= 950 && b >= 400 && b <= 800){
      Serial.println("pink detected");
      state = pink;
    }
    if (r >= 500 && r <= 800 && g >= 900 && b >= 600 && b <= 1000){
      Serial.println("cyan detected");
      state = cyan;
    }
    if (r >= 900 && r <= 1400 && g >= 900 && g <= 1400 && b >= 500 && b <= 800){
      Serial.println("yellow detected");
      state = yellow;
    }
    if (r >= 200 && r <= 600 && g >= 600 && g <=1100 && b >= 750){
      Serial.println("blue detected");
      state = blue;
    }
    if (r >= 200 && r <= 600 && g >= 600 && g <=1100 && b >= 750){
      Serial.println("blue detected");
      state = orange;
    }
    break;

    case pink:
    state = reading;
    break;

    case blue:
    state = reading;
    break;

    case cyan:
    state = reading;
    break;

    case orange:
    state = reading;
    break;

    case yellow:
    state = reading;
    break;
  }
}

int position_robot(int threshold){
  int intensity_gauche = analogRead(0);
  int intensity_milieu = analogRead(1);
  int intensity_droite = analogRead(2);

  if (intensity_milieu >= threshold){
    return 0;
  }
  else if (intensity_droite >= threshold){
    return 1;
  }
  else if (intensity_gauche >= threshold) {
    return 2;
  }

}