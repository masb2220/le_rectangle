#include "PathFinding/obstacle_sensor.h"
#include <Arduino.h>
void obstacle_sensor_init(const obstacle_sensor_pins &pins) {
    pinMode(pins.left, INPUT);
    pinMode(pins.right, INPUT);
}
void obstacle_sensor_read(const obstacle_sensor_pins &pins, bool &left, bool &right) {
    left = (digitalRead(pins.left) == LOW) == pins.active_low;
    right = (digitalRead(pins.right) == LOW) == pins.active_low;
}
