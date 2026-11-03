/** @file obstacle_sensor.h
 * @brief Brochage et lecture des deux sorties IR; aucune dependance au controle du robot.
 */
#ifndef OBSTACLE_SENSOR_H
#define OBSTACLE_SENSOR_H
#include <stdint.h>
struct obstacle_sensor_pins {
    uint8_t left = 49;
    uint8_t right = 48;
    bool active_low = true;
};
void obstacle_sensor_init(const obstacle_sensor_pins &pins);
void obstacle_sensor_read(const obstacle_sensor_pins &pins, bool &left, bool &right);
#endif
