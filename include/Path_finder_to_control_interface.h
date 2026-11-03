#ifndef PATH_FINDER_TO_CONTROL_INTERFACE
#define PATH_FINDER_TO_CONTROL_INTERFACE

#include "robot_control.h"


bool TURN_LEFT(float angle);
bool TURN_RIGHT(float angle);
bool FORWARD(float distance);
bool BACKWARD(float distance);
#endif
