#ifndef _RUDDER_H_
#define _RUDDER_H_

#include <stdbool.h>

#include "types.h"

extern tFloatToChar rudderAngle;

/**
 * Returns the current vessel rudder angle. Units are in radians with positive values to port.
 */
float GetRudderAngle();

/**
 * Stores a recorded value of the rudder angle.
 */
void SetRudderData(const uint8_t data[2]);

/**
 * Clears the stored rudder data to all zeros.
 */
void ClearRudderAngle();

#endif // _RUDDER_H_
