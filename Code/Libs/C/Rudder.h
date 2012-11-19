#ifndef RUDDER_H
#define RUDDER_H

#include <stdbool.h>

#include "Types.h"

struct RudderData {
	tFloatToChar RudderAngle;
	tUnsignedShortToChar RudderPotValue;
	tUnsignedShortToChar RudderPotLimitStarboard;
	tUnsignedShortToChar RudderPotLimitPort;
	bool LimitHitStarboard;
	bool LimitHitPort;
        bool Enabled;
        bool Calibrated;
        bool Calibrating;
};
extern struct RudderData rudderSensorData;

/**
 * Returns the current vessel rudder angle. Units are in radians with positive values to port.
 */
float GetRudderAngle(void);

/**
 * Returns the rudder state as a 3-bit number following
 * from msg 0x8081 used with the rudder.
 */
uint8_t GetRudderStatus(void);

/**
 * Stores a recorded value of the rudder angle.
 */
void SetRudderAngle(const uint8_t data[2]);

/**
 * Clears the stored rudder data to all zeros.
 */
void ClearRudderAngle(void);

void RudderStartCalibration(void);

void RudderSendAngleCommand(uint8_t sourceNode, float angleCommand);

#endif // RUDDER_H
