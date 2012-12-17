#include <stdbool.h>

#include "Rudder.h"
#include "Ecan1.h"
#include "Nmea2000Encode.h"
#include "CanMessages.h"

struct RudderData rudderSensorData;

float GetRudderAngle(void)
{
	return rudderSensorData.RudderAngle.flData;
}

uint8_t GetRudderStatus(void)
{
	return rudderSensorData.Enabled |
               (rudderSensorData.Calibrated << 1) |
               (rudderSensorData.Calibrating << 2);
}

void SetRudderAngle(const uint8_t data[2])
{
	rudderSensorData.RudderAngle.chData[0] = data[0];
	rudderSensorData.RudderAngle.chData[1] = data[1];
}

void ClearRudderAngle(void)
{
	rudderSensorData.RudderAngle.flData = 0.0;
}

void RudderStartCalibration(void)
{
	// Set CAN header information.
	CanMessage msg;

	CanMessagePackageRudderSetState(&msg, true, false, true);

	// And finally transmit it.
	Ecan1Transmit(&msg);
}

void RudderSendAngleCommand(uint8_t sourceNode, float angleCommand)
{
	// Set CAN header information.
	CanMessage msg;
	PackagePgn127245(&msg, sourceNode, 0xFF, 0x3, angleCommand, NAN);

	// And finally transmit it.
	Ecan1Transmit(&msg);
}