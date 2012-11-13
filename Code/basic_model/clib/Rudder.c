#include <stdbool.h>

#include "Rudder.h"
#include "EcanFunctions.h"
#include "Nmea2000Encode.h"
#include "CanMessages.h"

struct RudderData rudderSensorData;

float GetRudderAngle(void)
{
	return rudderSensorData.RudderAngle.flData;
}

uint8_t GetRudderStatus(void)
{
	return rudderSensorData.RudderState;
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
	tCanMessage msg;

	CanMessagePackageRudderSetState(&msg, true, false, true);

	// And finally transmit it.
	ecan1_buffered_transmit(&msg);
}

void RudderSendAngleCommand(uint8_t sourceNode, float angleCommand)
{
	// Set CAN header information.
	tCanMessage msg;
	PackagePgn127245(&msg, sourceNode, 0xFF, 0x3, angleCommand, -180);

	// And finally transmit it.
	ecan1_buffered_transmit(&msg);
}