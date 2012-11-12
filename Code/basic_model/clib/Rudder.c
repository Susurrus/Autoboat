#include <stdbool.h>

#include "Rudder.h"
#include "EcanFunctions.h"
#include "Nmea2000Encode.h"
#include "CanMessages.h"

struct RudderData rudderSensorData;

float GetRudderAngle(void )
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

void ClearRudderAngle(const uint8_t data[2])
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

void RudderSendAngleCommand(float angleCommand)
{
	// Set CAN header information.
	tCanMessage msg;
	PackagePgn127245(&msg, 9, 0, CAN_INV_DATA, angleCommand, CAN_INV_DATA);

	// And finally transmit it.
	ecan1_buffered_transmit(&msg);
}