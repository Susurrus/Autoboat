#include "Acs300.h"
#include "Packing.h"

void Acs300SendThrottleCommand(int16_t command)
{
	CanMessage msg;

	// If the commanded current is non-zero, make sure we set the ACS300 into run mode.
	if (command != 0) {
		Acs300PackageVelocityCommand(&msg, 0, 0, ACS300_COMMAND_RUN);
		Ecan1Transmit(&msg);
	}

	Acs300PackageWriteParam(&msg, ACS300_PARAM_CC, command);
	Ecan1Transmit(&msg);
}

void Acs300PackageVelocityCommand(CanMessage *msg, int16_t torqueFeedForward, int16_t velCommand, uint16_t status)
{
    msg->id = ACS300_CAN_ID_VEL_CMD;
	msg->buffer = 0; // NOTE: This needs to be changed to the appropriate buffer before transmission.
    msg->validBytes = 6;
    msg->frame_type = CAN_FRAME_STD;
    msg->message_type = CAN_MSG_DATA;

	BEPackInt16(&msg->payload[0], torqueFeedForward);
	BEPackInt16(&msg->payload[2], velCommand);
	BEPackUint16(&msg->payload[4], status);
}

void Acs300DecodeVelocityCommand(const uint8_t data[6], int16_t *torqueFeedForward, int16_t *velCommand, uint16_t *status)
{
	if (torqueFeedForward) {
		BEUnpackInt16(torqueFeedForward, &data[0]);
	}
	if (velCommand) {
		BEUnpackInt16(velCommand, &data[2]);
	}
	if (status) {
		BEUnpackUint16(status, &data[4]);
	}
}

void Acs300PackageWriteParam(CanMessage *msg, uint16_t address, uint16_t value)
{
    msg->id = ACS300_CAN_ID_WR_PARAM;
	msg->buffer = 0; // NOTE: This needs to be changed to the appropriate buffer before transmission.
    msg->validBytes = 4;
    msg->frame_type = CAN_FRAME_STD;
    msg->message_type = CAN_MSG_DATA;

	BEPackUint16(&msg->payload[0], address);
	BEPackUint16(&msg->payload[2], value);
}

void Acs300DecodeWriteParam(const uint8_t data[4], uint16_t *address, uint16_t *value)
{
	if (address) {
		BEUnpackUint16(address, &data[0]);
	}
	if (value) {
		BEUnpackUint16(value, &data[2]);
	}
}

void Acs300PackageHeartbeat(CanMessage *msg, uint16_t dataA, uint16_t dataB, uint16_t voltage, uint16_t errorStatus)
{
    msg->id = ACS300_CAN_ID_HRTBT;
	msg->buffer = 0; // NOTE: This needs to be changed to the appropriate buffer before transmission.
    msg->validBytes = 8;
    msg->frame_type = CAN_FRAME_STD;
    msg->message_type = CAN_MSG_DATA;

	BEPackUint16(&msg->payload[0], dataA);
	BEPackUint16(&msg->payload[2], dataB);
	BEPackUint16(&msg->payload[4], voltage);
	BEPackUint16(&msg->payload[6], errorStatus);
}

void Acs300DecodeHeartbeat(const uint8_t data[8], uint16_t *dataA, uint16_t *dataB, uint16_t *voltage, uint16_t *errorStatus)
{
	if (dataA) {
		BEUnpackUint16(dataA, &data[0]);
	}
	if (dataB) {
		BEUnpackUint16(dataB, &data[2]);
	}
	if (voltage) {
		BEUnpackUint16(voltage, &data[4]);
	}
	if (errorStatus) {
		BEUnpackUint16(errorStatus, &data[6]);
	}
}
