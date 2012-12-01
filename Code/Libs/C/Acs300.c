#include "Acs300.h"

void Acs300PackageVelocityCommand(tCanMessage *msg, int16_t torqueFeedForward, int16_t velCommand, uint16_t status)
{
    msg->id = ACS300_CAN_ID_VEL_CMD;
	msg->buffer = 0; // NOTE: This needs to be changed to the appropriate buffer before transmission.
    msg->validBytes = 6;
    msg->frame_type = CAN_FRAME_STD;
    msg->message_type = CAN_MSG_DATA;

	msg->payload[0] = torqueFeedForward >> 8;
	msg->payload[1] = (uint8_t)torqueFeedForward;
	msg->payload[2] = velCommand >> 8;
	msg->payload[3] = (uint8_t)velCommand;
	msg->payload[4] = status >> 8;
	msg->payload[5] = (uint8_t)status;
}

void Acs300DecodeVelocityCommand(const uint8_t data[6], int16_t *torqueFeedForward, int16_t *velCommand, uint16_t *status)
{
	if (torqueFeedForward) {
		*torqueFeedForward = data[1];
		*torqueFeedForward |= ((uint16_t)data[0]) << 8;
	}
	if (velCommand) {
		*velCommand = (uint16_t)data[3];
		*velCommand |= ((uint16_t)data[2]) << 8;
	}
	if (status) {
		*status = (uint16_t)data[5];
		*status |= ((uint16_t)data[4]) << 8;
	}
}

void Acs300PackageWriteParam(tCanMessage *msg, uint16_t address, uint16_t value)
{
    msg->id = ACS300_CAN_ID_WR_PARAM;
	msg->buffer = 0; // NOTE: This needs to be changed to the appropriate buffer before transmission.
    msg->validBytes = 4;
    msg->frame_type = CAN_FRAME_STD;
    msg->message_type = CAN_MSG_DATA;

    msg->payload[0] = (uint8_t)(address >> 8);
    msg->payload[1] = (uint8_t)address;
    msg->payload[2] = (uint8_t)(value >> 8);
    msg->payload[3] = (uint8_t)value;
}

void Acs300DecodeWriteParam(const uint8_t data[4], uint16_t *address, uint16_t *value)
{
	if (address) {
		*address = (uint16_t)data[1];
		*address |= ((uint16_t)data[0]) << 8;
	}
	if (*value) {
		*value = (uint16_t)data[3];
		*value |= ((uint16_t)data[2]) << 8;
	}
}

void Acs300PackageHeartbeat(tCanMessage *msg, uint16_t dataA, uint16_t dataB, uint16_t voltage, uint16_t errorStatus)
{
    msg->id = ACS300_CAN_ID_HRTBT;
	msg->buffer = 0; // NOTE: This needs to be changed to the appropriate buffer before transmission.
    msg->validBytes = 8;
    msg->frame_type = CAN_FRAME_STD;
    msg->message_type = CAN_MSG_DATA;

    msg->payload[0] = (uint8_t)(dataA >> 8);
    msg->payload[1] = (uint8_t)dataA;
    msg->payload[2] = (uint8_t)(dataB >> 8);
    msg->payload[3] = (uint8_t)dataB;
    msg->payload[4] = (uint8_t)(voltage >> 8);
    msg->payload[5] = (uint8_t)voltage;
    msg->payload[6] = (uint8_t)(errorStatus >> 8);
    msg->payload[7] = (uint8_t)errorStatus;
}

void Acs300DecodeHeartbeat(const uint8_t data[8], uint16_t *dataA, uint16_t *dataB, uint16_t *voltage, uint16_t *errorStatus)
{
	if (dataA) {
		*dataA = (uint16_t)data[1];
		*dataA |= ((uint16_t)data[0]) << 8;
	}
	if (*dataB) {
		*dataB = (uint16_t)data[3];
		*dataB |= ((uint16_t)data[2]) << 8;
	}
	if (voltage) {
		*voltage = (uint16_t)data[5];
		*voltage |= ((uint16_t)data[4]) << 8;
	}
	if (*errorStatus) {
		*errorStatus = (uint16_t)data[7];
		*errorStatus |= ((uint16_t)data[6]) << 8;
	}
}
