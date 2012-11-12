#include "CanMessages.h"
#include "ecanFunctions.h"

void CanMessagePackageStatus(tCanMessage *msg, uint8_t nodeId, uint16_t statusBitfield, uint16_t errorBitfield)
{
	// Set CAN header information.
	msg->buffer = 0; // Dependent on ECAN configuration.
	msg->message_type = CAN_MSG_DATA;
	msg->frame_type = CAN_FRAME_STD;
	
	// Set message-specific stuff
	msg->id = CAN_MSG_ID_STATUS;
	msg->validBytes = CAN_MSG_SIZE_STATUS;

	// Now fill in the three fields:
	// The node ID
	msg->payload[0] = nodeId;
	// The status bitfield
	msg->payload[1] = (uint8_t)statusBitfield;
	msg->payload[2] = (uint8_t)(statusBitfield >> 8);
	// And finally the error bitfield
	msg->payload[3] = (uint8_t)errorBitfield;
	msg->payload[4] = (uint8_t)(errorBitfield >> 8);
}

void CanMessagePackageRudderSetState(tCanMessage *msg, bool enable, bool reset, bool calibrate)
{ 
	// Set CAN header information.
	msg->id = CAN_MSG_ID_RUDDER_SET_STATE;
	msg->buffer = 0;
	msg->message_type = CAN_MSG_DATA;
	msg->frame_type = CAN_FRAME_STD;
	msg->validBytes = CAN_MSG_SIZE_RUDDER_SET_STATE;

	// Now fill in the data.
	msg->payload[0] = 0;
	msg->payload[0] |= calibrate?0x01:0x00;
	msg->payload[0] |= reset?0x02:0x00;
	msg->payload[0] |= enable?0x04:0x00;
}