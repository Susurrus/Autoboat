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

void CanMessageDecodeStatus(const tCanMessage *msg, uint8_t *nodeId, uint16_t *statusBitfield, uint16_t *errorBitfield)
{
	if (nodeId) {
		*nodeId = msg->payload[0];
	}

	if (statusBitfield) {
		*statusBitfield = ((uint16_t)msg->payload[1]) || (((uint16_t)msg->payload[2]) << 8);
	}

	if (errorBitfield) {
		*errorBitfield = ((uint16_t)msg->payload[3]) || (((uint16_t)msg->payload[4]) << 8);
	}
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
	msg->payload[0] = calibrate?0x01:0x00;
	msg->payload[0] |= reset?0x02:0x00;
	msg->payload[0] |= enable?0x04:0x00;
}

void CanMessageDecodeRudderSetState(const tCanMessage *msg, bool *enable, bool *reset, bool *calibrate)
{
    if (calibrate) {
        *calibrate = msg->payload[0] & 0x01;
    }
    if (reset) {
        *reset = msg->payload[0] & 0x02;
    }
    if (enable) {
        *enable = msg->payload[0] & 0x04;
    }
}

void CanMessageDecodeRudderSetTxRate(const tCanMessage *msg, uint16_t *angleRate, uint16_t *statusRate)
{
    if (angleRate) {
        *angleRate = msg->payload[0];
    }
    if (statusRate) {
        *statusRate = msg->payload[1];
    }
}

void CanMessagePackageRudderDetails(tCanMessage *msg, uint16_t potVal, uint16_t portLimitVal, uint16_t sbLimitVal, bool portLimitTrig, bool sbLimitTrig, bool enabled, bool calibrated, bool calibrating)
{
    msg->id = CAN_MSG_ID_RUDDER_DETAILS;
    msg->buffer = 0;
    msg->message_type = CAN_MSG_DATA;
    msg->frame_type = CAN_FRAME_STD;
    msg->validBytes = CAN_MSG_SIZE_RUDDER_DETAILS;

    // Now fill in the data.
    msg->payload[0] = potVal;
    msg->payload[1] = potVal >> 8;
    msg->payload[2] = portLimitVal;
    msg->payload[3] = portLimitVal >> 8;
    msg->payload[4] = sbLimitVal;
    msg->payload[5] = sbLimitVal >> 8;
    msg->payload[6] = portLimitTrig << 7;
    msg->payload[6] |= sbLimitTrig << 5;
    msg->payload[6] |= enabled;
    if (calibrated) {
        msg->payload[6] |= 0x02;
    }
    if (calibrating) {
        msg->payload[6] |= 0x04;
    }
}

void CanMessageDecodeRudderDetails(const tCanMessage *msg, uint16_t *potVal, uint16_t *portLimitVal, uint16_t *sbLimitVal, bool *portLimitTrig, bool *sbLimitTrig, bool *enabled, bool *calibrated, bool *calibrating)
{
    if (potVal) {
        *potVal = ((uint16_t)msg->payload[0]) | (((uint16_t)msg->payload[1]) << 8);
    }
    if (portLimitVal) {
        *portLimitVal = ((uint16_t)msg->payload[2]) | (((uint16_t)msg->payload[3]) << 8);
    }
    if (sbLimitVal) {
        *sbLimitVal = ((uint16_t)msg->payload[4]) | (((uint16_t)msg->payload[5]) << 8);
    }

    if (portLimitTrig) {
        *portLimitTrig = (msg->payload[6] >> 7) & 1;
    }
    if (sbLimitTrig) {
        *sbLimitTrig = (msg->payload[6] >> 5) & 1;
    }

    if (enabled) {
        *enabled = msg->payload[6] & 1;
    }
    if (calibrated) {
        *calibrated = (msg->payload[6] >> 1) & 1;
    }
    if (calibrating) {
        *calibrating = (msg->payload[6] >> 2) & 1;
    }
}

void CanMessagePackageImuData(tCanMessage *msg, float direction, float pitch, float roll)
{
    msg->id = CAN_MSG_ID_IMU_DATA;
    msg->buffer = 0;
    msg->message_type = CAN_MSG_DATA;
    msg->frame_type = CAN_FRAME_STD;
    msg->validBytes = CAN_MSG_SIZE_IMU_DATA;

    // Now fill in the data.
    int16_t intDirection = (int16_t)(direction * 8192.0);
    msg->payload[0] = (uint8_t)intDirection;
    msg->payload[1] = ((uint16_t)intDirection) >> 8;
    int16_t intPitch = (int16_t)(pitch * 8192.0);
    msg->payload[2] = (uint8_t)intPitch;
    msg->payload[3] = ((uint16_t)intPitch) >> 8;
    int16_t intRoll = (int16_t)(roll * 8192.0);
    msg->payload[4] = (uint8_t)intRoll;
    msg->payload[5] = ((uint16_t)intRoll) >> 8;
}

void CanMessageDecodeImuData(const tCanMessage *msg, float *direction, float *pitch, float *roll)
{
    if (direction) {
        int16_t tmp = (int16_t)(((uint16_t)msg->payload[0]) | (((uint16_t)msg->payload[1]) << 8));
        *direction = (float)tmp / 8192.0;
    }
    if (pitch) {
        int16_t tmp = (int16_t)(((uint16_t)msg->payload[2]) | (((uint16_t)msg->payload[3]) << 8));
        *pitch = (float)tmp / 8192.0;
    }
    if (roll) {
        int16_t tmp = (int16_t)(((uint16_t)msg->payload[4]) | (((uint16_t)msg->payload[5]) << 8));
        *roll = (float)tmp / 8192.0;
    }
}
