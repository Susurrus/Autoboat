#include "CanMessages.h"
#include "EcanDefines.h"
#include "Packing.h"

void CanMessagePackageStatus(CanMessage *msg, uint8_t nodeId, uint8_t cpuLoad, int8_t temp, uint8_t voltage, uint16_t status, uint16_t errors)
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
	// And the CPU load
	msg->payload[1] = cpuLoad;
	// The onboard temp sensor readings. Units are in degrees celsius
	msg->payload[2] = temp;
	// The node input voltage
	msg->payload[3] = voltage;
	// The status bitfield
	LEPackUint16(&msg->payload[4], status);
	// And the error bitfield
	LEPackUint16(&msg->payload[6], errors);
}

void CanMessageDecodeStatus(const CanMessage *msg, uint8_t *nodeId, uint8_t *cpuLoad, int8_t *temp, uint8_t *voltage, uint16_t *status, uint16_t *errors)
{
	if (nodeId) {
		*nodeId = msg->payload[0];
	}

	if (cpuLoad) {
		*cpuLoad = msg->payload[1];
	}

	if (temp) {
		*temp = (int8_t)msg->payload[2];
	}

	if (voltage) {
		*voltage = msg->payload[3];
	}

	if (status) {
		LEUnpackUint16(status, &msg->payload[4]);
	}

	if (errors) {
		LEUnpackUint16(errors, &msg->payload[6]);
	}
}

void CanMessagePackageRudderSetState(CanMessage *msg, bool enable, bool reset, bool calibrate)
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

void CanMessageDecodeRudderSetState(const CanMessage *msg, bool *enable, bool *reset, bool *calibrate)
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

void CanMessageDecodeRudderSetTxRate(const CanMessage *msg, uint16_t *angleRate, uint16_t *statusRate)
{
    if (angleRate) {
        *angleRate = msg->payload[0];
    }
    if (statusRate) {
        *statusRate = msg->payload[1];
    }
}

void CanMessagePackageRudderDetails(CanMessage *msg, uint16_t potVal, uint16_t portLimitVal, uint16_t sbLimitVal, bool portLimitTrig, bool sbLimitTrig, bool enabled, bool calibrated, bool calibrating)
{
    msg->id = CAN_MSG_ID_RUDDER_DETAILS;
    msg->buffer = 0;
    msg->message_type = CAN_MSG_DATA;
    msg->frame_type = CAN_FRAME_STD;
    msg->validBytes = CAN_MSG_SIZE_RUDDER_DETAILS;

    // Now fill in the data.
	LEPackUint16(&msg->payload[0], potVal);
	LEPackUint16(&msg->payload[2], portLimitVal);
	LEPackUint16(&msg->payload[4], sbLimitVal);
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

void CanMessageDecodeRudderDetails(const CanMessage *msg, uint16_t *potVal, uint16_t *portLimitVal, uint16_t *sbLimitVal, bool *portLimitTrig, bool *sbLimitTrig, bool *enabled, bool *calibrated, bool *calibrating)
{
    if (potVal) {
		LEUnpackUint16(potVal, &msg->payload[0]);
    }
    if (portLimitVal) {
		LEUnpackUint16(portLimitVal, &msg->payload[2]);
    }
    if (sbLimitVal) {
		LEUnpackUint16(sbLimitVal, &msg->payload[4]);
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

void CanMessagePackageImuData(CanMessage *msg, float direction, float pitch, float roll)
{
    msg->id = CAN_MSG_ID_IMU_DATA;
    msg->buffer = 0;
    msg->message_type = CAN_MSG_DATA;
    msg->frame_type = CAN_FRAME_STD;
    msg->validBytes = CAN_MSG_SIZE_IMU_DATA;

    // Now fill in the data.
    int16_t intDirection = (int16_t)(direction * 8192.0);
	BEPackInt16(&msg->payload[0], intDirection);
    int16_t intPitch = (int16_t)(pitch * 8192.0);
	BEPackInt16(&msg->payload[2], intPitch);
    int16_t intRoll = (int16_t)(roll * 8192.0);
	BEPackInt16(&msg->payload[4], intRoll);
}

void CanMessageDecodeImuData(const CanMessage *msg, float *direction, float *pitch, float *roll)
{
	int16_t tmp;
    if (direction) {
		BEUnpackInt16(&tmp, &msg->payload[0]);
        *direction = (float)tmp / 8192.0;
    }
    if (pitch) {
		BEUnpackInt16(&tmp, &msg->payload[2]);
        *pitch = (float)tmp / 8192.0;
    }
    if (roll) {
		BEUnpackInt16(&tmp, &msg->payload[4]);
        *roll = (float)tmp / 8192.0;
    }
}

void CanMessagePackageGyroData(CanMessage *msg, float zRate)
{
    msg->id = CAN_MSG_ID_GYRO_DATA;
    msg->buffer = 0;
    msg->message_type = CAN_MSG_DATA;
    msg->frame_type = CAN_FRAME_STD;
    msg->validBytes = CAN_MSG_SIZE_GYRO_DATA;

    // Now fill in the data.
    uint32_t intZRate = (uint32_t)(zRate * 1e8);
	BEPackInt16(&msg->payload[0], intZRate);
}

void CanMessageDecodeGyroData(const CanMessage *msg, float *zRate)
{
    if (zRate) {
		uint32_t tmp;
		LEUnpackUint32(&tmp, &msg->payload[0]);
        *zRate = (float)tmp / 1e8;
    }
}
