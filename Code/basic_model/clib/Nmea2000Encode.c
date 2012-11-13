#include "nmea2000.h"
#include "Nmea2000Encode.h"
#include "ecanDefinitions.h"

void PackagePgn127245(tCanMessage *msg, uint8_t sourceDevice, uint8_t instance, uint8_t dirOrder, float angleOrder, float position)
{
	/// Set CAN header information.
	msg->id = Iso11783Encode(PGN_RUDDER, sourceDevice, CAN_INV_DATA, 2); // Leave the priority hardcoded to 2.
	msg->buffer = 0; // NOTE: This needs to be changed to the appropriate buffer before transmission.
	msg->message_type = CAN_MSG_DATA;
	msg->frame_type = CAN_FRAME_EXT;
	msg->validBytes = 6;

	/// Now fill in the data.
	msg->payload[0] = instance;
	msg->payload[1] = 0xFC | dirOrder;
	// Convert commanded rudder angle to 1e-4 radians
	int16_t angle;
        if (angleOrder > 90 || angleOrder < -90) {
            angle = 0xFFFF;
        } else {
            angle = (int16_t)(angleOrder * 10000);
        }
	// Send current angle over the CAN bus
	msg->payload[2] = angle;
	msg->payload[3] = angle >> 8;
	// Convert current rudder angle to 1e-4 radians
        if (position > 90 || position < -90) {
            angle = 0xFFFF;
        } else {
            angle = (int16_t)(position * 10000);
        }
	// Send current angle over the CAN bus
	msg->payload[4] = angle;
	msg->payload[5] = angle >> 8;
}