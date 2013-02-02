#include "Nmea2000.h"
#include "Nmea2000Encode.h"
#include "Packing.h"
#include "EcanDefines.h"

#include <math.h>

void PackagePgn127245(CanMessage *msg, uint8_t sourceDevice, uint8_t instance, uint8_t dirOrder, float angleOrder, float position)
{
	/// Set CAN header information.
	msg->id = Iso11783Encode(PGN_RUDDER, sourceDevice, 0xFF, 2); // Leave the priority hardcoded to 2.
	msg->buffer = 0; // NOTE: This needs to be changed to the appropriate buffer before transmission.
	msg->message_type = CAN_MSG_DATA;
	msg->frame_type = CAN_FRAME_EXT;
	msg->validBytes = 6;

	/// Now fill in the data.
	msg->payload[0] = instance;
	msg->payload[1] = 0xFC | (0x3 & dirOrder);

	// Convert commanded rudder angle to 1e-4 radians
	int16_t angle;
	if (angleOrder == angleOrder) {
		angle = (int16_t)(angleOrder * 10000);
	} else {
		angle = 0x7FFF;
	}
	LEPackInt16(&msg->payload[2], angle);

	// Convert current rudder angle to 1e-4 radians
    // The following is a test to see if position is NAN
	if (position == position) {
		angle = (int16_t)(position * 10000);
	} else {
		angle = 0x7FFF;
	}
	LEPackInt16(&msg->payload[4], angle);
}

void PackagePgn127508(CanMessage *msg, uint8_t sourceDevice, uint8_t battInstance, float voltage, float amperage, float temp, uint8_t sid)
{
	msg->id = Iso11783Encode(PGN_BATTERY_STATUS, sourceDevice, 0xFF, 3);
	msg->message_type = CAN_MSG_DATA;
	msg->frame_type = CAN_FRAME_EXT;
	msg->buffer = 0;
	msg->validBytes = 8;

	// Field 0: Battery instance
	msg->payload[0] = battInstance;

	// Field 1: Voltage (in .01V). Check that it's a valid voltage (basically NOT NaN).
	uint16_t x = 0xFFFF;
    // The following is a test to see if position is NAN
	if (voltage == voltage) {
		voltage *= 100.0f;
		x = (uint16_t)voltage;
	}
	LEPackUint16(&msg->payload[1], x);

	// Field 2: Current (in .1A)
	x = 0xFFFF;
    // The following is a test to see if position is NAN
	if (amperage == amperage) {
		amperage *= 10.0f;
		x = (uint16_t)amperage;
	}
	LEPackUint16(&msg->payload[3], x);

	// Field 3: Temperature (in 1K)
	// All 1s indicated no-measurement
	x = 0xFFFF;
    // The following is a test to see if position is NAN
	if (temp == temp) {
		temp = (temp + 273.15) * 100;
		x = (uint16_t)temp;
	}
	LEPackUint16(&msg->payload[5], x);

	// Field 4: Sequence ID
	msg->payload[7] = sid;
}

void PackagePgn129025(CanMessage *msg, uint8_t sourceDevice, int32_t latitude, int32_t longitude)
{
    // Specify a new CAN message w/ metadata
    msg->id = Iso11783Encode(PGN_POSITION_RAP_UPD, sourceDevice, 0xFF, 2);
    msg->buffer = 0;
    msg->message_type = CAN_MSG_DATA;
    msg->frame_type = CAN_FRAME_EXT;
    msg->validBytes = 8;

	LEPackInt32(&msg->payload[0], latitude);
	LEPackInt32(&msg->payload[4], longitude);
}

void PackagePgn129026(CanMessage *msg, uint8_t sourceDevice, uint8_t seqId, uint8_t cogRef, uint16_t cog, uint16_t sog)
{
    // Specify a new CAN message w/ metadata
    msg->id = Iso11783Encode(PGN_COG_SOG_RAP_UPD, sourceDevice, 0xFF, 2);
    msg->buffer = 0;
    msg->message_type = CAN_MSG_DATA;
    msg->frame_type = CAN_FRAME_EXT;
    msg->validBytes = 8;

    msg->payload[0] = seqId;
	msg->payload[1] = cogRef & 0x03;
	LEPackUint16(&msg->payload[2], cog);
	LEPackUint16(&msg->payload[4], sog);
}

void PackagePgn129539(CanMessage *msg, uint8_t sourceDevice, uint8_t seqId, uint8_t desiredMode, uint8_t actualMode, int16_t hdop, int16_t vdop, int16_t tdop)
{
    // Specify a new CAN message w/ metadata
    msg->id = Iso11783Encode(PGN_GNSS_DOPS, sourceDevice, 0xFF, 2);
    msg->buffer = 0;
    msg->message_type = CAN_MSG_DATA;
    msg->frame_type = CAN_FRAME_EXT;
    msg->validBytes = 8;

    msg->payload[0] = seqId;
	msg->payload[1] = 0xC0;
	msg->payload[1] |= desiredMode & 0x07;
	msg->payload[1] |= (actualMode & 0x07) << 3;
	LEPackInt16(&msg->payload[2], hdop);
	LEPackInt16(&msg->payload[4], vdop);
	LEPackInt16(&msg->payload[6], tdop);
}

void PackagePgn130311(CanMessage *msg, uint8_t sourceDevice, uint8_t sid, uint8_t tempInst, uint8_t humidInst, float temp, float humid, float press)
{
    // Specify a new CAN message w/ metadata
    msg->id = Iso11783Encode(PGN_ENV_PARAMETERS2, sourceDevice, 0xFF, 2);
    msg->buffer = 0;
    msg->message_type = CAN_MSG_DATA;
    msg->frame_type = CAN_FRAME_EXT;
    msg->validBytes = 8;

    // Now set the data.
    msg->payload[0] = sid;      // SID
    msg->payload[1] = tempInst & 0x03;           // Temp instance
    msg->payload[1] |= (humidInst & 0x03F) << 6; // Humidity instance
    // Convert temperature from Celius to units of .01Kelvin.
    // The following is a test to see if position is NAN
    uint16_t tempConverted = (temp == temp)?(uint16_t)((temp + 273.15) * 100):0xFFFF;
	LEPackUint16(&msg->payload[2], tempConverted);
    // Convert humidity from % to 0.004 %.
    // The following is a test to see if position is NAN
    uint16_t humidConverted = (humid == humid)?(uint16_t)(humid * 250):0xFFFF;
	LEPackUint16(&msg->payload[4], humidConverted);
    // Convert pressure from kPa and record as hPa.
    // The following is a test to see if position is NAN
    uint16_t pressConverted = (press == press)?(uint16_t)(press * 10):0xFFFF;
	LEPackUint16(&msg->payload[6], pressConverted);
}