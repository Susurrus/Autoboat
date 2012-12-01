#include "Can.h"
#include "ecanDefinitions.h"
#include "ecanFunctions.h"
#include "Nmea2000.h"
#include "Nmea2000Encode.h"
#include "CanMessages.h"
#include "Node.h"
#include "Hil.h"
#include "MessageScheduler.h"
#include "Acs300.h"

// Declare some constants for use with the message scheduler
// (don't use PGN or message ID as it must be a uint8)
enum {
    // Rudder messages
    SCHED_ID_RUDDER_ANGLE = 1,
    SCHED_ID_RUDDER_LIMITS, // For status bits

    // Throttle messages
    SCHED_ID_THROTTLE_STATUS,

    // RC node status messages
    SCHED_ID_RC_STATUS,

    // GPS messages
    SCHED_ID_LAT_LON,
    SCHED_ID_COG_SOG,

    // IMU messages
    SCHED_ID_ATT_ANGLE,

    // DST800 messages
    SCHED_ID_WATER_SPD
};

// Set up the message scheduler's various data structures.
#define ECAN_MSGS_SIZE 8
static uint8_t ids[ECAN_MSGS_SIZE] = {
    SCHED_ID_RUDDER_ANGLE,
    SCHED_ID_RUDDER_LIMITS,
    SCHED_ID_THROTTLE_STATUS,
    SCHED_ID_RC_STATUS,
    SCHED_ID_LAT_LON,
    SCHED_ID_COG_SOG,
    SCHED_ID_ATT_ANGLE,
    SCHED_ID_WATER_SPD
};
static uint16_t tsteps[ECAN_MSGS_SIZE][2][8] = {};
static uint8_t  mSizes[ECAN_MSGS_SIZE];
static MessageSchedule sched = {
	ECAN_MSGS_SIZE,
	ids,
	mSizes,
	0,
	tsteps
};

void CanEnableMessages(void)
{
	// Transmit the rudder angle at 10Hz
	if (!AddMessageRepeating(&sched, SCHED_ID_RUDDER_ANGLE, 10)) {
		while (1);
	}

	// Transmit the rudder status at 10Hz
	if (!AddMessageRepeating(&sched, SCHED_ID_RUDDER_LIMITS, 10)) {
		while (1);
	}

	// Transmit the throttle status at 100Hz
	if (!AddMessageRepeating(&sched, SCHED_ID_THROTTLE_STATUS, 100)) {
		while (1);
	}

	// Transmit the throttle status at 2Hz
	if (!AddMessageRepeating(&sched, SCHED_ID_RC_STATUS, 2)) {
		while (1);
	}

	// Transmit latitude/longitude at 5Hz
	if (!AddMessageRepeating(&sched, SCHED_ID_LAT_LON, 5)) {
		while (1);
	}

	// Transmit heading & speed at 4Hz
	if (!AddMessageRepeating(&sched, SCHED_ID_COG_SOG, 4)) {
		while (1);
	}
}

void CanDisableMessages(void)
{
    ClearSchedule(&sched);
}

uint8_t CanReceiveMessages(void)
{
	uint8_t messagesLeft = 0;
	tCanMessage msg;
	uint32_t pgn;

	uint8_t messagesHandled = 0;

	do {
		int foundOne = ecan1_receive(&msg, &messagesLeft);
		if (foundOne) {
			// Process throttle messages here. Anything not explicitly handled is assumed to be a NMEA2000 message.
			if (msg.id == ACS300_CAN_ID_WR_PARAM) { // From the ACS300
                uint16_t address, data;
                Acs300DecodeWriteParam(msg.payload, &address, &data);
                if (address == ACS300_PARAM_CC) {
                    hilDataToTransmit.data.tCommandSpeed = (float)(int16_t)data;
                }
			} else {
				pgn = Iso11783Decode(msg.id, NULL, NULL, NULL);
				switch (pgn) {
                // Decode the commanded rudder angle from the PGN127245 messages.
				case PGN_RUDDER: {
					ParsePgn127245(msg.payload, NULL, NULL, NULL, &hilDataToTransmit.data.rCommandAngle, NULL);
				} break;
				}
			}

			++messagesHandled;
		}
	} while (messagesLeft > 0);

	return messagesHandled;
}

void CanTransmitMessages(void)
{
    // Track the last HIL status
    static bool hilStatus = false;

    // Enable or disable CAN messages based on whether HIL is active or not.
    if (!hilStatus && HilActive()) {
        CanEnableMessages();
        hilStatus = true;
    } else if (hilStatus && !HilActive()) {
        CanDisableMessages();
        hilStatus = false;
    }
    
    // Track the messages to be transmit for this timestep.
    static uint8_t msgs[ECAN_MSGS_SIZE];

    uint8_t messagesToSend = GetMessagesForTimestep(&sched, msgs);
    int i;
    tCanMessage msg;
    for (i = 0; i < messagesToSend; ++i) {
        switch (msgs[i]) {
            case SCHED_ID_RC_STATUS:
                CanMessagePackageStatus(&msg, CAN_NODE_RC, 0, 0);
                ecan1_buffered_transmit(&msg);
            break;
            case SCHED_ID_RUDDER_ANGLE:
                PackagePgn127245(&msg, nodeId, 0xFF, 0xF, NAN, hilReceivedData.data.rAngle);
                ecan1_buffered_transmit(&msg);
            break;
            case SCHED_ID_RUDDER_LIMITS:
                CanMessagePackageRudderDetails(&msg, 0, 0, 0, false, false, true, true, false);
                ecan1_buffered_transmit(&msg);
            break;
            case SCHED_ID_THROTTLE_STATUS:
                Acs300PackageHeartbeat(&msg, (uint16_t)hilReceivedData.data.tSpeed, 0, 0, 0);
                ecan1_buffered_transmit(&msg);
            break;
            case SCHED_ID_LAT_LON:
                PackagePgn129025(&msg, nodeId, hilReceivedData.data.gpsLatitude, hilReceivedData.data.gpsLongitude);
                ecan1_buffered_transmit(&msg);
            break;
            case SCHED_ID_COG_SOG:
                PackagePgn129026(&msg, nodeId, 0xFF, 0x7, hilReceivedData.data.gpsCog, hilReceivedData.data.gpsSog);
                ecan1_buffered_transmit(&msg);
            break;
        }
    }
}