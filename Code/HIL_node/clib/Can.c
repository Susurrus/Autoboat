#include "Can.h"
#include "ecanDefinitions.h"
#include "ecanFunctions.h"
#include "Nmea2000.h"
#include "Nmea2000Encode.h"
#include "Types.h"
#include "Rudder.h"
#include "CanMessages.h"
#include "Node.h"
#include "Hil.h"
#include "MessageScheduler.h"

// Declare some constants for use with the message scheduler
// (don't use PGN or message ID as it must be a uint8)
enum {
    // Rudder messages
    SCHED_ID_RUDDER_ANGLE = 1,

    // Throttle messages
    SCHED_ID_THROTTLE_STATUS,

    // GPS messages
    SCHED_ID_LAT_LON,
    SCHED_ID_COG_SOG,

    // IMU messages
    SCHED_ID_ATT_ANGLE,

    // DST800 messages
    SCHED_ID_WATER_SPD
};

// Set up the message scheduler's various data structures.
#define ECAN_MSGS_SIZE 6
static uint8_t ids[ECAN_MSGS_SIZE] = {
    SCHED_ID_RUDDER_ANGLE,
    SCHED_ID_THROTTLE_STATUS,
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

struct RudderCanData rudderCanDataStore = {};
struct ThrottleData throttleDataStore = {};
struct GpsData gpsDataStore = {};
struct DateTimeData dateTimeDataStore = {};

void CanEnableMessages(void)
{
	// Transmit the rudder angle at 10Hz
	if (!AddMessageRepeating(&sched, SCHED_ID_RUDDER_ANGLE, 10)) {
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
			if (msg.id == 0x301) { // From the ACS300
				throttleDataStore.rpm.chData[0] = msg.payload[1];
				throttleDataStore.rpm.chData[1] = msg.payload[0];
				throttleDataStore.newData = true;
			} else if (msg.id == CAN_MSG_ID_STATUS) {
				uint8_t node;
				uint16_t status, errors;
				CanMessageDecodeStatus(&msg, &node, &status, &errors);
				if (node == CAN_NODE_RC) {
				}
			} else {
				pgn = Iso11783Decode(msg.id, NULL, NULL, NULL);
				switch (pgn) {
				case PGN_RUDDER: { // From the Rudder Controller
					if (ParsePgn127245(msg.payload, NULL, NULL, NULL, NULL, &rudderSensorData.RudderAngle.flData) == 0x10){
						// No action necessary.
					}
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
    for (i = 0; i < messagesToSend; ++i) {
        switch (msgs[i]) {
            case SCHED_ID_RUDDER_ANGLE: {
                tCanMessage msg;
                PackagePgn127245(&msg, nodeId, 0xFF, 0xF, NAN, hilReceivedData.data.rAngle);
                ecan1_buffered_transmit(&msg);
            } break;
        }
    }
}