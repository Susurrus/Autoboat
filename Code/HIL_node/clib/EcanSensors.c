#include "EcanSensors.h"

#include "ecanDefinitions.h"
#include "ecanFunctions.h"
#include "Nmea2000.h"
#include "Types.h"
#include "Rudder.h"
#include "CanMessages.h"
#include "Node.h"

struct RudderCanData rudderCanDataStore;
struct ThrottleData throttleDataStore = {};
struct GpsData gpsDataStore = {};
struct DateTimeData dateTimeDataStore = {};

uint8_t ProcessAllEcanMessages(void)
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
