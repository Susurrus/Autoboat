#include "ecanDefinitions.h"
#include "rudder_Subsystem.h"
#include "MavlinkMessageScheduler.h"
#include "ecanRudder.h"
#include "ecanFunctions.h"
#include "nmea2000.h"

// Declare some constants for use with the message scheduler
// (don't use PGN or message ID as it must be a uint8)
#define SCHED_ID_RUDDER_ANGLE    1
#define SCHED_ID_CUSTOM_LIMITS   2
#define SCHED_ID_TEMPERATURE     3

// Define the PGN numbers for necessary NMEA2000 messages
#define PGN_RUDDER_ANGLE 127245
#define PGN_TEMP 130311

// Define the CAN ID for our custom messages
#define CAN_ID_CUSTOM_LIMITS 0x8080
#define CAN_ID_SET_STATUS 0x8081
#define CAN_ID_SET_TX_RATE 0x8082

// Declare a struct for storing received data.
static struct {
	bool calibrate;	 // Whether a calibration has been requested or not.
	int16_t newAngle;   // The commanded rudder angle
} rudderMessageStore;

void RudderSubsystemInit(void)
{

	// Transmit the rudder angle at 10Hz
	if (!AddMessage(SCHED_ID_RUDDER_ANGLE, 10)) {
		while (1);
	}

	// Transmit status at 4Hz
	if (!AddMessage(SCHED_ID_CUSTOM_LIMITS, 4)) {
		while (1);
	}

	// Transmit temperature at 1Hz
	if (!AddMessage(SCHED_ID_TEMPERATURE, 1)) {
		while (1);
	}
}

void RudderSendNmea(void) {
	// Set CAN header information.
	tCanMessage Message;
	Message.id = Iso11783Encode(PGN_RUDDER_ANGLE, 10, 255, 2);
	Message.buffer = 0;
	Message.message_type = CAN_MSG_DATA;
	Message.frame_type = CAN_FRAME_EXT;
	Message.validBytes = 6;

	// Now fill in the data.
	Message.payload[0] = 0xFF; // Unused
	Message.payload[1] = 0xFF; // Unused
	Message.payload[2] = 0xFF; // Unused
	Message.payload[3] = 0xFF; // Unused
	// Convert rudderPositionAngle to 1e-4 radians
	int16_t angle = ((float)rudderData.rudderPositionAngle * 10000);
	// Send current angle over the CAN bus
	Message.payload[4] = angle;
	Message.payload[5] = angle >> 8;

	// And finally transmit it.
	ecan1_transmit(Message);
}

void RudderSendCustomLimit(void)
{
	// Set CAN header information.
	tCanMessage Message;
	Message.id = CAN_ID_CUSTOM_LIMITS;
	Message.buffer = 0;
	Message.message_type = CAN_MSG_DATA;
	Message.frame_type = CAN_FRAME_EXT;
	Message.validBytes = 7;

	// Now fill in the data.
	Message.payload[0] = rudderData.potValue;
	Message.payload[1] = rudderData.potValue >> 8;
	Message.payload[2] = rudderData.portLimitValue;
	Message.payload[3] = rudderData.portLimitValue >> 8;
	Message.payload[4] = rudderData.starLimitValue;
	Message.payload[5] = rudderData.starLimitValue >> 8;
	Message.payload[6] = rudderData.portLimit << 7;
	Message.payload[6] |= rudderData.starLimit << 5;
	//if rudder is calibrated set second bit high
	//if it is not calibrated set rudder to 'enable' (First bit high)
	if (rudderData.calibrate) {
		Message.payload[6] |= 0x02;
	} else {
		Message.payload[6] |= 0x01;
	}

	// And finally transmit it.
	ecan1_transmit(Message);
}

//configures the message into a tCanMessage, and sends it
void RudderSendTemperature(void)
{

	// Specify a new CAN message w/ metadata
	tCanMessage Message;
	Message.id = Iso11783Encode(PGN_TEMP, 10, 0xFF, 2);
	Message.buffer = 0;
	Message.message_type = CAN_MSG_DATA;
	Message.frame_type = CAN_FRAME_EXT;
	Message.validBytes = 8;

	// Now set the data.
	Message.payload[0] = 0xFF;      // SID
	Message.payload[1] = 2;         // Temperature instance (Inside)
	Message.payload[1] |= 0x3 << 6; // Humidity instance (Invalid)
	// Record the temperature in units of .01Kelvin.
	uint16_t temp = (uint16_t)((rudderData.temp + 273.2) * 100);
	Message.payload[2] = (uint8_t)temp;
	Message.payload[3] = (uint8_t)(temp >> 8);
	Message.payload[4] = 0xFF; // Humidity is invalid
	Message.payload[5] = 0xFF; // Humidity is invalid
	Message.payload[6] = 0xFF; // Atmospheric pressure is invalid
	Message.payload[7] = 0xFF; // Atmospheric pressure is invalid

	// And finally transmit it.
	ecan1_transmit(Message);
}

void SendAndReceiveEcan(void)
{
	uint8_t messagesLeft = 0;
	tCanMessage msg;
	uint32_t pgn;

	do {
		int foundOne = ecan1_receive(&msg, &messagesLeft);
		if (foundOne) {
			// Process custom rudder messages. Anything not explicitly handled is assumed to be a NMEA2000 message.
			if (msg.id == CAN_ID_SET_STATUS) {
				if ((msg.payload[0] & 0x01) == 1) {
					rudderMessageStore.calibrate = 1;
				} else{
					rudderMessageStore.calibrate = 0;
				}
			// Update send message rates
			} else if (msg.id == CAN_ID_SET_TX_RATE) {
				UpdateMessageRate(msg.payload[0], msg.payload[1]);
			} else {
				pgn = Iso11783Decode(msg.id, NULL, NULL, NULL);
				switch (pgn) {
				case PGN_RUDDER_ANGLE:
					if (msg.payload[2] != 0xFF && msg.payload[3] != 0xFF) {
						rudderMessageStore.newAngle = (msg.payload[3] << 8) | msg.payload[2];
					}
				break;
				}
			}
		}
	} while (messagesLeft > 0);

	// And now transmit all messages for this timestep
	SListItem *messagesToSend = IncrementTimestep();
	SListItem *j;
	for (j = messagesToSend; j; j = j->sibling) {
		switch (j->id) {
			case SCHED_ID_CUSTOM_LIMITS:
				RudderSendCustomLimit();
			break;

			case SCHED_ID_RUDDER_ANGLE:
				RudderSendNmea();
			break;

			case SCHED_ID_TEMPERATURE:
				RudderSendTemperature();
			break;
		}
	}
}

void UpdateMessageRate(const uint8_t angleRate, const uint8_t statusRate)
{
	//handle the angle message first
	if (angleRate != 0xFF) {
		if (angleRate == 0x00) {
		//write code for this
		} else if ((angleRate <= 100) && (angleRate >= 1)) {
			RemoveMessage(SCHED_ID_RUDDER_ANGLE);
			AddMessage(SCHED_ID_RUDDER_ANGLE, angleRate);
		}
	}

	//handle the status message
	if (statusRate != 0xFF) {
		if (statusRate == 0x00) {
		//write code for this
		} else if ((statusRate <= 100) && (statusRate >= 1)) {
			RemoveMessage(SCHED_ID_CUSTOM_LIMITS);
			AddMessage(SCHED_ID_CUSTOM_LIMITS, statusRate);
		}
	}
}

bool GetCalibrateMessage(void)
{
	bool temp = rudderMessageStore.calibrate;
	rudderMessageStore.calibrate = false;
	return temp;
}

float GetNewAngle(void)
{
	float temp = rudderMessageStore.newAngle;
	return temp/10000;
}
