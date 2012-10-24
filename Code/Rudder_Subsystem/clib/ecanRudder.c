#include "ecanDefinitions.h"
#include "rudder_Subsystem.h"
#include "MavlinkMessageScheduler.h"
#include "ecanRudder.h"
#include "ecanFunctions.h"
#include "nmea2000.h"

// Declare some constants for use with the message scheduler
// (don't use PGN or message ID as it must be a uint8)
#define RUDDER_MSG_ID_NMEA_ANGLE 10
#define RUDDER_MSG_ID_CUSTOM_LIMITS 20

// Define the PGN number for the NMEA angle message
#define ECAN_ID_NMEA_ANGLE 127245

// Define the CAN ID for our custom messages
#define ECAN_ID_CUSTOM_LIMITS 8080
#define RUDDER_MSG_ID_SET_STATUS 8081

// Declare a struct for storing received data.
static struct {
	bool calibrate;     // Whether a calibration has been requested or not.
	int16_t newAngle;   // The commanded rudder angle
} rudderMessageStore;

void RudderSubsystemInit(void) {

	// Transmit the rudder angle at 10Hz
	if (!AddMessage(RUDDER_MSG_ID_NMEA_ANGLE, 10)) {
		while (1);
	}

	// Transmit metadata at 4Hz
	if (!AddMessage(RUDDER_MSG_ID_CUSTOM_LIMITS, 4)) {
		while (1);
	}
}

//configures the message into a tCanMessage, and sends it
void RudderSendNmea(void) {
    //PGN127245 packing
    tCanMessage Message;
    Message.id = Iso11783Encode(ECAN_ID_NMEA_ANGLE, 10, 255, 2);
    Message.buffer = 0;
    Message.message_type = CAN_MSG_DATA;
    Message.frame_type = CAN_FRAME_EXT;
	//unused values by our functions
    Message.payload[0] = 0xFF;
    Message.payload[1] = 0xFF;
	//only used for receiving a wanted angle
    Message.payload[2] = 0xFF;
    Message.payload[3] = 0xFF;
	//convert rudderPositionAngle to 1e-4 radians
	int16_t angle = ((float)rudderData.rudderPositionAngle * 10000);
	//Send current angle over the CAN bus
    Message.payload[4] = angle;
    Message.payload[5] = (angle>>8);
    Message.validBytes = 6;
	//transmit the message over CAN
    ecan1_transmit(Message);

}

//configures the message into a tCanMessage, and sends it
void RudderSendCustomLimit(void){
    //MSG8080 Send Status
    tCanMessage Message;
    Message.id = ECAN_ID_CUSTOM_LIMITS;
    Message.buffer = 0;
    Message.message_type = CAN_MSG_DATA;
    Message.frame_type = CAN_FRAME_EXT;
    Message.payload[0] = rudderData.potValue;
    Message.payload[1] = rudderData.potValue >> 8;
    Message.payload[2] = rudderData.portLimitValue;
    Message.payload[3] = rudderData.portLimitValue >> 8;
    Message.payload[4] = rudderData.starLimitValue;
    Message.payload[5] = rudderData.starLimitValue >> 8;
    {
        Message.payload[6] = (rudderData.portLimit << 7);
        Message.payload[6] |= (rudderData.starLimit << 5);
        //if rudder is calibrated set second bit high
        //if it is not calibrated set rudder to 'enable' (First bit high)
        if(rudderData.calibrate) {
                Message.payload[6] |= 0x02;
        } else {
                Message.payload[6] |= 0x01;
        }
    }
    Message.validBytes = 7;
    //transmit the message over CAN
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
            if (msg.id == 0x8081) {
                if ((msg.payload[0] & 0x01) == 1) {
                    rudderMessageStore.calibrate = 1;
                }
                else{
                    rudderMessageStore.calibrate = 0;
                }
            // Update send message rates
            } else if (msg.id == 0x8082){
                UpdateMessageRate(msg.payload[0], msg.payload[1]);
            } else {
                pgn = Iso11783Decode(msg.id, NULL, NULL, NULL);
                switch (pgn) {
                case ECAN_ID_NMEA_ANGLE:
                    if((msg.payload[2] != 0xFF) && (msg.payload[3] != 0xFF)){
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
        switch(j->id) {
            case RUDDER_MSG_ID_CUSTOM_LIMITS: {
                RudderSendCustomLimit();
            } break;

            case RUDDER_MSG_ID_NMEA_ANGLE: {
                RudderSendNmea();
            } break;
        }
    }
}

void UpdateMessageRate(const uint8_t angleRate, const uint8_t statusRate) {
    //handle the angle message first
    if(angleRate != 0xFF){
        if(angleRate == 0x00){
        //write code for this
        } else if((angleRate <= 100) && (angleRate >= 1)){
            RemoveMessage(RUDDER_MSG_ID_NMEA_ANGLE);
            AddMessage(RUDDER_MSG_ID_NMEA_ANGLE, angleRate);
        }
    }

    //handle the status message
    if(statusRate != 0xFF){
        if(statusRate == 0x00){
        //write code for this
        } else if((statusRate <= 100) && (statusRate >= 1)){
            RemoveMessage(RUDDER_MSG_ID_CUSTOM_LIMITS);
            AddMessage(RUDDER_MSG_ID_CUSTOM_LIMITS, statusRate);
        }
    }
}

bool GetCalibrateMessage(void) {
	bool temp = rudderMessageStore.calibrate;
	rudderMessageStore.calibrate = false;
	return temp;
}

float GetNewAngle(void){
	float temp = rudderMessageStore.newAngle;
	return temp/10000;
}
