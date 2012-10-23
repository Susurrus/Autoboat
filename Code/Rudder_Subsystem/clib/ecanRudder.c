#include "ecanDefinitions.h"
#include "rudder_Subsystem.h"
#include "MavlinkMessageScheduler.h"
#include "ecanRudder.h"
#include "ecanFunctions.h"
#include "nmea2000.h"

#define RUDDER_MSG_ID_NMEA_ANGLE 10
#define ECAN_ID_NMEA_ANGLE 127245
#define RUDDER_MSG_ID_CUSTOM_LIMITS 20
#define ECAN_ID_CUSTOM_LIMITS 8080
#define RUDDER_MSG_ID_SET_STATUS 8081

static struct rudderMessages rudderMessageStore;

/**
 * Schedule the CAN messages for 10Hz.
 */
void RudderEcanInit(void){

	//NMEA2000 message
	if (!AddMessage(RUDDER_MSG_ID_NMEA_ANGLE, 10)) {
            while (1);
        }
	
	//Custom CAN message
	if (!AddMessage(RUDDER_MSG_ID_CUSTOM_LIMITS, 10)) {
            while (1);
        }


}

void rudderTransmit(void){
	// And now transmit all messages for this timestep
	SListItem *messagesToSend = IncrementTimestep();
	SListItem *j;
	for (j = messagesToSend; j; j = j->sibling) {
		switch(j->id) {
			case RUDDER_MSG_ID_CUSTOM_LIMITS: {
				rudderSendCustomLimit();
			} break;

			case RUDDER_MSG_ID_NMEA_ANGLE: {
				rudderSendNmea();
			} break;
			
			default: {
				
			} break;
		}
	}			
}

//configures the message into a tCanMessage, and sends it
void rudderSendNmea(void){
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
void rudderSendCustomLimit(void){
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


void processAllEcanMessages(void)
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
			//update send message rates
			} else if (msg.id == 0x8082){
				rudderMessageStore.angleRate = msg.payload[0];
				rudderMessageStore.statusRate = msg.payload[1];
				
			} else {
				pgn = Iso11783Decode(msg.id, NULL, NULL, NULL);
				switch (pgn) {
				case ECAN_ID_NMEA_ANGLE: { 
					if((msg.payload[2] != 0xFF) && (msg.payload[3] !=0xFF)){
					rudderMessageStore.newAngle = msg.payload[3];
					rudderMessageStore.newAngle = (rudderMessageStore.newAngle << 8) | msg.payload[2];
					}
				} break;
				}
			}
		}
	}while (messagesLeft > 0);
}

uint8_t getCalibrateMessage(void) {
	uint8_t temp = rudderMessageStore.calibrate;
	rudderMessageStore.calibrate = 0;
	return temp;
}

void updateMessageRate(void) {
	//handle the angle message first
	if(rudderMessageStore.angleRate != 0xFF){
		if(rudderMessageStore.angleRate == 0x00){
		//write code for this
		} else if((rudderMessageStore.angleRate <= 100) && (rudderMessageStore.angleRate >= 1)){
			RemoveMessage(RUDDER_MSG_ID_NMEA_ANGLE);
			AddMessage(RUDDER_MSG_ID_NMEA_ANGLE, rudderMessageStore.angleRate);
		}
	}
	
	//handle the status message
	if(rudderMessageStore.statusRate != 0xFF){
		if(rudderMessageStore.statusRate == 0x00){
		//write code for this
		} else if((rudderMessageStore.statusRate <= 100) && (rudderMessageStore.statusRate >= 1)){
			RemoveMessage(RUDDER_MSG_ID_CUSTOM_LIMITS);
			AddMessage(RUDDER_MSG_ID_CUSTOM_LIMITS, rudderMessageStore.statusRate);
		}
	}
	rudderMessageStore.statusRate = 0xFF;
	rudderMessageStore.angleRate = 0xFF;
}

double getNewAngle(void){
	double temp = rudderMessageStore.newAngle;
	return temp/10000;
}
