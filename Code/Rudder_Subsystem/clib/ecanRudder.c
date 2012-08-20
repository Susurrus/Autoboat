#include "ecanDefinitions.h"
#include "rudder_Subsystem.h"
#include "MavlinkMessageScheduler.h"
#include "ecanRudder.h"

#define RUDDER_MSG_ID_NMEA_ANGLE 10
#define ECAN_ID_NMEA_ANGLE 127245
#define RUDDER_MSG_ID_CUSTOM_LIMITS 20
#define ECAN_ID_CUSTOM_LIMITS 8080
#define RUDDER_MSG_ID_SET_STATUS 8081

static struct rudderMessages rudderMessageStore;

//Add messages to the Scheduler which we will later process.
void RudderEcanInit(void){

	//NMEA2000 message
	AddMessage(RUDDER_MSG_ID_NMEA_ANGLE,40);
	
	//Custom CAN message
	AddMessage(RUDDER_MSG_ID_CUSTOM_LIMITS, 40);


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
    Message.id = ECAN_ID_NMEA_ANGLE;
    Message.buffer = 0;
    Message.message_type = CAN_MSG_DATA;
    Message.frame_type = CAN_FRAME_EXT;
	//unused values by our functions
    Message.payload[0] = 0xFF;
    Message.payload[1] = 0xFF;
	//only used for receiving a wanted angle
    Message.payload[2] = 0xFF;
    Message.payload[3] = 0xFF;
	//Send current angle over the CAN bus
    Message.payload[4] = rudderData.rudderPositionAngle;
    Message.payload[5] = (rudderData.rudderPositionAngle>>8);
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
    Message.payload[1] = ((rudderData.potValue >> 8));
    Message.payload[2] = rudderData.portLimitValue;
    Message.payload[3] = ((rudderData.portLimitValue >> 8) & 0x02);
    Message.payload[4] = rudderData.starLimitValue;
    Message.payload[5] = ((rudderData.starLimitValue >> 8) & 0x02);
    {
        Message.payload[6] = (rudderData.portLimit << 7);
        Message.payload[6] = (rudderData.starLimit << 5) | Message.payload[6];
        //if rudder is calibrated set second bit high
        //if it is not calibrated set rudder to 'enable' (First bit high)
        if(rudderData.calibrate) {
                Message.payload[6] = 2 | Message.payload[6];
        } else {
                Message.payload[6] = 1 | Message.payload[6];
        }
    }
    Message.validBytes = 7;
	//transmit the message over CAN
    ecan1_transmit(Message);
}


void ProcessAllEcanMessages(void)
{
	uint8_t messagesLeft = 0;
	tCanMessage msg;
	uint32_t pgn;

	uint8_t messagesHandled = 0;

	do {
		int foundOne = ecan1_receive(&msg, &messagesLeft);
		if (foundOne) {
			// Process custom rudder messages. Anything not explicitly handled is assumed to be a NMEA2000 message.
			if (msg.id == 0x8081) {
				if ((msg.payload[1] & 0x01) == 1) {
					rudderMessageStore.calibrate = 1;
				}
				else{
					rudderMessageStore.calibrate = 0;
				}
			/*else if (msg.id == 0x8082){
			
			}*/
			} else {
				//pgn = Iso11783Decode(msg.id, NULL, NULL, NULL);
				pgn = 1;
				switch (pgn) {
				case 126992: { // From GPS
				/*	sensorAvailability.gps.enabled_counter = 0;
					uint8_t rv = ParsePgn126992(msg.payload, NULL, NULL, &dateTimeDataStore.year, &dateTimeDataStore.month, &dateTimeDataStore.day, &dateTimeDataStore.hour, &dateTimeDataStore.min, &dateTimeDataStore.sec, &dateTimeDataStore.usecSinceEpoch);
					// Check if all 6 parts of the datetime were successfully decoded before triggering an update
					if ((rv & 0xFC) == 0xFC) {
						sensorAvailability.gps.active_counter = 0;
						dateTimeDataStore.newData = true;
					}*/
				} break;
				}
			}

			++messagesHandled;
		}
	}while (messagesLeft > 0);

	//UpdateSensorsAvailability();
	
}


uint8_t GetRudderMessage(void) {
	uint8_t temp = rudderMessageStore.calibrate;
	rudderMessageStore.calibrate = 0;
	return temp;
}

