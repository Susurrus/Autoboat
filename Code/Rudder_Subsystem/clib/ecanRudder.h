#ifndef _ECAN_RUDDER_H_
#define _ECAN_RUDDER_H_
#include "types.h"

struct rudderMessages{
	uint8_t calibrate;
};



void RudderEcanInit(void);

void rudderTransmit(void);

void rudderSendNmea(void);

void rudderSendCustomLimit(void);

void ProcessAllEcanMessages(void);

uint8_t GetRudderMessage(void);

#endif // _ECAN_RUDDER_H_
