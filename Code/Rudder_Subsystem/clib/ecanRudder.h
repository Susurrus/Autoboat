#ifndef _ECAN_RUDDER_H_
#define _ECAN_RUDDER_H_
#include "types.h"

struct rudderMessages{
	uint8_t calibrate;
	uint8_t angleRate;
	uint8_t statusRate;
	int16_t newAngle;
};



void RudderEcanInit(void);

void rudderTransmit(void);

void rudderSendNmea(void);

void rudderSendCustomLimit(void);

void processAllEcanMessages(void);

uint8_t getcalibrateMessage(void);

void updateMessageRate(void);

double getNewAngle(void);

#endif // _ECAN_RUDDER_H_
