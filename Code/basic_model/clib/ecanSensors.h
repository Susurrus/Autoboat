#ifndef __ECAN_SENSORS_H__
#define __ECAN_SENSORS_H__

#include <stddef.h>
#include <stdbool.h>

#include "types.h"

static struct {
	tFloatToChar speed;
	tFloatToChar direction;
	bool newData;
} windData;

static struct {
	tFloatToChar temp;
	tFloatToChar pressure;
	tFloatToChar humidity;
	bool newData;
} airData;

static struct {
	tFloatToChar speed;
	tFloatToChar temp;
	tFloatToChar depth;
	bool newData;
} waterData;

static struct {
	tIntToChar rpm;
	bool newData;
} throttleData;

void GetWindData(unsigned char *data);

void GetAirData(unsigned char *data);

void GetWaterData(unsigned char *data);

void GetThrottleData(unsigned char *data);

unsigned char ProcessAllEcanMessages();

#endif // __ECAN_SENSORS_H__
