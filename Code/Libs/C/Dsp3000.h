/**
 * This file implements the serial library for the Tokimec VSAS-2GM IMU.
 */
#ifndef DSP3000_H
#define DSP3000_H

#include <stdint.h>

typedef struct {
	float zRate;
	bool status; // 0 if malfunction or starting up (bad data), 1 if good data
} Dsp3000Output;

bool Dsp3000Parse(char in, Dsp3000Output *data);

#endif // DSP3000_H
