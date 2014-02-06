/**
 * This file implements the serial library for the Tokimec VSAS-2GM IMU.
 */
#ifndef DSP3000_H
#define DSP3000_H

#include <stdint.h>

typedef struct {
	float zRate; // Rate of rotation in radians/s, clockwise is positive.
	bool status; // 0 if malfunction or starting up (bad data), 1 if good data
} Dsp3000Output;

/**
 * Decodes an incoming data stream one byte at a time and outputs the result into the provided
 * Dsp3000Output struct if a proper message was decoded.
 * @returns true if a message was successfully decoded.
 */
bool Dsp3000Parse(char in, Dsp3000Output *data);

#endif // DSP3000_H
