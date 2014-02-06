#include <stdbool.h>
#include <stdlib.h>

#include "Dsp3000.h"

// This is the maximum size the rate data will be.
#define MAX_DATA_SIZE 25

typedef enum {
	STATE_WAIT_RATE_DATA = 0, // Consumes spaces until a non-space is found. This should be the rate data.
	STATE_GETTING_RATE_DATA,  // Consumes rate data bytes
	STATE_WAIT_STATUS_BIT,    // Consumes spaces until a non-space is found. This should be the status bit.
	STATE_WAIT_CR,            // Consumes a carriage return.
	STATE_WAIT_LF             // Consumes a linefeed
} Dsp3000ParseState;

bool Dsp3000Parse(char in, Dsp3000Output *data)
{
	static uint8_t state = STATE_WAIT_CR; // Store the current parsing state. We start at the CR state because that'll be easiest to synchronize on.
	static char rateData[MAX_DATA_SIZE];  // Store the string that we will eventually parse.
	static uint8_t index = 0; // Store the current byte index in the rateData array

	if (state == STATE_WAIT_CR) {
		if (in == '\r') {
			state = STATE_WAIT_LF;
		}
	} else if (state == STATE_WAIT_LF) {
		if (in == '\n') {
			state = STATE_WAIT_RATE_DATA;
		} else {
			state = STATE_WAIT_CR;
		}
	} else if (state == STATE_WAIT_RATE_DATA) {
		if (in != ' ') {
			rateData[0] = in;
			index = 1;
			state = STATE_GETTING_RATE_DATA;
		}
	} else if (state == STATE_GETTING_RATE_DATA) {
		if (index == MAX_DATA_SIZE) {
			state = STATE_WAIT_CR; // We just fail out here if we can't store all of the data.
		} else if (in != ' ') {
			rateData[index++] = in;
		} else {
			// Make sure the rate is a proper C string.
			rateData[index] = '\0';
			
			// Decode the ASCII representation of the turn rate.
			data->zRate = atof(rateData);
			
			state = STATE_WAIT_STATUS_BIT;
		}
	} else if (state == STATE_WAIT_STATUS_BIT) {
		if (in == '\r') {
			state = STATE_WAIT_LF;
		} else if (in != ' ') {
			if (in == '1') {
				data->status = true;
				return true;
			} else if (in == '0') {
				data->status = false;
				return true;
			}
			state = STATE_WAIT_CR;
		}
	}
	return false;
}
