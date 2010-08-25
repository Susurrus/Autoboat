/*
The MIT License

Copyright (c) 2010 UCSC Autonomous Systems Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

// ==============================================================
// This code provides a protocol decoder for the binary communications
// protocol used between the groundstation/HIL and the dsPIC in
// the Autoboat project. As most programming relies on Simulink and
// the Real-Time Workshop, retrieval functions here return arrays of
// data to be compatible (instead of much-nicer structs).
// A complete structure is passed byte-by-byte and assembled in an 
// internal buffer. This is then verified by its checksum and the 
//data pushed into the appropriate struct. This data can then be 
// retrieved via an accessor function.
//
// While this code was written specifically for the Autoboat and its
// protocol, it has been kept as modular as possible to be useful
// in other situations with the most minimal alterations.
// 
// Code by: Bryant W. Mairs
// First Revision: Aug 25 2010
// ==============================================================

#include "commProtocol.h"
#include <string.h>

// These are local declarations of each of the message structs.
// They're populated with relevant data by buildAndcheckMessage().
tSensorData sensorDataMessage;
tActuatorData actuatorDataMessase;
tStateData stateDataMessage;
tCommandData commandDataMessage;

/**
 * This function builds a full message internally byte-by-byte,
 * verifies its checksum, and then pushes that data into the
 * appropriate struct.
 */
void buildAndCheckMessage(unsigned char characterIn) {
	static char message[127];
	static unsigned char messageIndex;
	static unsigned char checksum;
	static unsigned char messageState;

	// This contains the function's state of whether
	// it is currently building a message.
	// 0 - Awaiting header byte 0 (%)
	// 1 - Awaiting header byte 1 (&)
	// 2 - Building message
	// 3 - Awaiting header byte 0 (^)
	// 4 - Awaiting header byte 1 (&)
	// 5 - Reading checksum character
	
	// We start recording a new message if we see the header
	if (messageState == 0) {
		if (characterIn == '%') {
			message[0] = characterIn;
			messageIndex = 1;
			messageState = 1;
		}
	} else if (messageState == 1) {
		// If we don't find the necessary ampersand we start over
		// waiting for a new sentence
		if (characterIn == '&') {
			message[1] = characterIn;
			messageIndex = 2;
			messageState = 2;
		} else {
			messageIndex = 0;
			messageState = 0;
		}
	} else if (messageState == 2) {
		// Record every character that comes in now that we're building a sentence.
		// Only stop if we run out of room or an asterisk is found.
		message[messageIndex++] = characterIn;
		if (characterIn == '^') {
			messageState = 3;
		} else if (messageIndex > 127) {
			// If we've filled up the buffer, ignore the entire message as we can't store it all
			messageState = 0;
			messageIndex = 0;
		}
	} else if (messageState == 3) {
		// If we don't find the necessary ampersand we continue
		// recording data as we haven't found the footer yet.
		if (characterIn == '&') {
			message[messageIndex++] = characterIn;
			messageIndex = 4;
			messageState = 4;
		} else {
			messageIndex = 0;
			messageState = 0;
		}
	} else if (messageState == 4) {
		// Record the second ASCII-hex character of the checksum byte.
		checksum = characterIn;

		// The checksum is now verified and if successful the message
		// is stored in the appropriate struct.
		if (checksum == getChecksum(&message[2], messageIndex-4)) {
			// We now memcpy all the data into our global data structs.
			// NOTE: message[3] is used to skip the header & message ID info
			switch (message[3]) {
				case 1:
					memcpy(&sensorDataMessage, &message[3], sizeof(tSensorData));
					break;
				case 2:
					memcpy(&actuatorDataMessase, &message[3], sizeof(tActuatorData));
					break;
				case 3:
					memcpy(&stateDataMessage, &message[3], sizeof(tStateData));
					break;
				case 4:
					memcpy(&commandDataMessage, &message[3], sizeof(tCommandData));
					break;
			}
		}
		
		// We clear all state variables here regardless of success.
		messageIndex = 0;
		messageState = 0;
	}
}

/**
 * This function calculates the checksum of some bytes in an
 * array by XORing all of them.
 */
unsigned char calculateChecksum(char* sentence, unsigned char size) {

    unsigned char checkSum = 0;
	unsigned char i;
	for (i = 0; i < size; i++) {
		checkSum ^= sentence[i];
    }
	
    return checkSum;
}

void getSensorData(float* data) {
}

void getActuatorData(float* data) {
}

void getStateData(float* data) {
}

void getCommandData(float* data) {
}


