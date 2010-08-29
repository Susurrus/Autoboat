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
tActuatorData actuatorDataMessage;
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
		if (checksum == calculateChecksum(&message[2], messageIndex-4)) {
			// We now memcpy all the data into our global data structs.
			// NOTE: message[3] is used to skip the header & message ID info
			switch (message[3]) {
				case 1:
					memcpy(&sensorDataMessage, &message[3], sizeof(tSensorData));
					break;
				case 2:
					memcpy(&actuatorDataMessage, &message[3], sizeof(tActuatorData));
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
	data[0] = (float)sensorDataMessage.speed;
	data[1] = sensorDataMessage.lat;
	data[2] = sensorDataMessage.lon;
	data[3] = sensorDataMessage.alt;
	data[4] = (float)(((unsigned long)sensorDataMessage.month) | ((unsigned long)sensorDataMessage.year) << 16);
	data[5] = (float)(((unsigned long)sensorDataMessage.hour) | ((unsigned long)sensorDataMessage.day) << 16);
	data[6] = (float)(((unsigned long)sensorDataMessage.second) | ((unsigned long)sensorDataMessage.minute)<< 16);
	data[7] = sensorDataMessage.cog;
	data[8] = sensorDataMessage.sog;
	data[9] = sensorDataMessage.r_Position;
	data[10] = (float)(((unsigned long)sensorDataMessage.r_PortLimit) | ((unsigned long)sensorDataMessage.r_SBLimit) << 16);
	data[11] = sensorDataMessage.b_Position;
	data[12] = (float)(((unsigned long)sensorDataMessage.b_PortLimit) | ((unsigned long)sensorDataMessage.b_SBLimit) << 16);
}

void getActuatorData(unsigned long* data) {
	data[0] = ((unsigned long)actuatorDataMessage.r_enable) | ((unsigned long)actuatorDataMessage.r_direction) << 16;
	data[1] = ((unsigned long)actuatorDataMessage.r_up) | ((unsigned long)actuatorDataMessage.r_period) << 16;
	data[2] = ((unsigned long)actuatorDataMessage.b_enable) | ((unsigned long)actuatorDataMessage.b_direction) << 16;
	data[3] = actuatorDataMessage.t_identifier;
	data[4] = ((unsigned long)actuatorDataMessage.data[0]) | ((unsigned long)actuatorDataMessage.data[1]) << 8 | ((unsigned long)actuatorDataMessage.data[2]) << 16 | ((unsigned long)actuatorDataMessage.data[3]) << 24;
	data[5] = ((unsigned long)actuatorDataMessage.data[4]) | ((unsigned long)actuatorDataMessage.data[5]) << 8 | ((unsigned long)actuatorDataMessage.size) << 16 | ((unsigned long)actuatorDataMessage.trigger) << 24;
}

void setActuatorData(unsigned long* data) {
	actuatorDataMessage.r_enable = (unsigned char)data[0];
	actuatorDataMessage.r_direction = (unsigned char)(data[0] >> 16);
	actuatorDataMessage.r_up = (unsigned short)data[1];
	actuatorDataMessage.r_period = (unsigned short)((data[1] & 0xFF00) >> 16);
	actuatorDataMessage.t_identifier = data[3];
	actuatorDataMessage.data[0] = (unsigned char)data[4];
	actuatorDataMessage.data[1] = (unsigned char)(data[4] >> 8);
	actuatorDataMessage.data[2] = (unsigned char)(data[4] >> 16);
	actuatorDataMessage.data[3] = (unsigned char)(data[4] >> 24);
	actuatorDataMessage.data[4] = (unsigned char)data[5];
	actuatorDataMessage.data[5] = (unsigned char)(data[5] >> 8);
	actuatorDataMessage.size = (unsigned char)(data[5] >> 16);
	actuatorDataMessage.trigger = (unsigned char)(data[5] >> 24);
}

void getStateData(float* data) {
	data[0] = stateDataMessage.L2_Vector[0];
	data[1] = stateDataMessage.L2_Vector[1];
	data[2] = stateDataMessage.L2_Vector[2];
	data[3] = stateDataMessage.desiredRudder;
	data[4] = stateDataMessage.velocity[0];
	data[5] = stateDataMessage.velocity[2];
	data[6] = stateDataMessage.velocity[3];
	data[7] = stateDataMessage.solar_azimuth;
	data[8] = stateDataMessage.solar_zenith;
	data[9] = (float)(((unsigned long)stateDataMessage.currentWaypointIndex) | ((unsigned long)stateDataMessage.waypointMode) << 16);
	data[10] = (float)stateDataMessage.waypointCount;
}

void getCommandData(unsigned long* data) {
	data[0] = ((unsigned long)commandDataMessage.stop) | (((unsigned long)commandDataMessage.go) << 16);
	data[1] = ((unsigned long)commandDataMessage.returnToBase) | (((unsigned long)commandDataMessage.setWaypointMode) << 16);
	data[2] = ((unsigned long)commandDataMessage.setWaypoints[0]) | (((unsigned long)commandDataMessage.setWaypoints[1]) << 16);
	data[3] = ((unsigned long)commandDataMessage.setWaypoints[2]) | (((unsigned long)commandDataMessage.setWaypoints[3]) << 16);
	data[4] = ((unsigned long)commandDataMessage.setWaypoints[4]) | (((unsigned long)commandDataMessage.setWaypoints[5]) << 16);
	data[5] = ((unsigned long)commandDataMessage.setWaypoints[6]) | (((unsigned long)commandDataMessage.setWaypoints[7]) << 16);
	data[6] = ((unsigned long)commandDataMessage.setWaypoints[8]) | (((unsigned long)commandDataMessage.setWaypoints[9]) << 16);
	data[7] = ((unsigned long)commandDataMessage.setWaypoints[10]) | (((unsigned long)commandDataMessage.setWaypoints[11]) << 16);
	data[8] = ((unsigned long)commandDataMessage.setWaypoints[12]) | (((unsigned long)commandDataMessage.setWaypoints[13]) << 16);
	data[9] = ((unsigned long)commandDataMessage.setWaypoints[14]) | (((unsigned long)commandDataMessage.setWaypoints[15]) << 16);
	data[10] = ((unsigned long)commandDataMessage.setWaypointCount) | (((unsigned long)commandDataMessage.enableManualControl) << 16);
}


