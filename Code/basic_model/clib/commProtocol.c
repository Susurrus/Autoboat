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
	data[0] = (float)sensorDataMessage.speed;
	data[1] = sensorDataMessage.lat;
	data[2] = sensorDataMessage.lon;
	data[3] = sensorDataMessage.alt;
	data[4] = sensorDataMessage.cog;
	data[5] = sensorDataMessage.sog;
	data[6] = (float)(sensorDataMessage.day | sensorDataMessage.month << 8);
	data[7] = (float)(sensorDataMessage.year | sensorDataMessage.second << 8);
	data[8] = (float)(sensorDataMessage.minute | sensorDataMessage.hour<< 8);
	data[9] = sensorDataMessage.r_Position;
	data[10] = (float)(sensorDataMessage.r_SBLimit | sensorDataMessage.r_PortLimit << 8);
	data[11] = sensorDataMessage.b_Position;
	data[12] = (float)(sensorDataMessage.b_SBLimit | sensorDataMessage.b_PortLimit << 8);
}

void getActuatorData(float* data) {
	data[0] = (float)(actuatorDataMessase.r_enable | actuatorDataMessase.r_direction << 8);
	data[1] = (float)actuatorDataMessase.r_up;
	data[2] = (float)actuatorDataMessase.r_period;
	data[3] = (float)(actuatorDataMessase.b_enable | b_direction << 8);
	data[4] = (float)actuatorDataMessase.t_identifier;
	data[5] = (float)(actuatorDataMessase.data[0] | actuatorDataMessase.data[1] << 8);
	data[6] = (float)(actuatorDataMessase.data[1] | actuatorDataMessase.data[2] << 8);
	data[7] = (float)(actuatorDataMessase.data[3] | actuatorDataMessase.data[4] << 8);
	data[8] = (float)(actuatorDataMessase.size | actuatorDataMessase.trigger << 8);
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
	data[9] = (float)(stateDataMessage.currentWaypointIndex | stateDataMessage.waypointMode << 8);
	data[10] = (float)stateDataMessage.waypointCount;
}

void getCommandData(float* data) {
	data[0] = (float)(commandDataMessage.stop | commandDataMessage.go << 8);
	data[1] = (float)(commandDataMessage.returnToBase | commandDataMessage.setWaypointMode << 8);
	data[2] = (float)commandDataMessage.setWaypoints[0];
	data[3] = (float)commandDataMessage.setWaypoints[1];
	data[4] = (float)commandDataMessage.setWaypoints[2];
	data[5] = (float)commandDataMessage.setWaypoints[3];
	data[6] = (float)commandDataMessage.setWaypoints[4];
	data[7] = (float)commandDataMessage.setWaypoints[5];
	data[8] = (float)commandDataMessage.setWaypoints[6];
	data[9] = (float)commandDataMessage.setWaypoints[7];
	data[10] = (float)commandDataMessage.setWaypoints[8];
	data[11] = (float)commandDataMessage.setWaypoints[9];
	data[12] = (float)commandDataMessage.setWaypoints[10];
	data[13] = (float)commandDataMessage.setWaypoints[11];
	data[14] = (float)commandDataMessage.setWaypoints[12];
	data[15] = (float)commandDataMessage.setWaypoints[13];
	data[16] = (float)commandDataMessage.setWaypoints[14];
	data[17] = (float)commandDataMessage.setWaypoints[15];
	data[18] = (float)(commandDataMessage.setWaypointCount | commandDataMessage.enableManualControl << 8);
}


