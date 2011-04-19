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
#include "circBuffer.h"
#include "uart2.h"

// These are local declarations of each of the message structs.
// They're populated with relevant data by buildAndcheckMessage().
tSensorData sensorDataMessage;
tActuatorData actuatorDataMessage;
tStateData stateDataMessage;
tCommandData commandDataMessage;

unsigned long receivedMessageCount; // Keep track of how many messages were successfully received.

/**
 * This function builds a full message internally byte-by-byte,
 * verifies its checksum, and then pushes that data into the
 * appropriate struct.
 */
void buildAndCheckMessage(unsigned char characterIn) {
	static unsigned char message[128];
	static unsigned char messageIndex;
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
		} else if (messageIndex == 127) {
			// If we've filled up the buffer, ignore the entire message as we can't store it all
			messageState = 0;
			messageIndex = 0;
		}
	} else if (messageState == 3) {
		// If we don't find the necessary ampersand we continue
		// recording data as we haven't found the footer yet until
		// we've filled up the entire message (ends at 124 characters
		// as we need room for the 2 footer chars).
		message[messageIndex++] = characterIn;
		if (characterIn == '&') {
			messageState = 4;
		} else if (messageIndex == 63) {
			messageState = 0;
			messageIndex = 0;
		} else {
			messageState = 3;
		}
	} else if (messageState == 4) {
		// Record the second ASCII-hex character of the checksum byte.
		message[messageIndex] = characterIn;

		// The checksum is now verified and if successful the message
		// is stored in the appropriate struct.
		if (message[messageIndex] == calculateChecksum(&message[2], messageIndex-4)) {
			// We now memcpy all the data into our global data structs.
			// NOTE: message[3] is used to skip the header & message ID info
			receivedMessageCount++;
			switch (message[2]) {
				case 1:
					setSensorData(&message[3]);
					break;
				case 2:
					setActuatorData(&message[3]);
					break;
				case 3:
					//memcpy(&stateDataMessage, &message[3], sizeof(tStateData));
					break;
				case 4:
					//memcpy(&commandDataMessage, &message[3], sizeof(tCommandData));
					break;
			}
		}
		
		// We clear all state variables here regardless of success.
		messageIndex = 0;
		messageState = 0;
		int b;
		for (b = 0;b < 64;b++) {
			message[b] = 0;
		}
	}
}

/**
 * This function sets the proper UART2
 * baud rate depending on the HIL mode.
 * A mode value of 0 will set the baud
 * rate back to the default 1200. A value
 * of 1 will set it to 115200.
 */
void setHilMode(unsigned char mode) {
	static unsigned char oldMode = 0;
	
	// Detect a change to HIL
	if (!oldMode && mode) {
		changeUart2BaudRate(HIL_BRG_REG);
		oldMode = mode;
	} else if (oldMode && !mode) {
		changeUart2BaudRate(BAUD4800_BRG_REG);
		oldMode = mode;
	}
}

void enableHil() {
	setHilMode(1);
}

void disableHil() {
	setHilMode(0);
}

/**
 * This function calculates the checksum of some bytes in an
 * array by XORing all of them.
 */
unsigned char calculateChecksum(unsigned char* sentence, unsigned char size) {

    unsigned char checkSum = 0;
	unsigned char i;
	for (i = 0; i < size; i++) {
		checkSum ^= sentence[i];
    }
	
    return checkSum;
}

/**
 * This function should be called continously. Each timestep
 * it runs through the most recently received data, parsing
 * it for sensor data. Once a complete message has been parsed
 * the data inside will be returned through the message
 * array.
 */
void processNewCommData(unsigned char* message) {
	while (getLength(&uart2RxBuffer) > 0) {
		buildAndCheckMessage(readFront(&uart2RxBuffer));
	}
}

void getSensorData(unsigned char* data) {
	data[0] = sensorDataMessage.speed.chData[0];
	data[1] = sensorDataMessage.speed.chData[1];
	data[2] = sensorDataMessage.lat.chData[0];
	data[3] = sensorDataMessage.lat.chData[1];
	data[4] = sensorDataMessage.lat.chData[2];
	data[5] = sensorDataMessage.lat.chData[3];
	data[6] = sensorDataMessage.lon.chData[0];
	data[7] = sensorDataMessage.lon.chData[1];
	data[8] = sensorDataMessage.lon.chData[2];
	data[9] = sensorDataMessage.lon.chData[3];
	data[10] = sensorDataMessage.alt.chData[0];
	data[11] = sensorDataMessage.alt.chData[1];
	data[12] = sensorDataMessage.alt.chData[2];
	data[13] = sensorDataMessage.alt.chData[3];
	data[14] = sensorDataMessage.year;
	data[15] = sensorDataMessage.month;
	data[16] = sensorDataMessage.day;
	data[17] = sensorDataMessage.hour;
	data[18] = sensorDataMessage.minute;
	data[19] = sensorDataMessage.second;
	data[20] = sensorDataMessage.cog.chData[0];
	data[21] = sensorDataMessage.cog.chData[1];
	data[22] = sensorDataMessage.cog.chData[2];
	data[23] = sensorDataMessage.cog.chData[3];
	data[24] = sensorDataMessage.sog.chData[0];
	data[25] = sensorDataMessage.sog.chData[1];
	data[26] = sensorDataMessage.sog.chData[2];
	data[27] = sensorDataMessage.sog.chData[3];
	data[28] = sensorDataMessage.newGpsData;
	data[29] = sensorDataMessage.r_Position.chData[0];
	data[30] = sensorDataMessage.r_Position.chData[1];
	data[31] = sensorDataMessage.r_SBLimit;
	data[32] = sensorDataMessage.r_PortLimit;
	data[33] = sensorDataMessage.b_Position.chData[0];
	data[34] = sensorDataMessage.b_Position.chData[1];
	data[35] = sensorDataMessage.b_SBLimit;
	data[36] = sensorDataMessage.b_PortLimit;
	data[37] = sensorDataMessage.timestamp.chData[0];
	data[38] = sensorDataMessage.timestamp.chData[1];
	data[39] = sensorDataMessage.newData;
	sensorDataMessage.newData = 0;
}

void setSensorData(unsigned char* data) {
	sensorDataMessage.speed.chData[0] = data[0];
	sensorDataMessage.speed.chData[1] = data[1];
	sensorDataMessage.lat.chData[0] = data[2];
	sensorDataMessage.lat.chData[1] = data[3];
	sensorDataMessage.lat.chData[2] = data[4];
	sensorDataMessage.lat.chData[3] = data[5];
	sensorDataMessage.lon.chData[0] = data[6];
	sensorDataMessage.lon.chData[1] = data[7];
	sensorDataMessage.lon.chData[2] = data[8];
	sensorDataMessage.lon.chData[3] = data[9];
	sensorDataMessage.alt.chData[0] = data[10];
	sensorDataMessage.alt.chData[1] = data[11];
	sensorDataMessage.alt.chData[2] = data[12];
	sensorDataMessage.alt.chData[3] = data[13];
	sensorDataMessage.year = data[14];
	sensorDataMessage.month = data[15];
	sensorDataMessage.day = data[16];
	sensorDataMessage.hour = data[17];
	sensorDataMessage.minute = data[18];
	sensorDataMessage.second = data[19];
	sensorDataMessage.cog.chData[0] = data[20];
	sensorDataMessage.cog.chData[1] = data[21];
	sensorDataMessage.cog.chData[2] = data[22];
	sensorDataMessage.cog.chData[3] = data[23];
	sensorDataMessage.sog.chData[0] = data[24];
	sensorDataMessage.sog.chData[1] = data[25];
	sensorDataMessage.sog.chData[2] = data[26];
	sensorDataMessage.sog.chData[3] = data[27];
	sensorDataMessage.newGpsData = data[28];
	sensorDataMessage.r_Position.chData[0] = data[29];
	sensorDataMessage.r_Position.chData[1] = data[30];
	sensorDataMessage.r_SBLimit = data[31];
	sensorDataMessage.r_PortLimit = data[32];
	sensorDataMessage.b_Position.chData[0] = data[33];
	sensorDataMessage.b_Position.chData[1] = data[34];
	sensorDataMessage.b_SBLimit = data[35];
	sensorDataMessage.b_PortLimit = data[36];
	sensorDataMessage.timestamp.chData[0] = data[37];
	sensorDataMessage.timestamp.chData[1] = data[38];
	sensorDataMessage.newData = 1;
}

void clearSensorData() {
	sensorDataMessage.speed.shData = 0;
	sensorDataMessage.lat.flData = 0.0;
	sensorDataMessage.lon.flData = 0.0;
	sensorDataMessage.alt.flData = 0.0;
	sensorDataMessage.year = 0;
	sensorDataMessage.month = 0;
	sensorDataMessage.day = 0;
	sensorDataMessage.hour = 0;
	sensorDataMessage.minute = 0;
	sensorDataMessage.second = 0;
	sensorDataMessage.cog.flData = 0.0;
	sensorDataMessage.sog.flData = 0.0;
	sensorDataMessage.newGpsData = 0;
	sensorDataMessage.r_Position.usData = 0;
	sensorDataMessage.r_SBLimit = 0;
	sensorDataMessage.r_PortLimit = 0;
	sensorDataMessage.b_Position.usData = 0;
	sensorDataMessage.b_SBLimit = 0;
	sensorDataMessage.b_PortLimit = 0;
	sensorDataMessage.timestamp.shData = 0;
	sensorDataMessage.newData = 0;
}

void getActuatorData(unsigned char* data) {
	data[0] = actuatorDataMessage.r_enable;
	data[1] = actuatorDataMessage.r_direction;
	data[2] = actuatorDataMessage.r_up.chData[0];
	data[3] = actuatorDataMessage.r_up.chData[1];
	data[4] = actuatorDataMessage.r_period.chData[0];
	data[5] = actuatorDataMessage.r_period.chData[1];
	data[6] = actuatorDataMessage.b_enable;
	data[7] = actuatorDataMessage.b_direction;
	data[8] = actuatorDataMessage.t_identifier.chData[0];
	data[9] = actuatorDataMessage.t_identifier.chData[1];
	data[10] = actuatorDataMessage.t_identifier.chData[2];
	data[11] = actuatorDataMessage.t_identifier.chData[3];
	data[12] = actuatorDataMessage.data[0];
	data[13] = actuatorDataMessage.data[1];
	data[14] = actuatorDataMessage.data[2];
	data[15] = actuatorDataMessage.data[3];
	data[16] = actuatorDataMessage.data[4];
	data[17] = actuatorDataMessage.data[5];
	data[18] = actuatorDataMessage.size;
	data[19] = actuatorDataMessage.trigger;
	actuatorDataMessage.timestamp.chData[0] = data[20];
	actuatorDataMessage.timestamp.chData[1] = data[21];
}

void setActuatorData(unsigned char* data) {
	actuatorDataMessage.r_enable = data[0];
	actuatorDataMessage.r_direction = data[1];
	actuatorDataMessage.r_up.chData[0] = data[2];
	actuatorDataMessage.r_up.chData[1] = data[3];
	actuatorDataMessage.r_period.chData[0] = data[4];
	actuatorDataMessage.r_period.chData[1] = data[5];
	actuatorDataMessage.b_enable = data[6];
	actuatorDataMessage.b_direction = data[7];
	actuatorDataMessage.t_identifier.chData[0] = data[8];
	actuatorDataMessage.t_identifier.chData[1] = data[9];
	actuatorDataMessage.t_identifier.chData[2] = data[10];
	actuatorDataMessage.t_identifier.chData[3] = data[11];
	actuatorDataMessage.data[0] = data[12];
	actuatorDataMessage.data[1] = data[13];
	actuatorDataMessage.data[2] = data[14];
	actuatorDataMessage.data[3] = data[15];
	actuatorDataMessage.data[4] = data[16];
	actuatorDataMessage.data[5] = data[17];
	actuatorDataMessage.size = data[18];
	actuatorDataMessage.trigger = data[19];
	actuatorDataMessage.timestamp.chData[0] = data[20];
	actuatorDataMessage.timestamp.chData[1] = data[21];
}

void getStateData(unsigned char* data) {
	data[0] = stateDataMessage.L2_Vector[0].chData[0];
	data[1] = stateDataMessage.L2_Vector[0].chData[1];
	data[2] = stateDataMessage.L2_Vector[0].chData[2];
	data[3] = stateDataMessage.L2_Vector[0].chData[3];
	data[4] = stateDataMessage.L2_Vector[1].chData[0];
	data[5] = stateDataMessage.L2_Vector[1].chData[1];
	data[6] = stateDataMessage.L2_Vector[1].chData[2];
	data[7] = stateDataMessage.L2_Vector[1].chData[3];
	data[8] = stateDataMessage.L2_Vector[2].chData[0];
	data[9] = stateDataMessage.L2_Vector[2].chData[1];
	data[10] = stateDataMessage.L2_Vector[2].chData[2];
	data[11] = stateDataMessage.L2_Vector[2].chData[3];
	data[12] = stateDataMessage.desiredRudder.chData[0];
	data[13] = stateDataMessage.desiredRudder.chData[1];
	data[14] = stateDataMessage.desiredRudder.chData[2];
	data[15] = stateDataMessage.desiredRudder.chData[3];
	data[16] = stateDataMessage.actualRudder.chData[0];
	data[17] = stateDataMessage.actualRudder.chData[1];
	data[18] = stateDataMessage.actualRudder.chData[2];
	data[19] = stateDataMessage.actualRudder.chData[3];
	data[20] = stateDataMessage.desiredVelocity[0].chData[0];
	data[21] = stateDataMessage.desiredVelocity[0].chData[1];
	data[22] = stateDataMessage.desiredVelocity[0].chData[2];
	data[23] = stateDataMessage.desiredVelocity[0].chData[3];
	data[24] = stateDataMessage.desiredVelocity[1].chData[0];
	data[25] = stateDataMessage.desiredVelocity[1].chData[1];
	data[26] = stateDataMessage.desiredVelocity[1].chData[2];
	data[27] = stateDataMessage.desiredVelocity[1].chData[3];
	data[28] = stateDataMessage.desiredVelocity[2].chData[0];
	data[29] = stateDataMessage.desiredVelocity[2].chData[1];
	data[30] = stateDataMessage.desiredVelocity[2].chData[2];
	data[31] = stateDataMessage.desiredVelocity[2].chData[3];
	data[32] = stateDataMessage.actualVelocity[0].chData[0];
	data[33] = stateDataMessage.actualVelocity[0].chData[1];
	data[34] = stateDataMessage.actualVelocity[0].chData[2];
	data[35] = stateDataMessage.actualVelocity[0].chData[3];
	data[36] = stateDataMessage.actualVelocity[1].chData[0];
	data[37] = stateDataMessage.actualVelocity[1].chData[1];
	data[38] = stateDataMessage.actualVelocity[1].chData[2];
	data[39] = stateDataMessage.actualVelocity[1].chData[3];
	data[40] = stateDataMessage.actualVelocity[2].chData[0];
	data[41] = stateDataMessage.actualVelocity[2].chData[1];
	data[42] = stateDataMessage.actualVelocity[2].chData[2];
	data[43] = stateDataMessage.actualVelocity[2].chData[3];
	data[44] = stateDataMessage.currentWaypointIndex;
	data[45] = stateDataMessage.waypointMode;
	data[46] = stateDataMessage.waypointCount;
}

void getCommandData(unsigned char* data) {
	data[0] = commandDataMessage.runMode;
	data[1] = commandDataMessage.HILEnable;
	data[2] = commandDataMessage.waypointMode;
	data[3] = commandDataMessage.waypointCount;
	data[4] = commandDataMessage.waypoints[0].chData[0];
	data[5] = commandDataMessage.waypoints[0].chData[1];
	data[6] = commandDataMessage.waypoints[1].chData[0];
	data[7] = commandDataMessage.waypoints[1].chData[1];
	data[8] = commandDataMessage.waypoints[2].chData[0];
	data[9] = commandDataMessage.waypoints[2].chData[1];
	data[10] = commandDataMessage.waypoints[3].chData[0];
	data[11] = commandDataMessage.waypoints[3].chData[1];
	data[12] = commandDataMessage.waypoints[4].chData[0];
	data[13] = commandDataMessage.waypoints[4].chData[1];
	data[14] = commandDataMessage.waypoints[5].chData[0];
	data[15] = commandDataMessage.waypoints[5].chData[1];
	data[16] = commandDataMessage.waypoints[6].chData[0];
	data[17] = commandDataMessage.waypoints[6].chData[1];
	data[18] = commandDataMessage.waypoints[7].chData[0];
	data[19] = commandDataMessage.waypoints[7].chData[1];
	data[20] = commandDataMessage.waypoints[8].chData[0];
	data[21] = commandDataMessage.waypoints[8].chData[1];
	data[22] = commandDataMessage.waypoints[9].chData[0];
	data[23] = commandDataMessage.waypoints[9].chData[1];
	data[24] = commandDataMessage.waypoints[10].chData[0];
	data[25] = commandDataMessage.waypoints[10].chData[1];
	data[26] = commandDataMessage.waypoints[11].chData[0];
	data[27] = commandDataMessage.waypoints[11].chData[1];
	data[28] = commandDataMessage.waypoints[12].chData[0];
	data[29] = commandDataMessage.waypoints[12].chData[1];
	data[30] = commandDataMessage.waypoints[13].chData[0];
	data[31] = commandDataMessage.waypoints[13].chData[1];
	data[32] = commandDataMessage.waypoints[14].chData[0];
	data[33] = commandDataMessage.waypoints[14].chData[1];
	data[34] = commandDataMessage.waypoints[15].chData[0];
	data[35] = commandDataMessage.waypoints[15].chData[1];
}
