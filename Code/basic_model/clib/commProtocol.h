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

// Declaration of the relevant message structs used.
typedef struct tSensorData {
	short speed;
	float lat;
	float lon;
	float alt;
	float cog;
	float sog;
	unsigned char day;
	unsigned char month;
	unsigned char year;
	unsigned char second;
	unsigned char minute;
	unsigned char hour;
	float r_Position;
	unsigned char r_SBLimit;
	unsigned char r_PortLimit;
	float b_Position;
	unsigned char b_SBLimit;
	unsigned char b_PortLimit;
} tSensorData;

typedef struct tActuatorData {
	unsigned char r_enable;
	unsigned char r_direction;
	unsigned short r_up;
	unsigned short r_period;
	unsigned char b_enable;
	unsigned char b_direction;
	unsigned int t_identifier;
	unsigned char data[6];
	unsigned char size;
	unsigned char trigger;
} tActuatorData;

typedef struct tStateData {
	float L2_Vector[3];
	float desiredRudder;
	float velocity[3];
	float solar_azimuth;
	float solar_zenith;
	unsigned char currentWaypointIndex;
	unsigned char waypointMode;
	unsigned char waypointCount;
} tStateData;

typedef struct tCommandData {
	unsigned char stop;
	unsigned char go;
	unsigned char returnToBase;
	unsigned char setWaypointMode;
	unsigned short setWaypoints[16]; // Store room for 8 north/east pairs of waypoints
	unsigned char setWaypointCount;
	unsigned char enableManualControl;
} tCommandData;

/**
 * This function builds a full message internally byte-by-byte,
 * verifies its checksum, and then pushes that data into the
 * appropriate struct.
 */
void buildAndCheckMessage(unsigned char characterIn);

/**
 * This function calculates the checksum of some bytes in an
 * array by XORing all of them.
 */
unsigned char calculateChecksum(char* sentence, unsigned char size);

void getSensorData(float* data);

void getActuatorData(float* data);

void getStateData(float* data);

void getCommandData(float* data);


