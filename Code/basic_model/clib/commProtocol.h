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

// Definitions of unions useful in transmitting data serially
typedef union{
	unsigned char    chData[2];
	unsigned short   usData;
} tUnsignedShortToChar; 

typedef union{
	unsigned char    chData[2];
 	short   		 shData;
} tShortToChar; 

typedef union{
	unsigned char   chData[4];
 	unsigned int   	uiData;
} tUnsignedIntToChar; 

typedef union{
	unsigned char   chData[4];
 	int   			inData;
} tIntToChar; 

typedef union{
	unsigned char   chData[4];
 	float   		flData;
	unsigned short	shData[2];
} tFloatToChar; 

// Declaration of the relevant message structs used.
typedef struct tSensorData {
	tShortToChar speed;
	tFloatToChar lat;
	tFloatToChar lon;
	tFloatToChar alt;
	unsigned char year;
	unsigned char month;
	unsigned char day;
	unsigned char hour;
	unsigned char minute;
	unsigned char second;
	tFloatToChar cog;
	tFloatToChar sog;
	tUnsignedShortToChar r_Position;
	unsigned char r_SBLimit;
	unsigned char r_PortLimit;
	tUnsignedShortToChar b_Position;
	unsigned char b_SBLimit;
	unsigned char b_PortLimit;
} tSensorData;

typedef struct tActuatorData {
	unsigned char r_enable;
	unsigned char r_direction;
	tUnsignedShortToChar r_up;
	tUnsignedShortToChar r_period;
	unsigned char b_enable;
	unsigned char b_direction;
	tUnsignedIntToChar t_identifier;
	unsigned char data[6];
	unsigned char size;
	unsigned char trigger;
} tActuatorData;

typedef struct tStateData {
	tFloatToChar L2_Vector[3];
	tFloatToChar desiredRudder;
	tFloatToChar velocity[3];
	tFloatToChar solar_azimuth;
	tFloatToChar solar_zenith;
	unsigned char currentWaypointIndex;
	unsigned char waypointMode;
	unsigned char waypointCount;
} tStateData;

typedef struct tCommandData {
	unsigned char runMode; // 0 for stop, 1 for run, 2 for return-to-base, 3 for manual RC control)
	unsigned char HILEnable; // 0 to enable normal running, 1 for HIL running)
	unsigned char waypointMode; // Sets the waypoint navigation modes.
	unsigned char waypointCount; // Number of waypoints being transmit
	tShortToChar waypoints[16]; // Store room for 8 north/east pairs of waypoints
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

void getSensorData(unsigned char* data);

void setSensorData(unsigned char* data);

void getActuatorData(unsigned char* data);

void setActuatorData(unsigned char* data);

void getStateData(unsigned char* data);

void getCommandData(unsigned char* data);


