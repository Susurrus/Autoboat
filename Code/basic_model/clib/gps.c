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
// gps.c
// This is code implements a NMEA0138 parse for use in the autoboat
// project. It makes use of the circular buffer data structure cBuffer.c.
// It has been 
// written to be implemented in Simulink through the use of C-Function
// Call blocks. buildAndCheckSentence will append bytes from a data stream
// onto an internally managed buffer. Once a complete sentence has been
// constructed & verified by its checksum, it's passed off to the parser.
// This code is utilized in Simulink by using a UART or datastream block
// and feeding its output into a C-Function block calling buildAndCheckSentence.
// The required C files for compilation are: cBuffer.c, sbrk.c, gps.c.
// 
// Code by: Mariano I. Lizarraga
// First Revision: Aug 21 2008 @ 21:15
// Modified by: Bryant W. Mairs
// First Revision Aug 25 2010
// ==============================================================

#include <string.h>
#include <stdlib.h>

#include "gps.h"
#include "uart2.h"
#include "nmea0183.h"
#include "types.h"
#include "conversions.h"

static tGpsData gpsControlData;
static char sentence[127];
static unsigned char sentenceIndex;
static unsigned char checksum;
static unsigned char sentenceState;

/**
 * This function initializes the GPS by reconfiguring it to NOT output
 * GSA or GSV messages. This should leave just GGA and RMC messages,
 * which are the only two we care about.
 * It assumes that it is using UART2 and relies on the circular buffer
 * initialized for UART2 for transmission.
 */
void initGps() {
	// Configure GPS by:
	// - Disabling GSA
	// - Disabling GSV
	unsigned char disableGSASentence[] = "$PSRF103,2,0,0,1*26\r\n";
	unsigned char disableGSVSentence[] = "$PSRF103,3,0,0,1*27\r\n";

	// Enqueue the size of the sentences - 1 to remove string-terminating null character
	uart2EnqueueData(disableGSASentence, sizeof(disableGSASentence) - 1);
	uart2EnqueueData(disableGSVSentence, sizeof(disableGSVSentence) - 1);
}

void processGpsSentence(char *sentence) {
	if (sentence[3] == 'R' &&
		sentence[4] == 'M' &&
		sentence[5] == 'C') {
		parseRMC(sentence);
	} else if(sentence[3] == 'G' &&
		sentence[4] == 'G' &&
		sentence[5] == 'A') {
		parseGGA(sentence);
	}
}

void processNewGpsData() {
	while (GetLength(&uart2RxBuffer) > 0) {
		unsigned char c;
		Read(&uart2RxBuffer, &c);
		buildAndCheckSentence((char)c, sentence, &sentenceIndex, &sentenceState, &checksum, processGpsSentence);
	}
}

void getGpsData(unsigned char* data) {
	data[0] = gpsControlData.lat.chData[0];
	data[1] = gpsControlData.lat.chData[1];
	data[2] = gpsControlData.lat.chData[2];
	data[3] = gpsControlData.lat.chData[3];
	data[4] = gpsControlData.lon.chData[0];
	data[5] = gpsControlData.lon.chData[1];
	data[6] = gpsControlData.lon.chData[2];
	data[7] = gpsControlData.lon.chData[3];
	data[8] = gpsControlData.alt.chData[0];
	data[9] = gpsControlData.alt.chData[1];
	data[10] = gpsControlData.alt.chData[2];
	data[11] = gpsControlData.alt.chData[3];
	
	// Add date info
	data[12] = gpsControlData.year;
	data[13] = gpsControlData.month;
	data[14] = gpsControlData.day;
	
	// Add time info
	data[15] = gpsControlData.hour;
	data[16] = gpsControlData.min;
	data[17] = gpsControlData.sec;
	
	data[18] = gpsControlData.cog.chData[0];
	data[19] = gpsControlData.cog.chData[1];
	data[20] = gpsControlData.cog.chData[2];
	data[21] = gpsControlData.cog.chData[3];
	data[22] = gpsControlData.sog.chData[0];
	data[23] = gpsControlData.sog.chData[1];
	data[24] = gpsControlData.sog.chData[2];
	data[25] = gpsControlData.sog.chData[3];
	data[26] = gpsControlData.hdop.chData[0];
	data[27] = gpsControlData.hdop.chData[1];
	data[28] = gpsControlData.hdop.chData[2];
	data[29] = gpsControlData.hdop.chData[3];
	
	data[30] = gpsControlData.fix;
	data[31] = gpsControlData.sats;
	data[32] = gpsControlData.newData;
	
	// Mark this data as old now
	gpsControlData.newData = 0;
}

void SetGpsData(unsigned char* data) {
	gpsControlData.lat.chData[0] = data[0];
	gpsControlData.lat.chData[1] = data[1];
	gpsControlData.lat.chData[2] = data[2];
	gpsControlData.lat.chData[3] = data[3];
	
	gpsControlData.lon.chData[0] = data[4];
	gpsControlData.lon.chData[1] = data[5];
	gpsControlData.lon.chData[2] = data[6];
	gpsControlData.lon.chData[3] = data[7];
	
	gpsControlData.alt.chData[0] = data[8];
	gpsControlData.alt.chData[1] = data[9];
	gpsControlData.alt.chData[2] = data[10];
	gpsControlData.alt.chData[3] = data[11];
	
	gpsControlData.year = data[12];
	gpsControlData.month = data[13];
	gpsControlData.day = data[14];
	gpsControlData.hour = data[15];
	gpsControlData.min = data[16];
	gpsControlData.sec = data[17];
	
	gpsControlData.cog.chData[0] = data[18];
	gpsControlData.cog.chData[1] = data[19];
	gpsControlData.cog.chData[2] = data[20];
	gpsControlData.cog.chData[3] = data[21];
	gpsControlData.sog.chData[0] = data[22];
	gpsControlData.sog.chData[1] = data[23];
	gpsControlData.sog.chData[2] = data[24];
	gpsControlData.sog.chData[3] = data[25];
	
	gpsControlData.fix = 3;
	gpsControlData.sats = 7;
	
	gpsControlData.newData = data[26];
}

void clearGpsData() {
	gpsControlData.year = 0;
	gpsControlData.month = 0;
	gpsControlData.day = 0;
	gpsControlData.hour = 0;
	gpsControlData.min = 0;
	gpsControlData.sec = 0;
	gpsControlData.lat.flData = 0.0;
	gpsControlData.lon.flData = 0.0;
	gpsControlData.alt.flData = 0.0;
	gpsControlData.cog.flData = 0.0;
	gpsControlData.sog.flData = 0.0;
	gpsControlData.hdop.flData = 0.0;
	gpsControlData.fix = 0;
	gpsControlData.sats = 0;
	gpsControlData.newData = 0;
}

void parseRMC(char* stream) {
	// declare the local vars
	char token[15]; // Tokens set to 15 characters in length
	char tmp [3] ={0,0,'\0'}, tmp3[4]={0,0,0,'\0'};
	unsigned char chTmp = 0;
	
	
	// initialize tokenizer, let go first token which holds the msg type
	// token = strtok(stream, ",");
	myTokenizer(stream, ',', token);
	
	// 1.- hhmmss.ssss
	myTokenizer(NULL, ',', token);
	if (strlen(token)>5) {
		tmp[0] = token[0]; tmp[1] = token[1];
		gpsControlData.hour = (unsigned char) atoi(tmp);
		tmp[0] = token[2]; tmp[1] = token[3];
		gpsControlData.min = (unsigned char) atoi(tmp);
		tmp[0] = token[4]; tmp[1] = token[5];
		gpsControlData.sec = (unsigned char) atoi(tmp);		
	}
	
	// 2.- Status of position Fix
	myTokenizer(NULL, ',', token);
	if (strlen(token)== 1) {
		if (token[0] == 'A' || token[0] == 'D') {
			gpsControlData.fix = 1;
		} else {
			gpsControlData.fix = 0;
		}
	}
	
	// 3.- Latitude
	// ddmm.mmmmmm
	myTokenizer(NULL, ',', token);
	if (strlen(token)>0) {
		// get the first two values
		tmp[0] = token[0]; tmp[1] = token[1];
		// get the degrees
		chTmp = (unsigned char)atoi(tmp);
		// make the degrees zero for minutes conversion
		token[0]='0'; token[1]='0';
		// get the float
		gpsControlData.lat.flData = degMinToDeg(chTmp,atof(token));
		
		// 4.- Latitude Sector
		myTokenizer(NULL, ',', token);
		if (strlen(token)==1) {
			// set the sign of the float value
			if (token[0] == 'S' || token[0] == 'W') {
				gpsControlData.lat.flData = -gpsControlData.lat.flData;
			}
		}
	}
	
	// 5.- Longitude
	// dddmm.mmmmmm
	myTokenizer(NULL, ',', token);
	if (strlen(token)>0) {
		// get the first two values
		tmp3[0] = token[0]; tmp3[1] = token[1]; tmp3[2] = token[2];
		// get the degrees
		chTmp = (unsigned char)atoi(tmp3);
		// make the degrees zero for minutes conversion
		token[0]='0'; token[1]='0'; token [2] = '0';
		// get the float
		gpsControlData.lon.flData = degMinToDeg(chTmp,atof(token));
		
		// 6.- Longitude Sector
		myTokenizer(NULL, ',', token);
		if (strlen(token) == 1) {
			// set the sign of the float value
			if (token[0] == 'S' || token[0] == 'W') {
				gpsControlData.lon.flData = -gpsControlData.lon.flData;
			}
		}
	}
	
	// 7.- Speed over ground in knots
	// xx.xx
	myTokenizer(NULL, ',', token);	
	if (strlen(token) > 0) {
		gpsControlData.sog.flData = atof(token);
	}
	
	// 8.- Course over ground in degrees
	// xxx.xxx
	myTokenizer(NULL, ',', token);	
	if (strlen(token) > 0) {
		gpsControlData.cog.flData = atof(token);	
	}
	
	// 9.- UTC Date
	// ddmmyy
	myTokenizer(NULL, ',', token);	
	if (strlen(token) > 5) {
		// get day
		tmp[0]= token[0]; tmp[1]=token[1];
		gpsControlData.day = (unsigned char) atoi(tmp);	
		// get month
		tmp[0]= token[2]; tmp[1]=token[3];
		gpsControlData.month = (unsigned char) atoi(tmp);	
		// get year
		tmp[0]= token[4]; tmp[1]=token[5];
		gpsControlData.year = (unsigned char) atoi(tmp);	
	}
	
	// turn the flag on of new data
	gpsControlData.newData = 1;
}

void parseGGA(char* stream) {
	// declare the local vars
	char token[15]; // Declare a token to be 15 characters in length
	char tmp [3] ={0,0,'\0'}, tmp3[4]={0,0,0,'\0'};
	unsigned char chTmp = 0;
	
	// initialize tokenizer, let go first token which holds the msg type
	myTokenizer(stream, ',', token);
	
	// 1.- hhmmss.ssss
	myTokenizer(NULL, ',', token);
	// if (strlen(token)>5) {
		// tmp[0] = token[0]; tmp[1] = token[1];
		// gpsControlData.hour = (unsigned char) atoi(tmp);
		// tmp[0] = token[2]; tmp[1] = token[3];
		// gpsControlData.min = (unsigned char) atoi(tmp);
		// tmp[0] = token[4]; tmp[1] = token[5];
		// gpsControlData.sec = (unsigned char) atoi(tmp);		
	// }
	
	// 2.- Latitude
	// ddmm.mmmmmm
	myTokenizer(NULL, ',', token);
	if (strlen(token)>0) {
		// // get the first two values
		// tmp[0] = token[0]; tmp[1] = token[1];
		// // get the degrees
		// chTmp = (unsigned char)atoi(tmp);
		// // make the degrees zero for minutes conversion
		// token[0]='0'; token[1]='0';
		// // get the float
		// gpsControlData.lat.flData = degMinToDeg(chTmp,atof(token));		
		// // 3.- Latitude Sector
		myTokenizer(NULL, ',', token);
		// if (strlen(token)==1) {
			// // Set the sign of the float value.
			// // South & west are negative, so we invert the sign in
			// // those cases. North/East don't change the value so no
			// // need to check those.
			// if (token[0] == 'S' || token[1] == 'W') {
				// gpsControlData.lat.flData = -gpsControlData.lat.flData;
			// }
		// }
	}
	
	// 4.- Longitude
	// ddmm.mmmmmm
	myTokenizer(NULL, ',', token);	
	if (strlen(token)>0) {
		// // get the first two values
		// tmp3[0] = token[0]; tmp3[1] = token[1]; tmp3[2] = token[2];
		// // get the degrees
		// chTmp = (unsigned char)atoi(tmp3);
		// // make the degrees zero for minutes conversion
		// token[0]='0'; token[1]='0'; token [2] = '0';
		// // get the float
		// gpsControlData.lon.flData = degMinToDeg(chTmp,atof(token));
		
		// // 5.- Longitude Sector
		myTokenizer(NULL, ',', token);
		
		// if (strlen(token)>0) {
			// // set the sign of the float value
			// if (token[0] == 'S' || token[0] == 'W') {
				// gpsControlData.lon.flData = -gpsControlData.lon.flData;
			// }
		// }
	}
	
	// 6.- Quality Indicator
	myTokenizer(NULL, ',', token);
	// if (strlen(token) == 1) {
		// gpsControlData.fix = (char)atoi(token);
	// }

	// 7.- Sats used in solution
	// xx
	myTokenizer(NULL, ',', token);	
	if (strlen(token)>0) {
		gpsControlData.sats = (unsigned char) atoi(token);	
	}
	
	// 8.- Horizontal dilution of solution given from 0 to 99.99
	// xx.xx
	myTokenizer(NULL, ',', token);	
	if (strlen(token)>0) {
		gpsControlData.hdop.flData = atof(token);	
	}
	
	// 9.- Altitude above mean sea level given in meters
	// xxx.xxx
	myTokenizer(NULL, ',', token);	
	if (strlen(token)>0) {
		gpsControlData.alt.flData = atof(token);	
	}
	
	// turn the flag on of new data
	// gpsControlData.newData = 1;
}
