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

#ifndef _GPS_H_
#define _GPS_H_

#ifdef __cplusplus
       extern "C"{
#endif

#include <stdlib.h>
#include <string.h>

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

typedef struct tGpsData{
	unsigned char	 		year;
	unsigned char			month;
	unsigned char			day;
	unsigned char			hour;
	unsigned char			min;
	unsigned char			sec;	 
	tFloatToChar 			lat;
	tFloatToChar 			lon;
	tFloatToChar 			height;
	tUnsignedShortToChar	cog;
	tUnsignedShortToChar	sog;
	tUnsignedShortToChar	hdop;	
	unsigned char			fix;
	unsigned char 			sats;	
	unsigned char			newData; // Flag for whether this struct stores new data
}tGpsData;

#define KTS2MPS 		0.514444444

// GPS Circular Buffers
// ====================
#define MSIZE			150
#define CSIZE			26 //[newBytes payload remainingBytes]  (comms buffer out of readGPS)

/**
 * Converts a hexadecimal digit into its ascii equivalent.
 */
char hex2char(char halfhex);

/**
 * Converts degree-minutes to degrees.
 */
float degMinToDeg(unsigned char degrees, float minutes);

/**
 * Initializes the GPS to only return RMC and GGA data
 */
void initGps();

/**
 * Compiles GPS sentences one-byte at a time. Because of this
 * it is stateful. Once complete sentences are built, attempts
 * to parse and store in a tGpsData struct
 */
void buildAndCheckSentence(unsigned char characterIn);

/**
 * Pull new bytes from the UART2 receive buffer and
 * calls buildAndCheckSentence on each of them.
 */
void processNewGpsData();

/**
 * Computes the checksum for a given GPS sentence.
 */
unsigned char getChecksum(char* sentence, unsigned char size);

/**
 * This is a Matlab helper function that returns the most recent 
 * GPS data in a large array that Matlab can handle.
 * @param data A pointer to a float array for storing the GPS data that was requested.
 */
void getGpsMainData(float* data);

/**
 * A simple tokenizer. Similar to strtok(), but supports
 * multiple tokens in a row.
 */
unsigned char myTokenizer(char* stringToTokenize, char token, char * returnToken);

/**
 * Parses NMEA0183 RMC sentences. Results are stored in the
 * globally-declared gpsControlData struct.
 */
void parseRMC(char* stream);

/**
 * Parses NMEA0183 GGA sentences. Results are stored in the
 * globally-declared gpsControlData struct.
 */
void parseGGA(char* stream);

#ifdef __cplusplus
       }
#endif
       
#endif /* _GPS_H_ */
