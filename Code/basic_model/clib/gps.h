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

#ifndef __GPS_H__
#define __GPS_H__

#ifdef __cplusplus
       extern "C"{
#endif

#include "types.h"

typedef struct tGpsData {
	unsigned char	 		year;
	unsigned char			month;
	unsigned char			day;
	unsigned char			hour;
	unsigned char			min;
	unsigned char			sec;
	tFloatToChar 			lat;
	tFloatToChar 			lon;
	tFloatToChar 			alt;
	tFloatToChar			cog;
	tFloatToChar			sog;
	tFloatToChar			hdop;
	unsigned char			fix;
	unsigned char 			sats;
	unsigned char			newData; // Flag for whether this struct stores new data
} tGpsData;

/**
 * Initializes the GPS to only return RMC and GGA data
 */
void initGps();

void processGpsSentence(char *sentence);

/**
 * Pull new bytes from the UART2 receive buffer and
 * calls buildAndCheckSentence on each of them.
 */
void processNewGpsData();

/**
 * This is a Matlab helper function that returns the most recent 
 * GPS data in a large array that Matlab can handle.
 * @param data A pointer to a float array for storing the GPS data that was requested.
 */
void getGpsData(unsigned char* data);

/**
 * This function resets the entire GPS data struct to zeros.
 */
void clearGpsData();

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
       
#endif // __GPS_H__
