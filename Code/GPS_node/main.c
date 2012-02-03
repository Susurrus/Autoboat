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

#include "uart2.h"
#include "gps.h"
#include "ecanDefinitions.h"
#include "types.h"
#include <inttypes.h>

// This is the value of the BRG register for configuring different baud
// rates at a 40MHz run rate
#define BAUD4800_BRG_REG 520
#define BAUD57600_BRG_REG 42
#define BAUD115200_BRG_REG 21

void InitCommunications(void)
{
	initUart2(BAUD4800_BRG_REG);
}

uint8_t SendGpsDataOverCan(void)
{
	if (GpsNewData()) {
		// Grab the latest GPS data.
		tGpsData gpsData;
		GetGpsData(&gpsData);
		
		// Set up a tCanMessage for transmitting all this data.
		tCanMessage msg;
		msg.message_type = CAN_MSG_DATA;
		msg.frame_type = CAN_FRAME_EXT;
		msg.buffer = 0;

		// CUSTOM MESSAGE: Package and transmit the system time
		msg.id = 126992;
		msg.payload[0] = gpsData.day;
		msg.payload[1] = gpsData.month;
		
		tUnsignedShortToChar x;
		x.usData = (uint16_t)gpsData.year;
		x.usData += 2000;
		msg.payload[2] = x.chData[0];
		msg.payload[3] = x.chData[1];
		
		msg.payload[4] = gpsData.hour;
		msg.payload[5] = gpsData.min;
		msg.payload[6] = gpsData.sec;
		msg.validBytes = 7;
		ecan1_buffered_transmit(msg);
		
		// PGN129025: Package and transmit the latitude/longitude
		msg.id = 129025;
		
		msg.payload[0] = gpsData.lat.chData[0];
		msg.payload[1] = gpsData.lat.chData[1];
		msg.payload[2] = gpsData.lat.chData[2];
		msg.payload[3] = gpsData.lat.chData[3];
		
		msg.payload[4] = gpsData.lon.chData[0];
		msg.payload[5] = gpsData.lon.chData[1];
		msg.payload[6] = gpsData.lon.chData[2];
		msg.payload[7] = gpsData.lon.chData[3];
		msg.validBytes = 8;
		ecan1_buffered_transmit(msg);
		
		// PGN129026: Package and transmit the course over ground and speed over ground
		msg.id = 129026;
		
		// Sequence ID
		msg.payload[0] = 0;
		
		// COG reference: true
		msg.payload[1] = 0;
		
		// Course over ground in 1e-4 rads as a uint16
		x.usData = (uint16_t)(gpsData.cog.flData * 10000);
		msg.payload[2] = x.chData[0];
		msg.payload[3] = x.chData[1];
		
		// Speed over ground in cm/s as a uint16
		x.usData = (uint16_t)(gpsData.sog.flData * 10000);
		msg.payload[4] = x.chData[0];
		msg.payload[5] = x.chData[1];
		msg.validBytes = 6;
		ecan1_buffered_transmit(msg);
		
		// CUSTOM MESSAGE: Package and transmit the horizontal dilution of precision and GPS status
		msg.id = 129539;
		
		msg.payload[0] = gpsData.fix;
		
		// Horizontal dilution of precision in cm.
		x.usData = (uint16_t)(gpsData.hdop.flData * 100);
		msg.payload[1] = x.chData[0];
		msg.payload[2] = x.chData[1];
		msg.validBytes = 3;
		ecan1_buffered_transmit(msg);
		
		return TRUE;
	}
	
	return FALSE;
}
