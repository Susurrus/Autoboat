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

#include "ecanDefinitions.h"
#include "types.h"

void ProcessAdcData(float voltage, float amperage)
{
	static uint8_t sequenceID = 0;
	
	// Set up a tCanMessage for transmitting all this data.
	// Set it as a data message with an extended frame type.
	tCanMessage msg;
	msg.message_type = CAN_MSG_DATA;
	msg.frame_type = CAN_FRAME_EXT;
	msg.buffer = 0;
	
	// 127508: Voltage, current, temperature
	msg.id = 127508;
	
	// Field 0: Battery instance
	msg.payload[0] = 0;
	
	// Field 1: Voltage (in .01V)
	tUnsignedShortToChar x;
	x.usData = (uint16_t)(100.0f * voltage);
	msg.payload[1] = x.chData[0];
	msg.payload[2] = x.chData[1];
	
	// Field 2: Current (in .1A)
	x.usData = (uint16_t)(10.0f * amperage);
	msg.payload[3] = x.chData[0];
	msg.payload[4] = x.chData[1];
	
	// Field 3: Temperature (in 1K)
	// All 1s indicated no-measurement
	msg.payload[5] = 0xFF;
	msg.payload[6] = 0xFF;
	
	// Field 4: Sequence ID
	msg.payload[7] = sequenceID++;
	
	// Finish it off with a payload length and send it.
	msg.validBytes = 8;
	ecan1_buffered_transmit(msg);
}
