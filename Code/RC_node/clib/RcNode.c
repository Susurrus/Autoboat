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
// First Revision: Nov 10 2012
// ==============================================================

#include "types.h"
#include "uart1.h"
#include "DEE.h"

// Store some values for calibrating the RC transmitter.
uint16_t rcRudderRange[2];
uint16_t rcThrottleRange[2];
bool restoredCalibration;

// This is the value of the BRG register for configuring different baud
// rates. These BRG values have been calculated based on a 40MHz system clock.
#define BAUD115200_BRG_REG 21

/**
 * This function initializes UART1 to 115200baud for communications with a PC.
 * NOTE: This is really only useful for debugging.
 */
void InitUart(void)
{
	initUart1(BAUD115200_BRG_REG);
}

/**
 * This function restored the calibrated range for the RC receiver PWM signals if any exist.
 * Since the EEPROM is initialized to 0xFFFF for each uint16 we only restore the values if
 * the memory locations that should contain our data just contain 0xFFFF instead. restoredCalibration
 * is another exported global that is just a Boolean for whether or not we restored saved data. This
 * is used to correct the calibration state used elsewhere.
 */
void InitCalibrationRange()
{
	uint16_t tmp;

	// Initialize RC transmitter rudder range
	if ((tmp = DataEERead(10)) != 0xFFFF) {
		rcRudderRange[0] = tmp;
		restoredCalibration = true;
	}
	if ((tmp = DataEERead(11)) != 0xFFFF) {
		rcRudderRange[1] = tmp;
		restoredCalibration = true;
	}

	// Initialize RC transmitter throttle range
	if ((tmp = DataEERead(12)) != 0xFFFF) {
		rcThrottleRange[0] = tmp;
		restoredCalibration = true;
	}
	if ((tmp = DataEERead(13)) != 0xFFFF) {
		rcThrottleRange[1] = tmp;
		restoredCalibration = true;
	}
}

