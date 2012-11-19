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

#include "Types.h"
#include "Uart1.h"
#include "DEE.h"
#include "ecanFunctions.h"
#include "CanMessages.h"
#include "Node.h"
#include "Rudder.h"

// Store some values for calibrating the RC transmitter.
uint16_t rcRudderRange[2];
uint16_t rcThrottleRange[2];
bool restoredCalibration;

typedef struct {
    unsigned enabled         : 1; // If the sensor is enabled, i.e. it is online and transmitting messages.
    unsigned enabled_counter : 7; // The timeout counter for this sensor being enabled.
    unsigned active          : 1; // If the sensor is active, i.e. receiving valid data.
    unsigned active_counter  : 7; // The timeout counter for this sensor being active.
} timeoutCounters;

struct stc {
    timeoutCounters prop; // The ACS300 outputs CAN messages quite frequently. It's enabled whenever one of these messages has been received within the last second and active when it's enabled and in run mode within the last second.
    timeoutCounters rudder; // The rudder controller outputs messages quite frequently also. It's enabled whenever one of these messages has been received within the last second. It's active when it's enabled and calibrated and done calibrating.
} sensorAvailability;

void RcNodeInit(void)
{
	nodeId = CAN_NODE_RC;
}

void TransmitStatus(void)
{
    tCanMessage msg;
    CanMessagePackageStatus(&msg, nodeId, status, errors);
    ecan1_buffered_transmit(&msg);
}

bool GetEstopStatus(void)
{
    return sensorAvailability.prop.enabled;
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

void UpdateSensorsAvailability(void)
{
    if (sensorAvailability.prop.enabled && sensorAvailability.prop.enabled_counter >= 100) {
        sensorAvailability.prop.enabled = false;
    } else if (!sensorAvailability.prop.enabled && sensorAvailability.prop.enabled_counter == 0) {
        sensorAvailability.prop.enabled = true;
    }
    if (sensorAvailability.prop.active && sensorAvailability.prop.active_counter >= 100) {
        sensorAvailability.prop.active = false;
    } else if (!sensorAvailability.prop.active && sensorAvailability.prop.active_counter == 0) {
        sensorAvailability.prop.active = true;
    }
    if (sensorAvailability.rudder.enabled && sensorAvailability.rudder.enabled_counter >= 100) {
        sensorAvailability.rudder.enabled = false;
    } else if (!sensorAvailability.rudder.enabled && sensorAvailability.rudder.enabled_counter == 0) {
        sensorAvailability.rudder.enabled = true;
    }
    if (sensorAvailability.rudder.active && sensorAvailability.rudder.active_counter >= 100) {
        sensorAvailability.rudder.active = false;
    } else if (!sensorAvailability.rudder.active && sensorAvailability.rudder.active_counter == 0) {
        sensorAvailability.rudder.active = true;
    }
}

uint8_t ProcessAllEcanMessages(void)
{
	uint8_t messagesLeft = 0;
	tCanMessage msg;

	uint8_t messagesHandled = 0;

	if (sensorAvailability.prop.enabled_counter < 100) {
		++sensorAvailability.prop.enabled_counter;
	}
	if (sensorAvailability.prop.active_counter < 100) {
		++sensorAvailability.prop.active_counter;
	}
	if (sensorAvailability.rudder.enabled_counter < 100) {
		++sensorAvailability.rudder.enabled_counter;
	}
	if (sensorAvailability.rudder.active_counter < 100) {
		++sensorAvailability.rudder.active_counter;
	}
        
	do {
		int foundOne = ecan1_receive(&msg, &messagesLeft);
		if (foundOne) {
			// Process throttle messages here. Anything not explicitly handled is assumed to be a NMEA2000 message.
			if (msg.id == 0x402) { // From the ACS300 TODO: Move into a separate library.
				sensorAvailability.prop.enabled_counter = 0;
				if ((msg.payload[6] & 0x40) == 0) { // Checks the status bit to determine if the ACS300 is enabled.
					sensorAvailability.prop.active_counter = 0;
				}
			} else if (msg.id == CAN_MSG_ID_RUDDER_DETAILS) { // From the rudder controller
                            CanMessageDecodeRudderDetails(&msg,
                                                          &rudderSensorData.RudderPotValue.usData,
                                                          &rudderSensorData.RudderPotLimitStarboard.usData,
                                                          &rudderSensorData.RudderPotLimitPort.usData,
                                                          &rudderSensorData.LimitHitStarboard,
                                                          &rudderSensorData.LimitHitPort,
                                                          &rudderSensorData.Enabled,
                                                          &rudderSensorData.Calibrated,
                                                          &rudderSensorData.Calibrating);
                            if (rudderSensorData.Enabled &&
                                rudderSensorData.Calibrated &&
                                !rudderSensorData.Calibrating) {
                                sensorAvailability.rudder.active_counter = 0; // Then it's active
                            }
                            sensorAvailability.rudder.enabled_counter = 0;
			}

			++messagesHandled;
		}
	} while (messagesLeft > 0);

	UpdateSensorsAvailability();

	return messagesHandled;
}