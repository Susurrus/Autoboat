#include "DEE.h"

extern unsigned short rcRudderRange[2];
extern unsigned short rcThrottleRange[2];
extern unsigned short jRudderRange[2];
extern unsigned short jThrottleRange[2];
extern unsigned char restoredCalibration;
extern unsigned char restoredJoystickCalibration;

/**
 * This function restored the calibrated range for the RC receiver PWM signals if any exist.
 * Since the EEPROM is initialized to 0xFFFF for each uint16 we only restore the values if
 * the memory locations that should contain our data just contain 0xFFFF instead. restoredCalibration
 * is another exported global that is just a Boolean for whether or not we restored saved data. This
 * is used to correct the calibration state used elsewhere.
 */
void initCalibrationRange() {
	unsigned short tmp;

	// Initialize RC transmitter rudder range
	if ((tmp = DataEERead(10)) != 0xFFFF) {
		rcRudderRange[0] = tmp;
		restoredCalibration = 1;
	}
	if ((tmp = DataEERead(11)) != 0xFFFF) {
		rcRudderRange[1] = tmp;
		restoredCalibration = 1;
	}

	// Initialize RC transmitter throttle range
	if ((tmp = DataEERead(12)) != 0xFFFF) {
		rcThrottleRange[0] = tmp;
		restoredCalibration = 1;
	}
	if ((tmp = DataEERead(13)) != 0xFFFF) {
		rcThrottleRange[1] = tmp;
		restoredCalibration = 1;
	}

	// Initialize RC transmitter rudder range
	if ((tmp = DataEERead(14)) != 0xFFFF) {
		jRudderRange[0] = tmp;
		restoredJoystickCalibration = 1;
	}
	if ((tmp = DataEERead(15)) != 0xFFFF) {
		jRudderRange[1] = tmp;
		restoredJoystickCalibration = 1;
	}

	// Initialize RC transmitter throttle range
	if ((tmp = DataEERead(16)) != 0xFFFF) {
		jThrottleRange[0] = tmp;
		restoredJoystickCalibration = 1;
	}
	if ((tmp = DataEERead(17)) != 0xFFFF) {
		jThrottleRange[1] = tmp;
		restoredJoystickCalibration = 1;
	}
}

