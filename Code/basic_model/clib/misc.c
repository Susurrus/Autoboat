#include "DEE.h"
#include "MissionManager.h"
#include <common/mavlink.h>
//#include <sealion/mavlink.h>

extern unsigned short rudderRange[2];
extern unsigned short throttleRange[2];
extern unsigned short trackRange[2];
extern unsigned char restoredCalibration;

/**
 * This function restored the calibrated range for the RC receiver PWM signals if any exist.
 * Since the EEPROM is initialized to 0xFFFF for each uint16 we only restore the values if
 * the memory locations that should contain our data just contain 0xFFFF instead. restoredCalibration
 * is another exported global that is just a Boolean for whether or not we restored saved data. This
 * is used to correct the calibration state used elsewhere.
 */
void initCalibrationRange() {
	unsigned short tmp;

	// Initialize rudder range
	if ((tmp = DataEERead(10)) != 0xFFFF) {
		rudderRange[0] = tmp;
		restoredCalibration = 1;
	}
	if ((tmp = DataEERead(11)) != 0xFFFF) {
		rudderRange[1] = tmp;
		restoredCalibration = 1;
	}

	// Initialize throttle range
	if ((tmp = DataEERead(12)) != 0xFFFF) {
		throttleRange[0] = tmp;
		restoredCalibration = 1;
	}
	if ((tmp = DataEERead(13)) != 0xFFFF) {
		throttleRange[1] = tmp;
		restoredCalibration = 1;
	}

	// Initialize track range
	if ((tmp = DataEERead(14)) != 0xFFFF) {
		trackRange[0] = tmp;
		restoredCalibration = 1;
	}
	if ((tmp = DataEERead(15)) != 0xFFFF) {
		trackRange[1] = tmp;
	}
}

