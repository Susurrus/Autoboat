#include "Nvram.h"
#include "DEE.h"
#include "types.h"
#include "EcanRudder.h"

/**
 * Make constant the addresses for the various parameters stored in NVRAM
 */
enum {
	NVRAM_ADDR_CAL_STATUS = 3,
	NVRAM_ADDR_PORT_LIMIT_VAL,
	NVRAM_ADDR_STAR_LIMIT_VAL
};

/**
 * This function restored the calibrated range for the RC receiver PWM signals if any exist.
 * Since the EEPROM is initialized to 0xFFFF for each uint16 we only restore the values if
 * the memory locations that should contain our data just contain 0xFFFF instead. restoredCalibration
 * is another exported global that is just a Boolean for whether or not we restored saved data. This
 * is used to correct the calibration state used elsewhere.
 */
void InitRudderCalibrationRange(void) {
	uint16_t tmp;

	// If the rudder was in a calibrated state, restore those settings.
	if (DataEERead(NVRAM_ADDR_CAL_STATUS) == 0x0001) {
		rudderCalData.RestoredCalibration = true;
		rudderCalData.Calibrated = true;
		// Restore rudder pot limit
		if ((tmp = DataEERead(NVRAM_ADDR_PORT_LIMIT_VAL)) != 0xFFFF) {
			rudderCalData.PortLimitValue = tmp;
		}
		// Restore starboard pot limit
		if ((tmp = DataEERead(NVRAM_ADDR_STAR_LIMIT_VAL)) != 0xFFFF) {
			rudderCalData.StarLimitValue = tmp;
		}
		LATAbits.LATA3 = 1;
	}
}

void SaveRudderCalibrationRange(void) {
	// If the rudder was in a calibrated state, restore those settings.
	DataEEWrite(0x0001, NVRAM_ADDR_CAL_STATUS);
	DataEEWrite(rudderCalData.PortLimitValue, NVRAM_ADDR_PORT_LIMIT_VAL);
	DataEEWrite(rudderCalData.StarLimitValue, NVRAM_ADDR_STAR_LIMIT_VAL);
}



