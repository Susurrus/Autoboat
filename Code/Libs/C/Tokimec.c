#include <stdbool.h>
#include <stdio.h>

#include "Packing.h"
#include "Tokimec.h"

// Define pi here for ourselves since M_PI was removed from C99
#define PI 3.1415926535897932384626433832795

typedef enum {
	STATE_WAIT_HEADER_1 = 0,
	STATE_WAIT_HEADER_2,
	STATE_WAIT_FOOTER_1,
	STATE_WAIT_FOOTER_2,
	STATE_WAIT_CHECKSUM
} TokimecParseState;

// Each Tokimec RS232 data packet is 74 bytes.
static uint8_t tokimecDataBytes[74];

// Track the indices of all important data fields for the output data format
enum {
	OUT_BYTE_INDEX_STATUS = 3,
	OUT_BYTE_INDEX_YAW = 7,
	OUT_BYTE_INDEX_PITCH = 9,
	OUT_BYTE_INDEX_ROLL = 11,
	OUT_BYTE_INDEX_X_ANG_VEL = 13,
	OUT_BYTE_INDEX_Y_ANG_VEL = 15,
	OUT_BYTE_INDEX_Z_ANG_VEL = 17,
	OUT_BYTE_INDEX_X_ACCEL = 19,
	OUT_BYTE_INDEX_Y_ACCEL = 21,
	OUT_BYTE_INDEX_Z_ACCEL = 23,
	OUT_BYTE_INDEX_GPS_NO_DATA_TIME = 25,
	OUT_BYTE_INDEX_CALC_LAT = 27,
	OUT_BYTE_INDEX_CALC_LON = 31,
	OUT_BYTE_INDEX_LAT = 35,
	OUT_BYTE_INDEX_LON = 39,
	OUT_BYTE_INDEX_ALT = 43,
	OUT_BYTE_INDEX_VEL_N = 47,
	OUT_BYTE_INDEX_VEL_E = 49,
	OUT_BYTE_INDEX_VEL_U = 51,
	OUT_BYTE_INDEX_GPS_NUM = 53,
	OUT_BYTE_INDEX_GPS_STATUS = 54,
	OUT_BYTE_INDEX_GPS_DIR = 55,
	OUT_BYTE_INDEX_GPS_SPEED = 57,
	OUT_BYTE_INDEX_GPS_HDOP = 59,
	OUT_BYTE_INDEX_MAG_BEARING = 61,
	OUT_BYTE_INDEX_UTC_HOUR = 63,
	OUT_BYTE_INDEX_UTC_MINUTE = 64,
	OUT_BYTE_INDEX_UTC_SECOND = 65,
	// No data at 66
	OUT_BYTE_INDEX_UTC_DAY = 67,
	OUT_BYTE_INDEX_UTC_MONTH = 68,
	OUT_BYTE_INDEX_UTC_YEAR = 69,
	// No data at 70
	OUT_BYTE_INDEX_DLE = 71,
	OUT_BYTE_INDEX_ETX = 72,
	OUT_BYTE_INDEX_BCC = 73
};

// Track the indices of all important data fields for the input data format
enum {
	IN_BYTE_INDEX_DLE1 = 0,
	IN_BYTE_INDEX_STX = 1,
	IN_BYTE_INDEX_ID_CODE = 2,
	IN_BYTE_INDEX_COMMAND = 3,
	IN_BYTE_INDEX_ALIGN_TIME = 5,
	IN_BYTE_INDEX_DLE2 = 7,
	IN_BYTE_INDEX_ETX = 8,
	IN_BYTE_INDEX_BCC = 9
};

uint8_t _TokimecOutputChecksum(const uint8_t *data);
uint8_t _TokimecInputChecksum(const uint8_t *data);

bool TokimecParse(char in, TokimecOutput *outData)
{
	static uint8_t state = 0; // Store the current parsing state.
	static uint8_t index = 0; // Store the current byte index in the current TokimecOutput message

	// Full specification for NMEA0138 specifies a maximum sentence length
	// of 255 characters. We're going to ignore this for half the length as
	// we shouldn't get anything that big.

	// This contains the function's state of whether
	// it is currently building a sentence.
	// 0 - Awaiting start character ($)
	// 1 - Building sentence
	// 2 - Building first checksum character
	// 3 - Building second checksum character
	
	// We start recording a new sentence if we see a dollarsign.
	// The sentenceIndex is hard-set to 1 so that multiple dollar-signs
	// keep you at the beginning.
	if (state == STATE_WAIT_HEADER_1) {
		if (in == 0x10) {
			index = 0;
			state = STATE_WAIT_HEADER_2;
			tokimecDataBytes[index++] = in;
		}
	} else if (state == STATE_WAIT_HEADER_2) {
		if (in == 0x02) {
			state = STATE_WAIT_FOOTER_1;
			tokimecDataBytes[index++] = in;
		} else if (in == 0x10) {
			state = STATE_WAIT_HEADER_2;
			tokimecDataBytes[index++] = in;
		} else {
			state = STATE_WAIT_HEADER_1;
		}
	} else if (state == STATE_WAIT_FOOTER_1) {
		// Record every character that comes in now that we're building a message.
		if (index == 71) {
			if (in == 0x10) { // Catch the first footer byte.
				state = STATE_WAIT_FOOTER_2;
				tokimecDataBytes[index++] = in;
			} else {
				state = STATE_WAIT_HEADER_1;
				return -2;
			}
		} else {
			tokimecDataBytes[index++] = in;
		}
	} else if (state == STATE_WAIT_FOOTER_2) {
		// If we've found the next footer byte, move on to grabbing the checksum
		if (in == 0x03) {
			state = STATE_WAIT_CHECKSUM;
			tokimecDataBytes[index++] = in;
		}
		// Otherwise, we restart parsing for a new message 
		else {
			state = STATE_WAIT_HEADER_1;
			return -2;
		}
	} else if (state == STATE_WAIT_CHECKSUM) {
		tokimecDataBytes[index++] = in;

		uint8_t calculatedChecksum = _TokimecOutputChecksum(tokimecDataBytes);
		if (calculatedChecksum == tokimecDataBytes[OUT_BYTE_INDEX_BCC]) {
			BEUnpackUint16(&outData->status, &tokimecDataBytes[OUT_BYTE_INDEX_STATUS]);
			BEUnpackInt16(&outData->yaw, &tokimecDataBytes[OUT_BYTE_INDEX_YAW]);
			BEUnpackInt16(&outData->pitch, &tokimecDataBytes[OUT_BYTE_INDEX_PITCH]);
			BEUnpackInt16(&outData->roll, &tokimecDataBytes[OUT_BYTE_INDEX_ROLL]);
			BEUnpackInt16(&outData->x_angle_vel, &tokimecDataBytes[OUT_BYTE_INDEX_X_ANG_VEL]);
			BEUnpackInt16(&outData->y_angle_vel, &tokimecDataBytes[OUT_BYTE_INDEX_Y_ANG_VEL]);
			BEUnpackInt16(&outData->z_angle_vel, &tokimecDataBytes[OUT_BYTE_INDEX_Z_ANG_VEL]);
			BEUnpackInt16(&outData->x_accel, &tokimecDataBytes[OUT_BYTE_INDEX_X_ACCEL]);
			BEUnpackInt16(&outData->y_accel, &tokimecDataBytes[OUT_BYTE_INDEX_Y_ACCEL]);
			BEUnpackInt16(&outData->z_accel, &tokimecDataBytes[OUT_BYTE_INDEX_Z_ACCEL]);
			BEUnpackUint16(&outData->gpsNoDataTime, &tokimecDataBytes[OUT_BYTE_INDEX_GPS_NO_DATA_TIME]);
			BEUnpackInt32(&outData->est_latitude, &tokimecDataBytes[OUT_BYTE_INDEX_CALC_LAT]);
			BEUnpackInt32(&outData->est_longitude, &tokimecDataBytes[OUT_BYTE_INDEX_CALC_LON]);
			BEUnpackInt32(&outData->latitude, &tokimecDataBytes[OUT_BYTE_INDEX_LAT]);
			BEUnpackInt32(&outData->longitude, &tokimecDataBytes[OUT_BYTE_INDEX_LON]);
			BEUnpackInt32(&outData->altitude, &tokimecDataBytes[OUT_BYTE_INDEX_ALT]);
			BEUnpackInt16(&outData->velocity_n, &tokimecDataBytes[OUT_BYTE_INDEX_VEL_N]);
			BEUnpackInt16(&outData->velocity_e, &tokimecDataBytes[OUT_BYTE_INDEX_VEL_E]);
			BEUnpackInt16(&outData->velocity_u, &tokimecDataBytes[OUT_BYTE_INDEX_VEL_U]);
			outData->gpsNum = tokimecDataBytes[OUT_BYTE_INDEX_GPS_NUM];
			outData->gpsStatus = tokimecDataBytes[OUT_BYTE_INDEX_GPS_STATUS];
			BEUnpackInt16(&outData->gpsDirection, &tokimecDataBytes[OUT_BYTE_INDEX_GPS_DIR]);
			BEUnpackInt16(&outData->gpsSpeed, &tokimecDataBytes[OUT_BYTE_INDEX_GPS_SPEED]);
			BEUnpackInt16(&outData->gpsHdop, &tokimecDataBytes[OUT_BYTE_INDEX_GPS_HDOP]);
			BEUnpackInt16(&outData->magneticBearing, &tokimecDataBytes[OUT_BYTE_INDEX_MAG_BEARING]);
			outData->utcHour = tokimecDataBytes[OUT_BYTE_INDEX_UTC_HOUR];
			outData->utcMinute = tokimecDataBytes[OUT_BYTE_INDEX_UTC_MINUTE];
			outData->utcSecond = tokimecDataBytes[OUT_BYTE_INDEX_UTC_SECOND];
			outData->utcDay = tokimecDataBytes[OUT_BYTE_INDEX_UTC_DAY];
			outData->utcMonth = tokimecDataBytes[OUT_BYTE_INDEX_UTC_MONTH];
			outData->utcYear = tokimecDataBytes[OUT_BYTE_INDEX_UTC_YEAR];

			state = STATE_WAIT_HEADER_1;
			return 1;
		} else {
			state = STATE_WAIT_HEADER_1;
			return -1;
		}
	}
	return 0;
}

uint8_t _TokimecOutputChecksum(const uint8_t *data)
{
	uint8_t newChecksum = 0;
	int i;
	for (i = OUT_BYTE_INDEX_STATUS; i < OUT_BYTE_INDEX_DLE; ++i) {
		newChecksum += data[i];
	}
	newChecksum += data[OUT_BYTE_INDEX_ETX];
	return newChecksum;
}

/**
* Calculate the checksum for an input message. It's just a byte-wise sum of the command, alignment time, and ETX fields.
*/
uint8_t _TokimecInputChecksum(const uint8_t *data)
{
	uint8_t newChecksum = 0;
	for (int i = IN_BYTE_INDEX_COMMAND; i < IN_BYTE_INDEX_DLE2; ++i) {
		newChecksum += data[i];
	}
	newChecksum += data[IN_BYTE_INDEX_ETX];
	return newChecksum;
}

void TokimecPackageCommandMessage(char msg[10], uint16_t cmd, uint16_t atime)
{
	uint8_t msgTemplate[10] = {0x10, 0x02, 0x00, 0, 0, 0,  0, 0x10, 0x03, 0};
	BEPackUint16(&msgTemplate[IN_BYTE_INDEX_COMMAND], cmd);
	BEPackUint16(&msgTemplate[IN_BYTE_INDEX_ALIGN_TIME], atime);
	msgTemplate[IN_BYTE_INDEX_BCC] = _TokimecInputChecksum(msgTemplate);
	for (int i = 0; i < 10; ++i) {
		msg[i] = msgTemplate[i];
	}
}

void TokimecPrint(const TokimecOutput *data)
{
	printf("status: %u\n", data->status);
	if (data->status & TOKIMEC_STATUS_ALIGNMENT) {
            puts(" * Alignment: Complete");
        } else {
            puts(" * Alignment: In progress");
        }
        if (data->status & TOKIMEC_STATUS_MAG_COMPASS_MODE) {
            puts(" * Compass calibration: Complete");
        } else {
            puts(" * Compass calibration: In progress");
        }
        if (data->status & TOKIMEC_STATUS_MAG_COMPASS) {
            puts(" * Compass: Unused");
        } else {
            puts(" * Compass: Used");
        }
        if (data->status & TOKIMEC_STATUS_TEMP) {
            puts(" * Temperature sensor: Normal");
        } else {
            puts(" * Temperature sensor: Abnormal");
        }
        if (data->status & TOKIMEC_STATUS_ACCEL) {
            puts(" * Accelerometer: Normal");
        } else {
            puts(" * Accelerometer: Abnormal");
        }
        if (data->status & TOKIMEC_STATUS_GYRO) {
            puts(" * Gyro: Normal");
        } else {
            puts(" * Gyro: Abnormal");
        }
        if (data->status & TOKIMEC_STATUS_DETECTION) {
            puts(" * Detection: Normal");
        } else {
            puts(" * Detection: Abnormal");
        }
        if (data->status & TOKIMEC_STATUS_PROCESSOR) {
            puts(" * Processor: Normal");
        } else {
            puts(" * Processor: Abnormal");
        }
        if (data->status & TOKIMEC_STATUS_SENSOR) {
            puts(" * Sensors: Normal");
        } else {
            puts(" * Sensors: Abnormal");
        }
	printf("yaw: %f\n", (float)data->yaw/8192.0*180.0/PI);
	printf("pitch: %f\n", (float)data->pitch/8192.0*180.0/PI);
	printf("roll: %f\n", (float)data->roll/8192.0*180.0/PI);
	printf("Ang. vel (x): %f\n", (float)data->x_angle_vel/4096.0*180.0/PI);
	printf("Ang. vel (y): %f\n", (float)data->y_angle_vel/4096.0*180.0/PI);
	printf("Ang. vel (z): %f\n", (float)data->z_angle_vel/4096.0*180.0/PI);
	printf("Acceleration (x): %f\n", (float)data->x_accel/256.0);
	printf("Acceleration (y): %f\n", (float)data->y_accel/256.0);
	printf("Acceleration (z): %f\n", (float)data->z_accel/256.0);
	printf("GPS no data: %2.1fs\n", (float)data->gpsNoDataTime*.002);
	printf("Est. latitude: %f\n", (double)data->est_latitude/536870912.0*180.0/PI);
	printf("Est. longitude: %f\n", (double)data->est_longitude/536870912.0*180.0/PI);
	printf("GPS latitude: %f\n", (double)data->latitude/536870912.0*180.0/PI);
	printf("GPS longitude: %f\n", (double)data->longitude/536870912.0*180.0/PI);
	printf("GPS altitude: %.2fm\n", (float)data->altitude / 100.0);
	printf("Velocity (N): %f\n", (float)data->velocity_n/64.0);
	printf("Velocity (E): %f\n", (float)data->velocity_e/64.0);
	printf("Velocity (U): %f\n", (float)data->velocity_u/64.0);
	printf("GPS num: %u\n", data->gpsNum);
	if (data->gpsStatus == GPS_STATUS_OFF) {
		printf("GPS status: Off\n");
	} else if (data->gpsStatus == GPS_STATUS_PPGPS) {
		printf("GPS status: PPGPS\n");
	} else if (data->gpsStatus == GPS_STATUS_DGPS) {
		printf("GPS status: DGPS\n");
	}
	printf("GPS direction: %f\n", (float)data->gpsDirection/8192.0*180.0/PI);
	printf("GPS speed: %f\n", (float)data->gpsSpeed/64.0);
	printf("GPS HDOP: %f\n", (float)data->gpsHdop/256.0);
	printf("Mag. bearing: %f\n", (float)data->magneticBearing/8192.0*180/PI);
	printf("UTC time: %u:%u:%u\n", data->utcHour, data->utcMinute, data->utcSecond);
	printf("UTC date: %u:%u:%u\n", data->utcDay, data->utcMonth, data->utcYear);
}
