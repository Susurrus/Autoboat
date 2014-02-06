/**
 * This file implements the serial library for the Tokimec VSAS-2GM IMU.
 */
#ifndef TOKIMEC_H
#define TOKIMEC_H

#include <stdint.h>

// Flags for the 16-bit status field in the output message.
typedef enum {
	TOKIMEC_STATUS_ALIGNMENT = 1 << 5, // False => alignment in progress, true => alignment complete
	TOKIMEC_STATUS_MAG_COMPASS_MODE = 1 << 6, // False => calibration in progress, true => calibration complete
	TOKIMEC_STATUS_MAG_COMPASS = 1 << 7, // False => compass is active, True => compass unused
	TOKIMEC_STATUS_TEMP = 1 << 9, // True => Temp sensor normal, False => abnormal
	TOKIMEC_STATUS_ACCEL = 1 << 10, // True => Accelerometer operating normally, False => abnormal
	TOKIMEC_STATUS_GYRO = 1 << 11, // True => Gyroscope operating normally, False => abnormal
	TOKIMEC_STATUS_DETECTION = 1 << 12, // True => Detection circuit operating normally, False => abnormal
	TOKIMEC_STATUS_PROCESSOR = 1 << 13 // True => Processor monitor operating normally, False => abnormal
} TokimecStatusFlags;

// Flags for the GPS status field in the output message.
typedef enum __attribute__ ((__packed__)) {
	GPS_STATUS_OFF   = 0,
	GPS_STATUS_PPGPS = 1,
	GPS_STATUS_DGPS  = 2
} GpsStatus;

// Flags for the command bitfield in the input message.
typedef enum {
	TOKIMEC_COMMAND_RESTART           = 1 << 0,
	TOKIMEC_COMMAND_END_ALIGNMENT     = 1 << 1,
	TOKIMEC_COMMAND_CALIBRATE_COMPASS = 1 << 2,
	TOKIMEC_COMMAND_DISABLE_COMPASS   = 1 << 3
} TokimecCommandFlags;

typedef union {
	struct __attribute__ ((__packed__)) a {
		uint8_t dle; // Set to 0x10
		uint8_t stx; // Set to 0x2
		uint8_t id;  // Set to 0xFF
		uint16_t status; // Bitfield of status bits. See TokimecStatusFlags enum.
		uint16_t counter; // Incremented for each packet.
		int16_t yaw;
		int16_t pitch;
		int16_t roll;
		int16_t x_angle_vel;
		int16_t y_angle_vel;
		int16_t z_angle_vel;
		int16_t x_accel;
		int16_t y_accel;
		int16_t z_accel;
		uint16_t gpsNoDataTime; // Time since last GPS hit, units are 2ms, maximum value is 2500
		int32_t est_latitude;
		int32_t est_longitude;
		int32_t latitude;
		int32_t longitude;
		int32_t altitude;
		int16_t velocity_n;
		int16_t velocity_e;
		int16_t velocity_d;
		uint8_t gpsNum;
		GpsStatus gpsStatus;
		int16_t gpsDirection;
		int16_t gpsSpeed;
		int16_t gpsHdop;
		int16_t magneticBearing;
		uint8_t utcHours;
		uint8_t utcMinutes;
		uint8_t utcSeconds;
		uint8_t utcTimeNone; // Set to 0.
		uint8_t utcDay;
		uint8_t utcMonth;
		uint8_t utcYear;
		uint8_t utcDateNone;
		uint8_t dle2; // Set to 0x10
		uint8_t etx; // Set to 0x3
		uint8_t checksum; // Sum of status-through-utcDateNone and etx.
	} nice;
	uint8_t bytes[sizeof(struct a)]; 
} TokimecOutput;

typedef union {
	struct __attribute__ ((__packed__)) b {
		uint8_t dle; // Set to 0x10
		uint8_t stx; // Set to 0x02
		uint8_t id;  // Set to 0x00
		uint16_t command; // Bitfield of TokimecCommandFlags
		uint16_t alignmentTime; // Units are 's'
		uint8_t dle2; // Set to 0x10
		uint8_t etx; // Set to 0x3
		uint8_t checksum; // Sum of command, alignmentTime, and etx.
	} nice;
	uint8_t bytes[sizeof(struct b)];
} TokimecInput;

int TokimecParse(char in, TokimecOutput *data);

void TokimecPackageCommandMessage(char msg[10], uint16_t cmd, uint16_t atime);

void TokimecPrint(const TokimecOutput *data);

#endif // TOKIMEC_H
