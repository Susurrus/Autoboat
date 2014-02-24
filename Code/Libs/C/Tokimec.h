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
		int16_t yaw; // Absolute yaw in units of 2^-13 rads.
		int16_t pitch; // Absolute pitch in units of 2^-13 rads.
		int16_t roll; // Absolute roll in units of 2^-13 rads.
		int16_t x_angle_vel; // Angular velocity around the X axis in units of 2^-12 rads/s.
		int16_t y_angle_vel; // Angular velocity around the Y axis in units of 2^-12 rads/s.
		int16_t z_angle_vel; // Angular velocity around the Z axis in units of 2^-12 rads/s.
		int16_t x_accel; // Acceleration along the X axis (forward) in units of 2^-8 m/s^2.
		int16_t y_accel; // Acceleration along the Y axis (right) in units of 2^-8 m/s^2.
		int16_t z_accel; // Acceleration along the Z axis (up) in units of 2^-8 m/s^2.
		uint16_t gpsNoDataTime; // Time since last GPS hit, units are 2ms, maximum value is 2500
		int32_t est_latitude; // Calculated GPS latitude in units of 2^-29 radians.
		int32_t est_longitude; // Calculated GPS longitude in units of 2^-29 radians.
		int32_t latitude; // Raw GPS latitude in units of 2^-29 radians.
		int32_t longitude; // Raw GPS longitude in units of 2^-29 radians.
		int32_t altitude; // Raw GPS altitude in units of cm.
		int16_t velocity_n; // Velocity in the north direction in units of 2^-6 m/s.
		int16_t velocity_e; // Velocity in the east direction in units of 2^-6 m/s.
		int16_t velocity_d; // Velocity in the down direction in units of 2^-6 m/s.
		uint8_t gpsNum; // Number of GPS satellites being used
		GpsStatus gpsStatus; // GPS status.
		int16_t gpsDirection; // GPS course over ground in units of 2^-13 rads.
		int16_t gpsSpeed; // GPS speed in units of 2^-6 m/s.
		int16_t gpsHdop; // Horizontal degree of precision.
		int16_t magneticBearing; // Magnetic bearing in units of 2e-13 rads.
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
