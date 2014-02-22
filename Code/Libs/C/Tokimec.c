#include <stdbool.h>
#include <stdio.h>

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

// Endianness swapping macros:
#define Swap2Bytes(val) ((((val) >> 8) & 0xFF) | (((val) << 8) & 0xFF00))

#define Swap4Bytes(val) \
 ( (((val) >> 24) & 0xFF) | (((val) >>  8) & 0xFF00) | \
   (((val) <<  8) & 0xFF0000) | (((val) << 24) & 0xFF000000) )

uint8_t _TokimecOutputChecksum(const TokimecOutput *data);
uint8_t _TokimecInputChecksum(const TokimecInput *data);

int TokimecParse(char in, TokimecOutput *data)
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
			data->bytes[index++] = in;
		}
	} else if (state == STATE_WAIT_HEADER_2) {
		if (in == 0x02) {
			state = STATE_WAIT_FOOTER_1;
			data->bytes[index++] = in;
		} else if (in == 0x10) {
			state = STATE_WAIT_HEADER_2;
			data->bytes[index++] = in;
		} else {
			state = STATE_WAIT_HEADER_1;
		}
	} else if (state == STATE_WAIT_FOOTER_1) {
		// Record every character that comes in now that we're building a message.
		if (index == 71) {
			if (in == 0x10) { // Catch the first footer byte.
				state = STATE_WAIT_FOOTER_2;
				data->bytes[index++] = in;
			} else {
				state = STATE_WAIT_HEADER_1;
				return -2;
			}
		} else {
			data->bytes[index++] = in;
		}
	} else if (state == STATE_WAIT_FOOTER_2) {
		// If we've found the next footer byte, move on to grabbing the checksum
		if (in == 0x03) {
			state = STATE_WAIT_CHECKSUM;
			data->bytes[index++] = in;
		}
		// Otherwise, we restart parsing for a new message 
		else {
			state = STATE_WAIT_HEADER_1;
			return -2;
		}
	} else if (state == STATE_WAIT_CHECKSUM) {
		data->bytes[index++] = in;

		uint8_t calculatedChecksum = _TokimecOutputChecksum(data);
		if (calculatedChecksum == data->nice.checksum) {
			// Now if we're running on a little-endian system, we need to reverse the byte-ordering for all multi-byte data types in the message.
			// This needs to be set for both the dsPIC33E and x86 architectures so we just hardcode it here.
			// FIXME: Change the following code to work for more systems.
#if defined __x86_64__ || defined __x86__ || 1
			data->nice.status = Swap2Bytes(data->nice.status);
			data->nice.counter = Swap2Bytes(data->nice.counter);
			data->nice.yaw = Swap2Bytes(data->nice.yaw);
			data->nice.pitch = Swap2Bytes(data->nice.pitch);
			data->nice.roll = Swap2Bytes(data->nice.roll);
			data->nice.x_angle_vel = Swap2Bytes(data->nice.x_angle_vel);
			data->nice.y_angle_vel = Swap2Bytes(data->nice.y_angle_vel);
			data->nice.z_angle_vel = Swap2Bytes(data->nice.z_angle_vel);
			data->nice.x_accel = Swap2Bytes(data->nice.x_accel);
			data->nice.y_accel = Swap2Bytes(data->nice.y_accel);
			data->nice.z_accel = Swap2Bytes(data->nice.z_accel);
			data->nice.gpsNoDataTime = Swap2Bytes(data->nice.gpsNoDataTime);
			data->nice.est_latitude = Swap4Bytes(data->nice.est_latitude);
			data->nice.est_longitude = Swap4Bytes(data->nice.est_longitude);
			data->nice.latitude = Swap4Bytes(data->nice.latitude);
			data->nice.longitude = Swap4Bytes(data->nice.longitude);
			data->nice.altitude = Swap4Bytes(data->nice.altitude);
			data->nice.velocity_n = Swap2Bytes(data->nice.velocity_n);
			data->nice.velocity_e = Swap2Bytes(data->nice.velocity_e);
			data->nice.velocity_d = Swap2Bytes(data->nice.velocity_d);
			data->nice.gpsDirection = Swap2Bytes(data->nice.gpsDirection);
			data->nice.gpsSpeed = Swap2Bytes(data->nice.gpsSpeed);
			data->nice.gpsHdop = Swap2Bytes(data->nice.gpsHdop);
			data->nice.magneticBearing = Swap2Bytes(data->nice.magneticBearing);
#endif
			state = STATE_WAIT_HEADER_1;
			return 1;
		} else {
			state = STATE_WAIT_HEADER_1;
			return -1;
		}
	}
	return 0;
}

uint8_t _TokimecOutputChecksum(const TokimecOutput *data)
{
	uint8_t newChecksum = 0;
	for (int i = 3; i < 71; ++i) {
		newChecksum += data->bytes[i];
	}
	newChecksum += data->nice.etx;
	return newChecksum;
}

uint8_t _TokimecInputChecksum(const TokimecInput *data)
{
	uint8_t newChecksum = 0;
	for (int i = 3; i < 7; ++i) {
		newChecksum += data->bytes[i];
	}
	newChecksum += data->nice.etx;
	return newChecksum;
}

void TokimecPackageCommandMessage(char msg[10], uint16_t cmd, uint16_t atime)
{
	TokimecInput s = {.nice = {0x10, 0x02, 0x00, 0, 0, 0x10, 0x03, 0}};
	s.nice.command = cmd;
	s.nice.alignmentTime = atime;
	s.nice.checksum = _TokimecInputChecksum(&s);
	for (int i = 0; i < 10; ++i) {
		msg[i] = s.bytes[i];
	}
}

void TokimecPrint(const TokimecOutput *data)
{
	printf("Header: 0x%02x 0x%02x\n", data->nice.dle, data->nice.stx);
	printf("ID: %u\n", data->nice.id);
	printf("status: %u\n", data->nice.status);
	printf("counter: %u\n", data->nice.counter);
	printf("yaw: %f\n", (float)data->nice.yaw/8192.0*180.0/PI);
	printf("pitch: %f\n", (float)data->nice.pitch/8192.0*180.0/PI);
	printf("roll: %f\n", (float)data->nice.roll/8192.0*180.0/PI);
	printf("Ang. vel (x): %f\n", (float)data->nice.x_angle_vel/4096.0*180.0/PI);
	printf("Ang. vel (y): %f\n", (float)data->nice.y_angle_vel/4096.0*180.0/PI);
	printf("Ang. vel (z): %f\n", (float)data->nice.z_angle_vel/4096.0*180.0/PI);
	printf("Acceleration (x): %f\n", (float)data->nice.x_accel/256.0);
	printf("Acceleration (y): %f\n", (float)data->nice.y_accel/256.0);
	printf("Acceleration (z): %f\n", (float)data->nice.z_accel/256.0);
	printf("GPS no data: %2.1fs\n", (float)data->nice.gpsNoDataTime*.002);
	printf("Est. latitude: %f\n", (double)data->nice.est_latitude/536870912.0*180.0/PI);
	printf("Est. longitude: %f\n", (double)data->nice.est_longitude/536870912.0*180.0/PI);
	printf("GPS latitude: %f\n", (double)data->nice.latitude/536870912.0*180.0/PI);
	printf("GPS longitude: %f\n", (double)data->nice.longitude/536870912.0*180.0/PI);
	printf("GPS altitude: %.2fm\n", (float)data->nice.altitude / 100.0);
	printf("Velocity (N): %f\n", (float)data->nice.velocity_n/64.0);
	printf("Velocity (E): %f\n", (float)data->nice.velocity_e/64.0);
	printf("Velocity (D): %f\n", (float)data->nice.velocity_d/64.0);
	printf("GPS num: %u\n", data->nice.gpsNum);
	if (data->nice.gpsStatus == GPS_STATUS_OFF) {
		printf("GPS status: Off\n");
	} else if (data->nice.gpsStatus == GPS_STATUS_PPGPS) {
		printf("GPS status: PPGPS\n");
	} else if (data->nice.gpsStatus == GPS_STATUS_DGPS) {
		printf("GPS status: DGPS\n");
	}
	printf("GPS direction: %f\n", (float)data->nice.gpsDirection/8192.0*180.0/PI);
	printf("GPS speed: %f\n", (float)data->nice.gpsSpeed/64.0);
	printf("GPS HDOP: %f\n", (float)data->nice.gpsHdop/256.0);
	printf("Mag. bearing: %f\n", (float)data->nice.magneticBearing/8192.0*180/PI);
	printf("UTC time: %u:%u:%u\n", data->nice.utcHours, data->nice.utcMinutes, data->nice.utcSeconds);
	printf("UTC date: %u:%u:%u\n", data->nice.utcDay, data->nice.utcMonth, data->nice.utcYear);
	printf("Footer: 0x%02x 0x%02x\n", data->nice.dle2, data->nice.etx);
	printf("Checksum: 0x%02x\n", data->nice.checksum);
}
