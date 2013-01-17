#include "nmea2000.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.1415926535
#endif

uint32_t Iso11783Decode(uint32_t can_id, uint8_t *src, uint8_t *dest, uint8_t *pri)
{

	// The source address is the lowest 8 bits
	if (src) {
		*src = (uint8_t)can_id;
	}

	// The priority are the highest 3 bits
	if (pri) {
		*pri = (uint8_t)((can_id >> 26) & 7);
	}

	// Most significant byte
	uint32_t MS = (can_id >> 24) & 0x03;

	// PDU format byte
	uint32_t PF = (can_id >> 16) & 0xFF;

	// PDU specific byte
	uint32_t PS = (can_id >> 8) & 0xFF;

	uint32_t pgn;
	if (PF > 239) {
		// PDU2 format, the destination is implied global and the PGN is extended.
		if (dest) {
			*dest = 0xFF;
		}
		pgn = (MS << 16) | (PF << 8) | (PS);
	} else {
		// PDU1 format, the PDU Specific field contains the destination address.
		if (dest) {
			*dest = PS;
		}
		pgn = (MS << 16) | (PF << 8);
	}

	return pgn;
}

uint32_t Iso11783Encode(uint32_t pgn, uint8_t src, uint8_t dest, uint8_t pri)
{
	uint32_t can_id = 0;

	// The source address is the lowest 8 bits
	can_id |= src;

	// The priority is the highest 3 bits
	can_id |= ((uint32_t)pri & 0x07) << 26;

	// Note that both the reserved bit and data page bit are left as 0 according to the protocol.

	// The following depends on if it's a PDU1 or PDU2 message. This can be determined
	// by the destination. 255 implies a global message with an extended PGN as PDU2, otherwise
	// it's PDU1.
	// For PDU2
	if (pgn & 0xFF) {
		can_id |= (pgn & 0x7FFFF) << 8;
	}
	// For PDU1
	else {
		can_id |= ((pgn & 0x7FF00) | (uint32_t)dest) << 8;
	}

	return can_id;
}

void DaysSinceEpochToOffset(uint16_t days, uint8_t *offset_years, uint8_t *offset_months, uint8_t *offset_days)
{
	static const float quad_year = 365 + 365 + 366 + 365;
	static const uint16_t year_lengths[] = {365, 365, 366, 365};
	uint8_t month_lengths[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

	// Modulo out the number of days into the current year we are. This also calculates
	// what year offset we are.
	uint8_t years_tmp = 4 * floorf(((float)days) / quad_year);
	uint16_t days_tmp = fmodf((float)days, quad_year);
	uint8_t i = 0;
	while (days_tmp >= year_lengths[i]) {
		days_tmp -= year_lengths[i];
		++i;
		++years_tmp;
	}

	// If it's a leap year, account for that in the number of days in February
	if (i == 2) {
		month_lengths[1] = 29;
	}

	// Return the offset years.
	if (offset_years) {
		*offset_years = years_tmp;
	}

	// Modulo out the number of months along with the exact number of days
	i = 0;
	uint8_t months_tmp = 0;
	while (days_tmp >= month_lengths[i]) {
		days_tmp -= month_lengths[i];
		++i;
		++months_tmp;
	}

	// Return the months value
	if (offset_months) {
		*offset_months = months_tmp;
	}

	// Return the days value
	if (offset_days) {
		*offset_days = days_tmp;
	}
}

uint8_t ParsePgn126992(const uint8_t data[8], uint8_t *seqId, uint8_t *source, uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *minute, uint8_t *second, uint64_t *usecSinceEpoch)
{
	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each PGN field.
	uint8_t fieldStatus = 0;

	// Field 0: Sequence ID. Links data together across PGNs that occured at the same timestep. If the sequence ID is 255, it's invalid.
	if (seqId && data[0] != 0xFF) {
		*seqId = data[0];
		fieldStatus |= 0x01;
	}

	// Field 1: Source, 4-bit enumeration. This field describes the source of this date/time:
	// 0 (GPS),
	// 1 (GLONASS),
	// 2 (Radio station),
	// 3 (Local cesium clock),
	// 4 (local rubidium clock),
	// 5 (Local crystal clock).
	if (source && data[1] != 0xFF) {
		*source = data[1] & 0x0F;
		fieldStatus |= 0x02;
	}

	// Field 2: Date in days since Jan 1 1970.
	// This field can be invalid if all 1's.
	if (data[2] != 0xFF || data[3] != 0xFF) {
		uint16_t days = (uint16_t)data[2] | ((uint16_t)data[3] << 8);

		// Obtain the offset from epoch based on the number of days
		// I use local variables here only for consistency with the yearOffset variable.
		uint8_t yearOffset;
		uint8_t monthOffset;
		uint8_t dayOffset;
		DaysSinceEpochToOffset(days, &yearOffset, &monthOffset, &dayOffset);

		// Add the offsets to that these are the actual date
		if (year) {
			*year = 1970 + (uint16_t)yearOffset;
			fieldStatus |= 0x04;
		}
		if (month) {
			*month = 1 + monthOffset;
			fieldStatus |= 0x08;
		}
		if (day) {
			*day = 1 + dayOffset;
			fieldStatus |= 0x10;
		}
	}

	// Field 3: Seconds since midnight in units of 1e-4 second.
	if (data[4] != 0xFF || data[5] != 0xFF || data[6] != 0xFF || data[7] != 0xFF) {
		uint32_t x = data[4];
		x |= (uint32_t)data[5] << 8;
		x |= (uint32_t)data[6] << 8;
		x |= (uint32_t)data[7] << 8;

		uint32_t seconds = x / 1e4;
		if (hour) {
			*hour = seconds / 3600;
			fieldStatus |= 0x20;
		}
		seconds %= 3600;

		if (minute) {
			*minute = seconds / 60;
			fieldStatus |= 0x40;
		}
		seconds %= 60;

		if (second) {
			*second = seconds;
			fieldStatus |= 0x80;
		}

		if (usecSinceEpoch) {
			*usecSinceEpoch = x * 100;
		}
	}

	return fieldStatus;
}

uint8_t ParsePgn127245(const uint8_t data[8], uint8_t *instance, uint8_t *direction, float *angleOrder, float *position)
{

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each PGN field.
	uint8_t fieldStatus = 0;

	// Field 0: Instance. This field represents the rudder instance
	if (instance && (data[0] != 0xFF)) {
		*instance = data[0];
		fieldStatus |= 0x02;
	}

	// Field 1: Direction Order: 2-bit field, used to tell the direction.
	if (direction && ((data[1] & 0xC0) != 0xC0)) {
		*direction = (data[1] & 0xC0) >> 6;
		fieldStatus |= 0x04;
	}
	// Field 2: Angle Order. This is a 16-bit field that is used to command rudder angles. This field contains a signed value with the units of 0.0001 radians.
	if (angleOrder && (data[2] != 0xFF || data[3] != 0xFF)) {
		int x = (int)data[2] | ((int)data[3] << 8);
		*angleOrder = (float)x / 10000.0;
		fieldStatus |= 0x08;
	}

	//Field 3: Position. This is a 16-bit field that represents the current rudder angle. A value of all 1s (65535) means that the angle cannot be measured. This field contains a signed value with the units of 0.0001 radians.
	if (position && (data[4] != 0xFF || data[5] != 0xFF)) {
		int x = (int)data[4] | ((int)data[5] << 8);
		*position = (float)x / 10000.0;
		fieldStatus |= 0x10;
	}

	return fieldStatus;
}

uint8_t ParsePgn127508(const uint8_t data[8], uint8_t *seqId, uint8_t *instance, float *voltage, float *current, float *temperature)
{
	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each PGN field in the same order as the output arguments to this function.
	uint8_t fieldStatus = 0;

	// N2K Field 4: Sequence ID. Links data together across PGNs that occured at the same timestep. If the sequence ID is 255, it's invalid.
	if (seqId) {
		if (data[7] != 0xFF) {
			*seqId = data[7];
			fieldStatus |= 0x01;
		}
	}

	// N2K Field 0: Battery instance.
	if (instance) {
		if (data[0] != 0xFF) {
			*instance = data[0];
			fieldStatus |= 0x02;
		}
	}

	// N2K Field 1: Voltage. Comes in in units of .01V
	if (voltage) {
		// All 1s in NMEA2000 signifies invalid data.
		if (data[1] != 0xFF || data[2] != 0xFF) {
			uint16_t x = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
			*voltage = (float)x / 100.0;
			fieldStatus |= 0x04;
		}
	}

	// N2K Field 2: Current. Comes in in units of .1A.
	if (current) {
		// All 1s in NMEA2000 signifies invalid data.
		if (data[3] != 0xFF || data[4] != 0xFF) {
			uint16_t x = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
			*current = (float)x / 10.0;
			fieldStatus |= 0x08;
		}
	}

	// N2K Field 3: Temperature. Comes in in units of .01K.
	if (temperature) {
		// All 1s in NMEA2000 signifies invalid data.
		if (data[5] != 0xFF || data[6] != 0xFF) {
			uint16_t x = (uint16_t)data[5] | ((uint16_t)data[6] << 8);
			*temperature = (float)x / 100.0 - 273.15;
			fieldStatus |= 0x10;
		}
	}

	return fieldStatus;
}

uint8_t ParsePgn128259(const uint8_t data[8], uint8_t *seqId, float *waterSpeed)
{

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each PGN field.
	uint8_t fieldStatus = 0;

	// Field 0: Sequence ID. Links data together across PGNs that occured at the same timestep. If the sequence ID is 255, it's invalid.
	if (seqId && data[0] != 0xFF) {
		*seqId = data[0];
		fieldStatus |= 0x01;
	}

	// Field 1: Water speed. Raw units are centimeters/second. Converted to meters/second for output.
	if (waterSpeed && (data[1] != 0xFF || data[2] != 0xFF)) {
		uint16_t x = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
		*waterSpeed = (float)x / 100.0;
		fieldStatus |= 0x02;
	}

	return fieldStatus;
}

uint8_t ParsePgn128267(const uint8_t data[8], uint8_t *seqId, float *waterDepth, float *offset)
{

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each PGN field.
	uint8_t fieldStatus = 0;

	// Field 0: Sequence ID (8-bits). Links data together across PGNs that occured at the same timestep. If the sequence ID is 255, it's invalid.
	if (seqId && data[0] != 0xFF) {
		*seqId = data[0];
		fieldStatus |= 0x01;
	}

	// Field 1: Water depth (8-bits). Raw units are centimeters. Converted to meters for output.
	if (waterDepth && (data[1] != 0xFF || data[2] != 0xFF)) {
		uint16_t x = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
		*waterDepth = (float)x / 100.0;
		fieldStatus |= 0x02;
	}

	// Field 2: Water depth offset (8-bits). Raw units are centimeters. Converted to meters for output.
	if (offset && (data[5] != 0xFF || data[6] != 0xFF)) {
		uint16_t x = (uint16_t)data[5] | ((uint16_t)data[6] << 8);
		*offset = (float)x * .01;
		fieldStatus |= 0x04;
	}

	return fieldStatus;
}

uint8_t ParsePgn129025(const uint8_t data[8], int32_t *latitude, int32_t *longitude)
{

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each PGN field.
	uint8_t fieldStatus = 0;

	// Field 0: Latitude (32-bits). Raw units are 1e-7 degrees, but they're converted to raw radians on output.
	if (latitude && (data[0] != 0xFF || data[1] != 0xFF || data[2] != 0xFF || data[3] != 0x7F)) {
		int32_t x = data[0];
		x |= (int32_t)data[1] << 8;
		x |= (int32_t)data[2] << 8;
		x |= (int32_t)data[3] << 8;
		*latitude = x;
		fieldStatus |= 0x01;
	}

	// Field 1: Longitude (32-bits). Units are 1e-7 degrees.
	if (longitude && (data[4] != 0xFF || data[5] != 0xFF || data[6] != 0xFF || data[7] != 0x7F)) {
		int32_t x = data[4];
		x |= (int32_t)data[5] << 8;
		x |= (int32_t)data[6] << 8;
		x |= (int32_t)data[7] << 8;
		*longitude = x;
		fieldStatus |= 0x02;
	}

	return fieldStatus;
}

uint8_t ParsePgn129026(const uint8_t data[8], uint8_t *seqId, uint8_t *cogRef, uint16_t *cog, uint16_t *sog)
{

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each PGN field.
	uint8_t fieldStatus = 0;

	// Field 0: Sequence ID (8-bits). Links data together across PGNs that occured at the same timestep. If the sequence ID is 255, it's invalid.
	if (seqId && data[0] != 0xFF) {
		*seqId = data[0];
		fieldStatus |= 0x01;
	}

	// Field 1: Course over ground reference (2-bits). A 0 for True reference and a 1 for a magnetic field reference.
	if (cogRef && data[1] != 0xFF) {
		*cogRef = data[1] & 0x03;
		fieldStatus |= 0x02;
	}

	// 6-bits reserved

	// Field 2: Course over ground (16-bits). Units are .0001 degrees eastward from north.
	if (cog && (data[2] != 0xFF || data[3] != 0xFF)) {
		*cog = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
		fieldStatus |= 0x04;
	}

	// Field 3: Speed over ground (16-bits). Units are .01 m/s.
	if (sog && (data[4] != 0xFF || data[5] != 0xFF)) {
		*sog = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
		fieldStatus |= 0x08;
	}

	// Last 16-bits reserved

	return fieldStatus;
}

uint8_t ParsePgn130306(const uint8_t data[8], uint8_t *seqId, float *airSpeed, float *direction)
{

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each PGN field.
	uint8_t fieldStatus = 0;

	// Field 0: Sequence ID. Links data together across PGNs that occured at the same timestep. If the sequence ID is 255, it's invalid.
	if (seqId && data[0] != 0xFF) {
		*seqId = data[0];
		fieldStatus |= 0x01;
	}

	// Field 1: Wind speed. Message units are cm/s but are converted to m/s on output.
	if (airSpeed && (data[1] != 0xFF || data[2] != 0xFF)) {
		uint16_t x = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
		*airSpeed = (float)x / 100.0;
		fieldStatus |= 0x02;
	}

	// Field 2: Wind direction. Message units are e-4 rads but are converted to raw radians on output.
	if (direction && (data[3] != 0xFF || data[4] != 0xFF)) {
		uint16_t x = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
		*direction = (float)x * .0001;
		fieldStatus |= 0x04;
	}

	return fieldStatus;
}

uint8_t ParsePgn130310(const uint8_t data[8], uint8_t *seqId, float *waterTemp, float *airTemp, float *airPressure)
{

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each PGN field.
	uint8_t fieldStatus = 0;

	// Field 0: Sequence ID. Links data together across PGNs that occured at the same timestep. If the sequence ID is 255, it's invalid.
	if (seqId && data[0] != 0xFF) {
		*seqId = data[0];
		fieldStatus |= 0x01;
	}

	// Water temperature data. Read in as centiKelvin and converted to Celsius.
	// A value of 0xFFFF implies invalid data.
	if (waterTemp && (data[1] != 0xFF || data[2] != 0xFF)) {
		uint16_t x = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
		*waterTemp = (float)x / 100.0 - 273.15;
		fieldStatus |= 0x02;
	}

	// Air temperature data. Read in as centiKelvin and converted to Celsius.
	// A value of 0xFFFF implies invalid data.
	if (airTemp && (data[3] != 0xFF || data[4] != 0xFF)) {
		uint16_t x = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
		*airTemp = (float)x / 100.0 - 273.15;
		fieldStatus |= 0x04;
	}

	// Air pressure data. Read in as hectoPascals and converted to kiloPascals.
	// A value of 0xFFFF implies invalid data.
	if (airPressure && (data[5] != 0xFF || data[6] != 0xFF)) {
		uint16_t x = (uint16_t)data[5] | ((uint16_t)data[6] << 8);
		*airPressure = (float)x * 0.1;
		fieldStatus |= 0x08;
	}

	return fieldStatus;
}

uint8_t ParsePgn130311(const uint8_t data[8], uint8_t *seqId, uint8_t *tempInstance, uint8_t *humidityInstance, float *temp, float *humidity, float *pressure)
{

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each PGN field.
	uint8_t fieldStatus = 0;

	// N2K Field 0: Sequence ID. Links data together across PGNs that occured at the same timestep. If the sequence ID is 255, it's invalid.
	if (seqId && data[0] != 0xFF) {
		*seqId = data[0];
		fieldStatus |= 0x01;
	}

	// N2K Field 1: Temperature instance.
	// 0 - inside
	// 1 - outside
	// 2 - inside
	// 3 - engine room
	// 4 - main cabin
	if (tempInstance && ((data[1] & 0xFC) != 0xFC)) {
		*tempInstance = (data[1] & 0xFC) >> 2;
		fieldStatus |= 0x02;
	}

	// N2K Field 2: Humidity instance
	// 0 - inside
	// 1 - outside
	if (humidityInstance && ((data[1] & 0x03) != 0x03)) {
		*humidityInstance = data[1] & 0x03;
		fieldStatus |= 0x04;
	}

	// N2K Field 3: Air temperature. Read in as centiKelvin and converted to Celsius
	if (temp && (data[2] != 0xFF || data[3] != 0xFF)) {
		uint16_t x = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
		*temp = (float)x / 100.0 - 273.15;
		fieldStatus |= 0x08;
	}

	// N2K Field 4: Humidity data. Read in as units of .0004%, output in percent.
	if (humidity && (data[4] != 0xFF || data[5] != 0xFF)) {
		uint16_t x = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
		*humidity = (float)x * 0.004;
		fieldStatus |= 0x10;
	}

	// N2K Field 5: Pressure data. Read in as hectoPascals and converted to kiloPascals.
	if (pressure && (data[6] != 0xFF || data[7] != 0xFF)) {
		uint16_t x = (uint16_t)data[6] | ((uint16_t)data[7] << 8);
		*pressure = (float)x * 0.1;
		fieldStatus |= 0x20;
	}

	return fieldStatus;
}

#ifdef UNIT_TEST_NMEA2000

#include <stdio.h>
#include <assert.h>

// Define an epsilon for comparison equality of floating-point numbers
#define EPSILON 20e-4

int main(void)
{

	/** Test Iso11783Decode **/
	{
		// Test output from Maretron GPS200 with PGN 129025, priority 2, source 32.
		uint32_t id = 0x09F80120;
		uint8_t src;
		uint8_t dest;
		uint8_t pri;
		assert(Iso11783Decode(id, &src, &dest, &pri) == 129025);
		assert(src == 32);
		assert(dest == 255);
		assert(pri == 2);

		// Test output from Maretron GPS200 with PGN 129026, priority 2, source 32.
		id = 0x09F80220;
		assert(Iso11783Decode(id, &src, &dest, &pri) == 129026);
		assert(src == 32);
		assert(dest == 255);
		assert(pri == 2);

		// Test output from Maretron GPS200 with PGN 126992, priority 3, source 32.
		id = 0x0DF01020;
		assert(Iso11783Decode(id, &src, &dest, &pri) == 126992);
		assert(src == 32);
		assert(dest == 255);
		assert(pri == 3);

		// Test output from Maretron GPS200 with PGN 129540, priority 7, source 32.
		id = 0x19FA0420;
		assert(Iso11783Decode(id, &src, &dest, &pri) == 129540);
		assert(src == 32);
		assert(dest == 255);
		assert(pri == 6);

		// Test broadcast from Maretron WSO100 with PGN 59904 "ISO Request", priority 6, source 81.
		id = 0x18EAFF51;
		assert(Iso11783Decode(id, &src, &dest, &pri) == 59904);
		assert(src == 81);
		assert(dest == 255);
		assert(pri == 6);

		// Test directed message to the GPS200 from the WSO100 with PGN 59904 "ISO Request", priority 6, source 81, destination 32.
		id = 0x18EA2051;
		assert(Iso11783Decode(id, &src, &dest, &pri) == 59904);
		assert(src == 81);
		assert(dest = 32);
		assert(pri == 6);
		//printf("x:%d\nsrc:%d\ndest:%d\npri%d\n", x, src, dest, pri);

	}

	/** Test Iso11783Encode **/
	{
		// Test output from Maretron GPS200 with PGN 129025, priority 2, source 32.
		// This is a PDU2/broadcast message
		uint32_t pgn = 129025;
		uint8_t src = 32;
		uint8_t dest = 255;
		uint8_t pri = 2;
		assert(Iso11783Encode(pgn, src, dest, pri) == 0x09F80120);

		// Repeat the above test with a different destination. This value is not checked and should have no bearing on the result
		assert(Iso11783Encode(pgn, src, 120, pri) == 0x09F80120);

		// Test encoding a PDU1 message.
		pgn =  59904;
		src = 81;
		dest = 32;
		pri = 6;
		assert(Iso11783Encode(pgn, src, dest, pri) == 0x18EA2051);
	}

	/** Test parsing of PGN126992 */
	{
		uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

		// Check for null-pointer exceptions when no arguments are passed.
		ParsePgn126992(data, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);

		uint8_t seqId = 5;
		uint8_t source = 5;
		uint16_t year = 5;
		uint8_t month = 5;
		uint8_t day = 5;
		uint8_t hour = 5;
		uint8_t minute = 5;
		uint8_t second = 5;
		uint64_t uSeconds = 5;

		// Check for proper parsing of invalid data. It should return 0 and not modify any passed arguments.
		assert(ParsePgn126992(data, &seqId, &source, &year, &month, &day, &hour, &minute, &second, &uSeconds) == 0);
		assert(seqId == 5);
		assert(source == 5);
		assert(year == 5);
		assert(month == 5);
		assert(day == 5);
		assert(hour == 5);
		assert(minute == 5);
		assert(second == 5);
		assert(uSeconds == 5);
	}

	/** Test parsing of PGN127245 **/
	{
		uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

		/*Check for NULL pointer exception*/

		ParsePgn127245(data, NULL, NULL, NULL, NULL, NULL);

		uint8_t seqId = 5;
		uint8_t instance = 5;
		uint8_t direction = 5;
		float angleOrder = 5.0;
		float position = 5.0;

		// Check for proper parsing of invalid data. It should return 0 and not modify any passed arguments.

		assert(ParsePgn127245(data, &seqId, &instance, &direction, &angleOrder, &position) == 0);
		assert(seqId == 5);
		assert(instance == 5);
		assert(direction == 5);
		assert(angleOrder == 5.0);
		assert(position == 5.0);

		// Check for correct parsing of only seqId
		uint8_t data1[8] = {10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
		assert(ParsePgn127245(data1, &seqId, &instance, &direction, &angleOrder, &position) == 0x01);
		assert(seqId == 10);

		// Check for correct parsing of only instance
		uint8_t data2[8] = {0xFF, 10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
		assert(ParsePgn127245(data2, &seqId, &instance, &direction, &angleOrder, &position) == 0x02);
		assert(instance == 10);

		// Check for correct parsing of only direction
		uint8_t data3[8] = {0xFF, 0xFF, 0xBF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
		assert(ParsePgn127245(data3, &seqId, &instance, &direction, &angleOrder, &position) == 0x04);
		assert(direction == 2);

		// Check for correct parsing of only angleOrder
		uint8_t data4[8] = {0xFF, 0xFF, 0xFF, 10, 10, 0xFF, 0xFF, 0xFF};
		assert(ParsePgn127245(data4, &seqId, &instance, &direction, &angleOrder, &position) == 0x08);
		assert(angleOrder == 0x0A0A);

		// Check for correct parsing of only position
		uint8_t data5[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 10, 10, 0xFF};
		assert(ParsePgn127245(data5, &seqId, &instance, &direction, &angleOrder, &position) == 0x10);
		assert(position == 0x0A0A);

		// Check for correct parsing of seqId and angleOrder
		uint8_t data6[8] = {0xF5, 0xFF, 0xFF, 0x3F, 0xC3, 0xFF, 0xFF, 0xFF};
		assert(ParsePgn127245(data6, &seqId, &instance, &direction, &angleOrder, &position) == 0x09);
		assert(angleOrder == 0xC33F);
		assert(seqId == 0xF5);

		// Check for correct parsing of instance and position
		uint8_t data7[8] = {0xFF, 0xF5, 0xFF, 0xFF, 0xFF, 0x3F, 0xC3, 0xFF};
		assert(ParsePgn127245(data7, &seqId, &instance, &direction, &angleOrder, &position) == 0x12);
		assert(position == 0xC33F);
		assert(instance == 0xF5);

		// Check for correct parsing of all valid fields
		uint8_t data8[8] = {13, 13, 0x7F, 0x13, 0x13, 0x13, 0x13, 13};
		assert(ParsePgn127245(data8, &seqId, &instance, &direction, &angleOrder, &position) == 0x1F);
		assert(seqId == 13);
		assert(instance == 13);
		assert(direction == 1);
		assert(angleOrder == 0x1313);
		assert(position == 0x1313);

	}

	/** Test parsing of PGN127508 **/
	{
		uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

		// Check for null-pointer exceptions when passed no arguments
		ParsePgn127508(data, NULL, NULL, NULL, NULL, NULL);

		uint8_t seqId = 5;
		uint8_t instance = 5;
		float voltage = 5.0;
		float current = 5.0;
		float temperature = 5.0;

		// Check for proper parsing of invalid data. It should return 0 and not modify any
		// passed arguments.
		assert(ParsePgn127508(data, &seqId, &instance, &voltage, &current, &temperature) == 0);
		assert(seqId == 5);
		assert(instance == 5);
		assert(voltage == 5.0);
		assert(current == 5.0);
		assert(temperature == 5.0);
	}

	/** Test parsing of PGN128259 **/
	{
		uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

		// Check for null-pointer exceptions when passed no arguments
		ParsePgn128259(data, NULL, NULL);

		uint8_t seqId = 5;
		float waterSpeed = 5.0;

		// Check for proper parsing of invalid data. It should return 0 and not modify any
		// passed arguments.
		assert(ParsePgn128259(data, &seqId, &waterSpeed) == 0);
		assert(seqId == 5);
		assert(waterSpeed == 5.0);
	}

	/** Test parsing of PGN128267 **/
	{
		uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

		// Check for null-pointer exceptions when passed no arguments
		ParsePgn128267(data, NULL, NULL, NULL);

		uint8_t seqId = 5;
		float waterDepth = 5.0;
		float offset = 5.0;

		// Check for proper parsing of invalid data. It should return 0 and not modify any
		// passed arguments.
		assert(ParsePgn128267(data, &seqId, &waterDepth, &offset) == 0);
		assert(seqId == 5);
		assert(waterDepth == 5.0);
		assert(offset == 5.0);
	}

	/** Test parsing of PGN129025 **/
	{
		uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0x7F};

		// Check for null-pointer exceptions when passed no arguments
		ParsePgn129025(data, NULL, NULL);

		float latitude = 5.0;
		float longitude = 5.0;

		// Check for proper parsing of invalid data. It should return 0 and not modify any
		// passed arguments.
		assert(ParsePgn129025(data, &latitude, &longitude) == 0);
		assert(latitude == 5.0);
		assert(longitude == 5.0);
	}

	/** Test parsing of PGN129026 **/
	{
		uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

		// Check for null-pointer exceptions when passed no arguments
		ParsePgn129026(data, NULL, NULL, NULL, NULL);

		uint8_t seqId = 5;
		uint8_t cogRef = 5;
		float cog = 5.0;
		float sog = 5.0;

		// Check for proper parsing of invalid data. It should return 0 and not modify any
		// passed arguments.
		assert(ParsePgn129026(data, &seqId, &cogRef, &cog, &sog) == 0);
		assert(seqId == 5);
		assert(cogRef == 5);
		assert(cog == 5.0);
		assert(sog == 5.0);
	}

	/** Test parsing of PGN130306 **/
	{
		uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

		// Check for null-pointer exceptions when passed no arguments
		ParsePgn130306(data, NULL, NULL, NULL);

		uint8_t seqId = 5;
		float airSpeed = 5.0;
		float direction = 5.0;

		// Check for proper parsing of invalid data. It should return 0 and not modify any
		// passed arguments.
		assert(ParsePgn130306(data, &seqId, &airSpeed, &direction) == 0);
		assert(seqId == 5);
		assert(airSpeed == 5.0);
		assert(direction == 5.0);
	}

	/** Test parsing of PGN130310 **/
	{
		uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

		// Check for null-pointer exceptions when passed no arguments
		ParsePgn130310(data, NULL, NULL, NULL, NULL);

		uint8_t seqId = 5;
		float waterTemp = 5.0;
		float airTemp = 5.0;
		float airPressure = 5.0;

		// Check for proper parsing of invalid data. It should return 0 and not modify any
		// passed arguments.
		assert(ParsePgn130310(data, &seqId, &waterTemp, &airTemp, &airPressure) == 0);
		assert(seqId == 5);
		assert(waterTemp == 5.0);
		assert(airTemp == 5.0);
		assert(airPressure == 5.0);
	}

	/** Test parsing of PGN130311 **/
	{
		uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

		// Check for null-pointer exceptions when passed no arguments
		ParsePgn130311(data, NULL, NULL, NULL, NULL, NULL, NULL);

		uint8_t seqId = 5;
		uint8_t tempInstance = 5;
		uint8_t humidityInstance = 5;
		float temp = 5.0;
		float humidity = 5.0;
		float pressure = 5.0;

		// Check for proper parsing of invalid data. It should return 0 and not modify any
		// passed arguments.
		assert(ParsePgn130311(data, &seqId, &tempInstance, &humidityInstance, &temp, &humidity, &pressure) == 0);
		assert(seqId == 5);
		assert(tempInstance == 5);
		assert(humidityInstance == 5);
		assert(temp == 5.0);
		assert(humidity == 5.0);
		assert(pressure == 5.0);

		// Check for correct parsing of only seqId
		uint8_t data1[8] = {10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
		assert(ParsePgn130311(data1, &seqId, &tempInstance, &humidityInstance, &temp, &humidity, &pressure) == 0x01);
		assert(seqId == 10);

		// Check for correct parsing of only tempInstance
		uint8_t data2[8] = {0xFF, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
		assert(ParsePgn130311(data2, &seqId, &tempInstance, &humidityInstance, &temp, &humidity, &pressure) == 0x02);
		assert(tempInstance == 3);

		// Check for correct parsing of only humidityInstance
		uint8_t data3[8] = {0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
		assert(ParsePgn130311(data3, &seqId, &tempInstance, &humidityInstance, &temp, &humidity, &pressure) == 0x04);
		assert(humidityInstance == 2);

		// Check for correct parsing of only temp.
		// Testing the value of 255.4K, which should result in -17.75C.
		tUnsignedShortToChar unionedTemp = {25540};
		uint8_t data4[8] = {0xFF, 0xFF, unionedTemp.chData[0], unionedTemp.chData[1], 0xFF, 0xFF, 0xFF, 0xFF};
		assert(ParsePgn130311(data4, &seqId, &tempInstance, &humidityInstance, &temp, &humidity, &pressure) == 0x08);
		assert(fabs(temp - (-17.75)) <= EPSILON);

		// Check for correct parsing of only humidity
		tUnsignedShortToChar unionedHumidity = {17000};
		uint8_t data5[8] = {0xFF, 0xFF, 0xFF, 0xFF, unionedHumidity.chData[0], unionedHumidity.chData[1], 0xFF, 0xFF};
		assert(ParsePgn130311(data5, &seqId, &tempInstance, &humidityInstance, &temp, &humidity, &pressure) == 0x10);
		assert(fabs(humidity - 68.0) <= EPSILON);

		// Check for correct parsing of only pressure
		tUnsignedShortToChar unionedPressure = {1014};
		uint8_t data6[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, unionedPressure.chData[0], unionedPressure.chData[1]};
		assert(ParsePgn130311(data6, &seqId, &tempInstance, &humidityInstance, &temp, &humidity, &pressure) == 0x20);
		assert(fabs(pressure - 101.4) <= EPSILON);

		// Check for correct parsing of seqId and temp
		uint8_t data7[8] = {0xF5, 0xFF, unionedTemp.chData[0], unionedTemp.chData[1], 0xFF, 0xFF, 0xFF, 0xFF};
		assert(ParsePgn130311(data7, &seqId, &tempInstance, &humidityInstance, &temp, &humidity, &pressure) == 0x09);
		assert(fabs(temp - (-17.75)) <= EPSILON);
		assert(seqId == 0xF5);

		// Check for correct parsing of tempInstance and pressure
		uint8_t data8[8] = {0xFF, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, unionedPressure.chData[0], unionedPressure.chData[1]};
		assert(ParsePgn130311(data8, &seqId, &tempInstance, &humidityInstance, &temp, &humidity, &pressure) == 0x22);
		assert(fabs(pressure - 101.4) <= EPSILON);
		assert(tempInstance == 0x03);

		// Check for correct parsing of all valid fields
		uint8_t data9[8] = {
			13,
			(33 << 2) | 2,
			unionedTemp.chData[0], unionedTemp.chData[1],
			unionedHumidity.chData[0], unionedHumidity.chData[1],
			unionedPressure.chData[0], unionedPressure.chData[1]
		};
		assert(ParsePgn130311(data9, &seqId, &tempInstance, &humidityInstance, &temp, &humidity, &pressure) == 0x3F);
		assert(seqId == 13);
		assert(tempInstance == 33);
		assert(humidityInstance == 2);
		assert(fabs(temp - (-17.75)) <= EPSILON);
		assert(fabs(humidity - 68.0) <= EPSILON);
		assert(fabs(pressure - 101.4) <= EPSILON);
	}
	printf("All tests passed!\n");
	return 0;
}

#endif // UNIT_TEST_NMEA2000
