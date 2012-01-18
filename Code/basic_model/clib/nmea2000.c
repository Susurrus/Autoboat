#include "nmea2000.h"
#include "types.h"

#define M_PI 3.1415926535

unsigned long ISO11783Decode(unsigned long id, unsigned char *src, unsigned char *dest, unsigned char *pri)
{

	unsigned long pgn;

	// The source address is the lowest 8 bits
	if (src) {
	    	*src = (unsigned char)id;
	}
    
	// The priority are the highest 3 bits
	if (pri) {
		*pri = (unsigned char)((id >> 26) & 7);
	}
	
	// PDU Format byte
	unsigned char PF = (unsigned char)(id >> 8);
	
	// PDU Specific byte
	unsigned long PS = (id >> 16) & 0xFF;
	
	// Most Significant byte
	unsigned long MS = (id >> 24) & 3;

	if (PS > 239) {
		// PDU2 format, the destination is implied global and the PGN is extended.
		if (dest) {
			*dest = 0xFF;
		}
		pgn = (MS << 16) | (PS << 8) | ((unsigned long)PF);
	} else {
		// PDU1 format, the PF contains the destination address.
		if (dest) {
			*dest = PF;
		}
		pgn = (MS << 16) | (PS << 8);
	}

	return pgn;
}

unsigned char ParsePgn128259(unsigned char data[8], unsigned char *seqId, float *waterSpeed)
{

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each field.
	unsigned char fieldStatus = 0;
	
	// Field 0: Sequence ID. Links data together across PGNs that occured at the same timestep. If the sequence ID is 255, it's invalid.
	if (seqId && data[0] != 0xFF) {
		*seqId = data[0];
		fieldStatus |= 0x01;
	}
	
	// Field 1: Water speed. Raw units are centimeters/second. Converted to meters/second for output.
	if (waterSpeed) {
		unsigned int unpacked = data[1];
		unpacked |= ((unsigned int)data[2]) << 8;
		*waterSpeed = ((float)unpacked) / 100.0;
		fieldStatus |= 0x02;
	}
	
	return fieldStatus;
}

unsigned char ParsePgn128267(unsigned char data[8], unsigned char *seqId, float *waterDepth, float *offset)
{

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each field.
	unsigned char fieldStatus = 0;
	
	// Field 0: Sequence ID. Links data together across PGNs that occured at the same timestep. If the sequence ID is 255, it's invalid.
	if (seqId && data[0] != 0xFF) {
		*seqId = data[0];
		fieldStatus |= 0x01;
	}
	
	// Field 1: Water depth. Raw units are centimeters. Converted to meters for output.
	if (waterDepth) {
		unsigned int unpacked = data[1];
		unpacked |= ((unsigned int)data[2]) << 8;
		if (unpacked != 0xFFFF) {
			*waterDepth = ((float)unpacked) / 100.0;
			fieldStatus |= 0x02;
		} else {
			*waterDepth = 0.0;
		}
	}
	
	// Field 2: Water depth offset. Raw units are centimeters. Converted to meters for output.
	if (offset) {
		unsigned int unpacked = data[5];
		unpacked |= ((unsigned int)data[6]) << 8;
		*offset = ((float)unpacked) * .01;
		fieldStatus |= 0x04;
	}
	
	return fieldStatus;
}

unsigned char ParsePgn129025(unsigned char data[8], float *latitude, float *longitude)
{

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each field.
	unsigned char fieldStatus = 0;

	// Make conversions between long and chars easy
	tLongToChar unpacked;
		
	// Field 0: Latitude. Raw units are 1e-7 degrees, but they're converted to raw radians on output.
	if (latitude) {
		unpacked.chData[0] = data[0];
		unpacked.chData[1] = data[1];
		unpacked.chData[2] = data[2];
		unpacked.chData[3] = data[3];
		*latitude = ((float)unpacked.lData) / 1e7 * M_PI / 180;
		fieldStatus |= 0x01;
	}
	
	// Field 1: Longitude. Raw units are 1e-7 degrees, but they're converted to raw radians on output.
	if (longitude) {
		unpacked.chData[0] = data[4];
		unpacked.chData[1] = data[5];
		unpacked.chData[2] = data[6];
		unpacked.chData[3] = data[7];
		*longitude = ((float)unpacked.lData) / 1e7 * M_PI / 180;
		fieldStatus |= 0x02;
	}
	
	return fieldStatus;
}

unsigned char ParsePgn129026(unsigned char data[8], unsigned char *seqId, unsigned char *cogRef, float *cog, float *sog)
{

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each field.
	unsigned char fieldStatus = 0;
	
	// Field 0: Sequence ID. Links data together across PGNs that occured at the same timestep. If the sequence ID is 255, it's invalid.
	if (seqId && data[0] != 0xFF) {
		*seqId = data[0];
		fieldStatus |= 0x01;
	}
	
	// Field 1: Course over ground reference. A 0 for True reference and a 1 for a magnetic field reference.
	if (cogRef) {
		*cogRef = data[1] & 0x03;
		fieldStatus |= 0x02;
	}
	
	// Field 2: Course over ground. Raw units are .0001 degrees eastward from north but are converted to plain radians for output.
	if (cog) {
		tUnsignedShortToChar unpacked;
		unpacked.chData[0] = data[2];
		unpacked.chData[1] = data[3];
		*cog = ((float)unpacked.usData) / 1e4;
		fieldStatus |= 0x04;
	}
	
	// Field 3: Speed over ground. Raw units are .01 m/s but are converted to straight m/s on output.
	if (sog) {
		tUnsignedShortToChar unpacked;
		unpacked.chData[0] = data[4];
		unpacked.chData[1] = data[5];
		*sog = ((float)unpacked.usData) / 1e2;
		fieldStatus |= 0x08;
	}
	
	return fieldStatus;
}

unsigned char ParsePgn129033(unsigned char data[8], unsigned char *day, unsigned char *month, unsigned char *year, unsigned char *hour, unsigned char *minute, unsigned char *second)
{

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each field.
	unsigned char fieldStatus = 0;
	
	// Field 2: Local offset. Units in minutes. We fetch this field first as it affects the other two fields
	// Figure out proper algorithm and fill in the code below.
//	{
//		tShortToChar unpacked;
//		unpacked.chData[0] = data[6];
//		unpacked.chData[1] = data[7];
//		fieldStatus != 0x04;
//	}
	
	// Field 0: Date in days since Jan 1 1970.
	// This field can be invalid if all 1's.
	if (data[0] != 0xFF && data[1] != 0xFF) {
		tUnsignedShortToChar unpacked;
		unpacked.chData[0] = data[0];
		unpacked.chData[1] = data[1];
		fieldStatus |= 0x01;
		
		if (day) {
			*day = 0;
		}
		
		if (month) {
			*month = 0;
		}
		
		if (year) {
			*year = 0;
		}
	}
	
	// Field 1: Seconds since midnight in units of 1e-4 second.
	{
		tUnsignedLongToChar unpacked;
		unpacked.chData[0] = data[2];
		unpacked.chData[1] = data[3];
		unpacked.chData[2] = data[4];
		unpacked.chData[3] = data[5];
		unpacked.ulData /= 1e4;
		fieldStatus |= 0x02;

		unsigned long seconds = unpacked.ulData;
		if (hour) {
			*hour = seconds / 3600;
		}
		seconds %= 3600;
		
		if (minute) {
			*minute = seconds / 60;
		}
		seconds %= 60;
		
		if (second) {
			*second = seconds;
		}
	}
	
	return fieldStatus;
}

unsigned char ParsePgn130306(unsigned char data[8], unsigned char *seqId, float *airSpeed, float *direction)
{

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each field.
	unsigned char fieldStatus = 0;
	
	// Field 0: Sequence ID. Links data together across PGNs that occured at the same timestep. If the sequence ID is 255, it's invalid.
	if (seqId && data[0] != 0xFF) {
		*seqId = data[0];
		fieldStatus |= 0x01;
	}
	
	// Field 1: Wind speed. Message units are cm/s but are converted to m/s on output.
	if (airSpeed) {
		unsigned int unpacked = data[1];
		unpacked |= ((unsigned int)data[2]) << 8;
		*airSpeed = ((float)unpacked) / 100.0;
		fieldStatus |= 0x02;
	}
	
	// Field 2: Wind direction. Message units are e-4 rads but are converted to raw radians on output.
	if (direction) {
		unsigned int unpacked = data[3];
		unpacked |= ((unsigned int)data[4]) << 8;
		*direction = 0;
		if (unpacked != 0xFFFF) {
			*direction = ((float)unpacked) * .0001;
			fieldStatus |= 0x04;
		} else {
			*direction = 0;
		}
	}
	
	return fieldStatus;
}

unsigned char ParsePgn130310(unsigned char data[8], unsigned char *seqId, float *waterTemp, float *airTemp, float *airPressure)
{

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each field.
	unsigned char fieldStatus = 0;
	
	// Field 0: Sequence ID. Links data together across PGNs that occured at the same timestep. If the sequence ID is 255, it's invalid.
	if (seqId && data[0] != 0xFF) {
		*seqId = data[0];
		fieldStatus |= 0x01;
	}
	
	// Water temperature data. Read in as centiKelvin and converted to Celsius.
	// A value of 0xFFFF implies invalid data.
	if (waterTemp) {
		unsigned int unpacked = data[1];
		unpacked |= ((unsigned int)data[2]) << 8;
		if (unpacked != 0xFFFF) {
			*waterTemp = ((float)unpacked) / 100.0 - 273.15;
			fieldStatus |= 0x02;
		} else {
			*waterTemp = 0;
		}
	}
	
	// Air temperature data. Read in as centiKelvin and converted to Celsius.
	// A value of 0xFFFF implies invalid data.
	if (airTemp) {
		unsigned int unpacked = data[3];
		unpacked |= ((unsigned int)data[4]) << 8;
		if (unpacked != 0xFFFF) {
			*airTemp = ((float)unpacked) / 100.0 - 273.15;
			fieldStatus |= 0x04;
		} else {
			*airTemp = 0;
		}
	}

	// Air pressure data. Read in as hectoPascals and converted to kiloPascals.
	// A value of 0xFFFF implies invalid data.
	if (airPressure) {
		unsigned int unpacked = data[5];
		unpacked |= ((unsigned int)data[6]) << 8;
		if (unpacked != 0xFFFF) {
			*airPressure = ((float)unpacked) * 0.1;
			fieldStatus |= 0x08;
		} else {
			*airPressure = 0;
		}
	}
	
	return fieldStatus;
}

// TODO: Add code for processing the instance ID and instance fields (2nd byte)
unsigned char ParsePgn130311(unsigned char data[8], unsigned char *seqId, float *temp, float *humidity, float *pressure)
{

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each field.
	unsigned char fieldStatus = 0;
	
	// Field 0: Sequence ID. Links data together across PGNs that occured at the same timestep. If the sequence ID is 255, it's invalid.
	if (seqId && data[0] != 0xFF) {
		*seqId = data[0];
		fieldStatus |= 0x01;
	}
	
	// Add air temperature data. Read in as centiKelvin and converted to Celsius
	if (temp) {
		unsigned int unpacked = data[2];
		unpacked |= ((unsigned int)data[3]) << 8;
		*temp = ((float)unpacked) / 100.0 - 273.15;
		fieldStatus |= 0x02;
	}
	
	// Humidity data. Read in in units of .0004%, output in percent. 
	if (humidity) {
		unsigned int unpacked = data[4];
		unpacked |= ((unsigned int)data[5]) << 8;
		*humidity = ((float)unpacked) * 0.004;
		fieldStatus |= 0x04;
	}

	// Pressure data. Read in as hectoPascals and converted to kiloPascals.
	if (pressure) {
		unsigned int unpacked = data[6];
		unpacked |= ((unsigned int)data[7]) << 8;
		*pressure = ((float)unpacked) * 0.1;
		fieldStatus |= 0x08;
	}
	
	return fieldStatus;
}
