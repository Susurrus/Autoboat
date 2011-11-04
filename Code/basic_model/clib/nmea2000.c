#include "nmea2000.h"

unsigned long ISO11783Decode(unsigned long id, unsigned char *src, unsigned char *dest, unsigned char *pri) {

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

unsigned char ParsePgn128259(unsigned char data[8], unsigned char *seqId, float *waterSpeed) {

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each field.
	unsigned char fieldStatus = 0;
	
	// Field 0: Sequence ID. Links data together across PGNs that occured at the same timestep.
	if (seqId) {
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

unsigned char ParsePgn128267(unsigned char data[8], unsigned char *seqId, float *waterDepth, float *offset) {

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each field.
	unsigned char fieldStatus = 0;
	
	// Field 0: Sequence ID. Links data together across PGNs that occured at the same timestep.
	if (seqId) {
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

unsigned char ParsePgn130306(unsigned char data[8], unsigned char *seqId, float *airSpeed, float *direction) {

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each field.
	unsigned char fieldStatus = 0;
	
	// Field 0: Sequence ID. Links data together across PGNs that occured at the same timestep.
	if (seqId) {
		*seqId = data[0];
		fieldStatus |= 0x01;
	}	
	
	// Field 1: Wind speed. Message units are cm/s. Here we convert to m/s.
	if (airSpeed) {
		unsigned int unpacked = data[1];
		unpacked |= ((unsigned int)data[2]) << 8;
		*airSpeed = ((float)unpacked) / 100.0;
		fieldStatus |= 0x02;
	}
	
	// Field 2: Wind direction. Message units at e-4 rads
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

unsigned char ParsePgn130310(unsigned char data[8], unsigned char *seqId, float *waterTemp, float *airTemp, float *airPressure) {

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each field.
	unsigned char fieldStatus = 0;
	
	// Field 0: Sequence ID. Links data together across PGNs that occured at the same timestep.
	if (seqId) {
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
unsigned char ParsePgn130311(unsigned char data[8], unsigned char *seqId, float *temp, float *humidity, float *pressure) {

	// fieldStatus is a bitfield containing success (1) or failure (0) bits in increasing order for each field.
	unsigned char fieldStatus = 0;
	
	// Field 0: Sequence ID. Links data together across PGNs that occured at the same timestep.
	if (seqId) {
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
