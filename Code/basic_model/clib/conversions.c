#include "conversions.h"

/**
 * This function converts one hex ASCII character to numeric
 * hex. It's used for the checksum comparison.
 */
char hex2char(char halfhex) {
	if ((halfhex - 48) < 9) {
		return (halfhex - 48);
	}
	return (halfhex - 55);
}

float degMinToDeg(unsigned char degrees, float minutes) {
	return ((float)degrees + minutes/60.0);
}
