#include "conversions.h"

// Include stdio.h for the unit test mode.
#ifdef UNIT_TEST
#include <stdio.h>
#include <assert.h>
#endif // UNIT_TEST

char hex2char(char halfhex) {
	char rv;
	// Test for numeric characters
	if ((rv = halfhex - '0') <= 9 && rv >= 0) {
		return rv;
	}
	// Otherwise check for upper-case A-F
	else if ((rv = halfhex - 'A') <= 5 && rv >= 0) {
		return rv + 10;
	}
	// Finally check for lower-case a-f
	else if ((rv = halfhex - 'a') <= 5 && rv >= 0) {
		return rv + 10;
	}
	// Otherwise return -1 as an error
	return -1;
}

float degMinToDeg(unsigned char degrees, float minutes) {
	return ((float)degrees + minutes/60.0);
}

#ifdef UNIT_TEST_CONVERSIONS
int main() {
	
	printf("Testing conversions.c. All errors will be reported as failed assertions.\n");
	
	printf("Testing hex2char()\n");
	
	/** Testing hex2char() **/
	assert(hex2char('!') == -1);
	assert(hex2char('0') == 0);
	assert(hex2char('9') == 9);
	assert(hex2char(':') == -1);
	
	assert(hex2char('@') == -1);
	assert(hex2char('A') == 10);
	assert(hex2char('F') == 15);
	assert(hex2char('G') == -1);
	
	assert(hex2char('`') == -1);
	assert(hex2char('a') == 10);
	assert(hex2char('f') == 15);
	assert(hex2char('g') == -1);
	
	/** Testing degMinToDeg() **/
	printf("Testing degMinToDeg()\n");
	
	// 40 degrees, 41 arc minutes = 40.68
	assert(degMinToDeg(40, 41) - 40.68333 < .001);
	
	// 0 degrees, 0 arc minutes = 0
	assert(degMinToDeg(0, 0) - 0 < .001);
	
	// 122 degrees, 24 arc minutes = 122.4
	assert(degMinToDeg(122, 24) - (122.4) < .001);
	
	return 0;
}
#endif // UNIT_TEST
