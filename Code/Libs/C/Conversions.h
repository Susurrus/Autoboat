#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <stdint.h>

/**
 * Converts a hexadecimal ASCII character ('0' through 'f'/'F') to it's numeric representation.
 * -1 is returns if an error occurs.
 */
uint8_t hexchar2int(char halfhex);

/**
 * Convert an integer to its ASCII representation as a hexadecimal value.
 * @param value The value to convert.
 * @return An ASCII character or NUL if the argument was invalid.
 */
char int2hexchar(uint8_t value);

/**
 * Converts a representation of degrees from degrees and minutes to raw degrees in floating-point.
 * Degrees can be any positive value between 0 and 180. 
 * Minutes can be represented as any positive decimal. Checking for negative values isn't done 
 * and so providing negative numbers will result in incorrect calculations.
 */
float degMinToDeg(unsigned char degrees, float minutes);

#endif // CONVERSIONS_H
