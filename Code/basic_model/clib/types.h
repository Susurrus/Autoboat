#ifndef __TYPES_H__
#define __TYPES_H__

#include <inttypes.h>

typedef union {
	uint8_t  chData[2];
	uint16_t usData;
} tUnsignedShortToChar; 

typedef union {
	uint8_t	chData[2];
	int16_t	shData;
} tShortToChar;

typedef union {
	uint8_t  chData[4];
	uint32_t ulData;
} tUnsignedLongToChar;

typedef union {
	uint8_t chData[4];
	int32_t lData;
} tLongToChar;

typedef union {
	uint8_t  chData[4];
	float    flData;
	uint16_t usData[2];
} tFloatToChar;

#ifndef NULL
	#define NULL  0
#endif

#define TRUE  1
#define FALSE 0

#endif // __TYPES_H__
