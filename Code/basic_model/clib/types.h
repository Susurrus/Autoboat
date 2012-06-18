#ifndef _TYPES_H_
#define _TYPES_H_

#include <stdint.h>

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

#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327
#endif

#ifndef NAN
#define NAN __builtin_nan("")
#endif

#endif // _TYPES_H_
