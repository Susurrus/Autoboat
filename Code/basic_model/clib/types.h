#ifndef __TYPES_H__
#define __TYPES_H__

typedef union {
	unsigned char    chData[2];
	unsigned short   usData;
} tUnsignedShortToChar; 

typedef union {
	unsigned char    chData[2];
	short   		 shData;
} tShortToChar; 

typedef union {
	unsigned char   chData[4];
	unsigned int   	uiData;
} tUnsignedIntToChar; 

typedef union {
	unsigned char   chData[4];
	int   			inData;
} tIntToChar; 

typedef union {
	unsigned char   chData[4];
	float   		flData;
	unsigned short	shData[2];
} tFloatToChar;

#ifndef NULL
	#define NULL  0
#endif

#define TRUE  1
#define FALSE 0

#endif // __TYPES_H__
