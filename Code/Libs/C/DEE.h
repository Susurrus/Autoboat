/************************************************************************
*
*   Emulating Data EEPROM for PIC24 Microcontrollers and
*           dsPIC Digital Signal Controllers
*
*************************************************************************
* FileName:     DEE Emulation 16-bit.h
* Compiler:     MPLAB C30, v3.30 or higher
* Company:      Microchip Technology, Inc.
*
* Software License Agreement
*
* Copyright © 2007 Microchip Technology Inc. All rights reserved.
*
* Microchip licenses to you the right to use, modify, copy and distribute
* Software only when embedded on a Microchip microcontroller or digital
* signal controller, which is integrated into your product or third party
* product (pursuant to the sublicense terms in the accompanying license
* agreement).
*
* You should refer to the license agreement accompanying this Software for
* additional information regarding your rights and obligations.
*
* SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
* WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A
* PARTICULAR PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE
* LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY,
* CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY
* DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
* INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST
* PROFITS OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY,
* SERVICES, OR ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO
* ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*
* Author            Date        Comment
*************************************************************************
* D. Otten          2007/05/01  Version 1.0.0 - Initial Release
* D. Otten          2007/05/15  Version 1.0.1 - First publication release
* Pradeep Budagutta 2008/04/02  Version 1.1.0 - Multi EEPROM banks included
* Priyabrata Sinha  2011/01/20  Version 2.0.0 - Added dsPIC33E/PIC24E support
* Priyabrata Sinha  2012/04/19  Version 2.2.0 - Removed absolute path
************************************************************************/
// User defined constants
#define DATA_EE_BANKS       2
#define DATA_EE_SIZE        255
#define DATA_EE_TOTAL_SIZE  (DATA_EE_BANKS * DATA_EE_SIZE)
#define NUM_DATA_EE_PAGES   3
#if defined(__dsPIC33E__) || defined(__PIC24E__)
#define ERASE               0x4003
#define PROGRAM_ROW         0x4002
#define PROGRAM_WORD        0x4001
// Uncomment the following line if using Auxiliary Flash for EEPROM Emulation
//#define __AUXFLASH          1
#else
#define ERASE               0x4042
#define PROGRAM_ROW         0x4001
#define PROGRAM_WORD        0x4003
#endif

// Internal constants
#if defined(__dsPIC33E__) || defined(__PIC24E__)
#define	NUMBER_OF_INSTRUCTIONS_IN_PAGE  1024
#define	NUMBER_OF_INSTRUCTIONS_IN_ROW   128
#else
#define	NUMBER_OF_INSTRUCTIONS_IN_PAGE  512
#define	NUMBER_OF_INSTRUCTIONS_IN_ROW   64
#endif
#define ERASE_WRITE_CYCLE_MAX           5
#define NUMBER_OF_ROWS_IN_PAGE          (NUMBER_OF_INSTRUCTIONS_IN_PAGE / NUMBER_OF_INSTRUCTIONS_IN_ROW)
#define PAGE_AVAILABLE                  1
#define PAGE_CURRENT                    0
#define PAGE_EXPIRED                    0
#define PAGE_NOT_AVAILABLE              0
#define PAGE_NOT_CURRENT                1
#define PAGE_NOT_EXPIRED                1
#define STATUS_AVAILABLE                18
#define STATUS_CURRENT                  19
#define STATUS_EXPIRED                  20

#define GetaddrNotFound() dataEEFlags.addrNotFound
#define SetaddrNotFound(x) dataEEFlags.addrNotFound = x

#define GetPageCorruptStatus() dataEEFlags.pageCorrupt
#define SetPageCorruptStatus(x) dataEEFlags.pageCorrupt = x

#define GetPageExpiredPage() dataEEFlags.expiredPage
#define SetPageExpiredPage(x) dataEEFlags.expiredPage = x

#define GetPageIllegalAddress() dataEEFlags.IllegalAddress
#define SetPageIllegalAddress(x) dataEEFlags.IllegalAddress = x

#define GetPagePackBeforeInit() dataEEFlags.packBeforeInit
#define SetPagePackBeforeInit(x) dataEEFlags.packBeforeInit = x

#define GetPagePackBeforePageFull() dataEEFlags.packBeforePageFull
#define SetPagePackBeforePageFull(x) dataEEFlags.packBeforePageFull = x

#define GetPagePackSkipped() dataEEFlags.packSkipped
#define SetPagePackSkipped(x) dataEEFlags.packSkipped = x

#define GetPageWriteError() dataEEFlags.writeError
#define SetPageWriteError(x) dataEEFlags.writeError = x

typedef union
{
    unsigned char val;
    struct
    {
        unsigned addrNotFound:1;	    // Return 0xFFFF
        unsigned expiredPage:1;	   	    // Return 0x1
        unsigned packBeforePageFull:1;	// Not a return condition
        unsigned packBeforeInit:1;		// Return 0x3
        unsigned packSkipped:1;		    // Return 0x4
        unsigned IllegalAddress:1;	    // Return 0x5
        unsigned pageCorrupt:1;		    // Return 0x6
        unsigned writeError:1;		    // Return 0x7
    };
} DATA_EE_FLAGS;

extern DATA_EE_FLAGS dataEEFlags;

extern int  ReadPMHigh(int);
extern int  ReadPMLow(int);
extern void UnlockPM(void);
extern int  WritePMHigh(int, int);
extern int  WritePMHighB(int, int);
extern int  WritePMLow(int, int);
extern int  WritePMLowB(int, int);

void            UnlockWrite         (void);
int             GetPageStatus       (unsigned char bank, unsigned volatile char page, unsigned volatile char field);
void            ErasePage           (unsigned char bank, unsigned char page);
char            IncEWCount          (unsigned char *index);
unsigned int    GetNextAvailCount   (unsigned char bank);
int             PackEE              (unsigned char bank);
unsigned char   DataEEInit          (void);
unsigned int    DataEERead          (unsigned int addr);
unsigned char   DataEEWrite         (unsigned int data, unsigned int addr);

