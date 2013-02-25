/************************************************************************
*
*   Emulating Data EEPROM for PIC24 Microcontrollers and
*           dsPIC Digital Signal Controllers
*
* This application note provides a standard interface to an efficient
* Data EEPROM emulation algorithm and uses available program memory. 
* It is designed for Microchip Technology 16-bit PIC and dsPIC J devices 
* which currently include PIC24F, PIC24H and dsPIC33 products. The
* project is initially configured to use PIC24FJ128GA010 on the Explorer
* 16 Development Board. To use different device, simply select new device
* in MPLAB, replace C30 linker script and rebuild.
* User must select number pages of program memory, erase/write limit and 
* emulated DEE size. These are defined in "DEE Emulation 16-bit.h".
* At build-time, the linker reserves pages in the next available 
* locations in program memory. Compiler error occurs if more than 255 
* DEE locations are declared, less than 2 pages of program memory is 
* reserved, greater than 65,535 erase/write cycles specified or if 
* insufficient program memory is available. 
* Call initialization routine and clear status flags before attempting 
* any other DEE operation.
*
*************************************************************************
* FileName:     DEE.c
* Dependencies: DEES.s
*               DEE.h
* Compiler:     MPLAB C30, v2.01 or higher
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
* Author        Date        Comment
*************************************************************************
* D. Otten      2007/05/01  Version 1.0.0 - Initial Release
* D. Otten      2007/05/15  Version 1.0.1 - First publication release
************************************************************************/

/***********************************************************************
    Code Slightly modified by Mariano I. Lizararga to make it compatible
    with Lubin's dsPIC Embedded Target
************************************************************************/

#include "DEE.h"


// User constant validation
#if DATA_EE_SIZE > 255
    #error Maximum data EE size is 255
#endif

#if NUM_DATA_EE_PAGES < 2
    #error Minimum number of program memory pages is 2
#endif

#if ERASE_WRITE_CYCLE_MAX > 65535
    #error Maximum number of erase/write cycles is 65,535
#endif


DATA_EE_FLAGS dataEEFlags;

//Data EE info stored in PM in following format
//  Status in first two locations of PM page,
//  8-bit DEE Address (odd address, low byte) 16-bit DEE data (even address)
unsigned char emulationPages[NUM_DATA_EE_PAGES][NUMBER_OF_INSTRUCTIONS_IN_PAGE * 2]
    __attribute__ ((space(psv), aligned(NUMBER_OF_INSTRUCTIONS_IN_PAGE * 2), noload));

/************************************************************************
UnlockWrite

This routine saves the current CPU priority and sets it the highest
user level of 7. It calls an assembly routine to perform an unlock
sequence and sets the WR bit in NVMCON. The WR bit is polled until it
clears indicating the flash operation is complete. The previous CPU
priority is restored.

Parameters:		None
Return:			None
Side Effects:	None
************************************************************************/
void UnlockWrite(void)
{
    unsigned int sh_SR;

    SET_AND_SAVE_CPU_IPL(sh_SR, 7);

    UnlockPM();

    RESTORE_CPU_IPL(sh_SR);

    return;
}

/************************************************************************
GetPageStatus

This routine returns the page status for the selected page for the
selected field. The return value is right shifted into LSb position.

Parameters:		Page number, Status Field
Return:			Right justified bit value representing selected Status
                Field value
Side Effects:	None
************************************************************************/
int GetPageStatus(unsigned char page, unsigned volatile char field)
{
    unsigned int statusOffset;
    unsigned char statusByte;
    unsigned char status;

    statusOffset = __builtin_tbloffset( &emulationPages) + sizeof(emulationPages[0])*page;

    statusByte = (ReadPMHigh(statusOffset) & 0xFF);

    switch(field)
    {
        case STATUS_AVAILABLE:
            status = ((statusByte & 4) >> 2);
            break;
        case STATUS_CURRENT:
            status = ((statusByte & 8) >> 3);
            break;
        case STATUS_EXPIRED:
            status = ((statusByte & 16) >> 4);
            break;
        default:
            status = 0;
            break;
    }

    return(status);

}

/************************************************************************
ErasePage

This routine erases the selected page.

Parameters:		Page number
Return:			None
Side Effects:	Loads NVCOM with erase opcode
************************************************************************/
void ErasePage(unsigned char page)
{
    unsigned int pmOffset;           //Current array (page) offset of selected element (PM 16-bit word)

    NVMCON = ERASE;

    pmOffset = __builtin_tbloffset( &emulationPages) + sizeof(emulationPages[0])*page;

    WritePMLow(pmOffset, pmOffset);

    UnlockWrite();

    return;
}

/************************************************************************
GetNextAvailCount

This routine finds the active page and performs a backward search to find
the first available location. The available location is determined by
reading an LSB (odd address) with 0xFF. The returned value can be added
to the first address in page to compute the available address. A return
value of 0 means the entire page was filled which is an error condition.
This routine can be called by the user to determine how full the current
page is prior to a pack.

Parameters:		None
Return:			Page offset to next available location
Side Effects:	None
************************************************************************/
unsigned int GetNextAvailCount(void)
{
    unsigned int i = 0;
    int currentPage;        //Array row (PM page) of active DEE page
    unsigned char dataEEval;
    unsigned int pmOffset;           //Current array (page) offset of selected element (PM 16-bit word)
    int savedTBLPAG;        //Context save of TBLPAG value. Current and packed page are on same page.

    savedTBLPAG = TBLPAG;

    // Find the active page.
    for (currentPage = 0;
         (currentPage < NUM_DATA_EE_PAGES) &&
         (GetPageStatus(currentPage, STATUS_CURRENT) == PAGE_NOT_CURRENT);
         currentPage++) {}

    pmOffset = __builtin_tbloffset( &emulationPages) + sizeof(emulationPages[0])*currentPage;

    do
    {
        i+=2;
        pmOffset += 2;

        dataEEval = (ReadPMHigh(pmOffset) & 0xFF);
    }
    while ((i<NUMBER_OF_INSTRUCTIONS_IN_PAGE * 2) && (dataEEval != 0xFF));

    if(i == NUMBER_OF_INSTRUCTIONS_IN_PAGE * 2)
    {
        i = 0;  //Error - No available locations
    }

    TBLPAG = savedTBLPAG;

    return(i);
}
/************************************************************************
DataEERead

This routine verifies the address is valid. If not, the Illegal Address
flag is set and 0xFFFF is returned. It then finds the active page. If an
active page can not be found, the Page Corrupt status bit is set and
0xFFFF is returned. A reverse search of the active page attempts to find
the matching address in the program memory MSB (odd address). If a match
is found, the corresponding data EEPROM data (even address) is returned,
otherwise 0xFFFF is returned. This function can be called by the user.

Parameters:		Data EE address
Return:			Data EE data or 0xFFFF if address not found
Side Effects:	Data EE flags may be updated.
************************************************************************/
//unsigned int DataEERead(unsigned int addr)
unsigned int DataEERead(unsigned char addr)
{
    unsigned int savedTBLPAG;        //Context save of TBLPAG value. Current and packed page are on same page.
    unsigned int currentPage;
    unsigned int pmOffset;           //Current array (page) offset of selected element (PM 16-bit word)
    unsigned int latch;
    unsigned int i;

    if(addr >= DATA_EE_SIZE)
    {
        SetPageIllegalAddress(1);
        return(0xFFFF);
    }

    savedTBLPAG = TBLPAG;

    // Point the table page pointer to the emulation pages
    TBLPAG = __builtin_tblpage(&(emulationPages[0]));

    // Find the active page.
    for (currentPage = 0;
         (currentPage < NUM_DATA_EE_PAGES) &&
         (GetPageStatus(currentPage, STATUS_CURRENT) == PAGE_NOT_CURRENT);
         currentPage++) {}

    if (currentPage == NUM_DATA_EE_PAGES)
    {
        TBLPAG = savedTBLPAG;
        SetPageCorruptStatus(1);
        return(0xFFFF);     // Error - no active page
    }

    pmOffset = __builtin_tbloffset(&emulationPages) + (sizeof(emulationPages[0]) * (currentPage + 1) - 2);

    i=NUMBER_OF_INSTRUCTIONS_IN_PAGE;

    do
    {
        latch = ReadPMHigh(pmOffset);
        pmOffset -= 2;

        i--;
    }
    while((i > 0) && (latch != addr));

    if(!i)
    {
        SetaddrNotFound(1);
        TBLPAG = savedTBLPAG;
        return(0xFFFF);
    }

    pmOffset += 2;
    latch = ReadPMLow(pmOffset);

    TBLPAG = savedTBLPAG;
    return(latch);
}

/************************************************************************
PackEE

This routine finds the active page and an unexpired packed page. The most
recent data EEPROM values are located for each address using ReadEE
function and written into write latches. Page status is read from active
page and erase/write count is incremented if page 0 is packed. After all
information is programmed and verified, the current page is erased. The
packed page becomes the active page. This function can be called at any-
time by the user to schedule the CPU stall.

Parameters:		None
Return:			Status value (0 for pass)
Side Effects:	Generates CPU stall during program/erase operations and
                overwrites program memory write latches. Data EE flags
                may be updated
************************************************************************/
int PackEE(void)
{
    int currentPage;        //Array row (PM page) of active DEE page
    int packedPage;         //Array row (PM page) of packed page
    int savedTBLPAG;        //Context save of TBLPAG value. Current and packed page are on same page.
    int currentOffset;      //Current page offset
    int packedOffset;       //Packed page offset
    int i;
    unsigned char latchAddr;
    unsigned int latchData;
    unsigned char dataEEFlags_sh;

    savedTBLPAG = TBLPAG;

    // Point the table page pointer to the emulation pages
    TBLPAG = __builtin_tblpage( &(emulationPages[0]) );

    // Find the active page.
    for (currentPage = 0;
         (currentPage < NUM_DATA_EE_PAGES) &&
         (GetPageStatus(currentPage, STATUS_CURRENT) == PAGE_NOT_CURRENT);
         currentPage++) {}


    if (currentPage == NUM_DATA_EE_PAGES)
    {
        TBLPAG = savedTBLPAG;
        SetPagePackBeforeInit(1);
        return(3);      // Error - no active page
    }
    else
    {
        // Find the next unexpired page to use
        packedPage = currentPage + 1;
        if (packedPage == NUM_DATA_EE_PAGES)
        {
            packedPage = 0;
        }
        while(GetPageStatus(packedPage, STATUS_EXPIRED) == PAGE_EXPIRED)
        {
            packedPage++;
            if (packedPage == NUM_DATA_EE_PAGES)
            {
                packedPage = 0;
            }
            if(packedPage == currentPage)
            {
                TBLPAG = savedTBLPAG;
                SetPageExpiredPage(1);
                return(1);      // Error - all pages expired
            }
        }
    }

    // Point to first location in packed page
    packedOffset = __builtin_tbloffset( &emulationPages) + sizeof(emulationPages[0])*packedPage;

    if(GetNextAvailCount())
    {
        SetPagePackBeforePageFull(1);           // Pack called before the page was full
    }

    dataEEFlags_sh = dataEEFlags.val;
    SetaddrNotFound(0);                 // Initialize flag
    i = 0;
    NVMCON = PROGRAM_ROW;

    WritePMLow(0xFFFF, packedOffset);
    WritePMHigh(0xFF, packedOffset);
    packedOffset += 2;

    latchAddr = 0;
    i++;

    do
    {
        while((latchAddr != DATA_EE_SIZE) && (i < NUMBER_OF_INSTRUCTIONS_IN_ROW))
        {
            latchData = DataEERead(latchAddr);
            if(GetaddrNotFound())       //if address is unwritten, skip to next address
            {
                SetaddrNotFound(0);
            }
            else
            {
                WritePMLow(latchData, packedOffset);
                WritePMHigh(latchAddr, packedOffset);
                packedOffset += 2;
                i++;
            }
            latchAddr++;
            while((latchAddr == DATA_EE_SIZE) && (i < NUMBER_OF_INSTRUCTIONS_IN_ROW))
            {
                WritePMLow(0xFFFF, packedOffset);
                WritePMHigh(0xFF, packedOffset);
                packedOffset += 2;
                i++;
            }

        }
        UnlockWrite();
        i = 0;

    }
    while(latchAddr != DATA_EE_SIZE);

    dataEEFlags.val = dataEEFlags_sh;   //Restore status flags

    //Verify data was written correctly into packed page

    // Point to first location after status
    packedOffset = __builtin_tbloffset( &emulationPages) + sizeof(emulationPages[0])*packedPage + 2;

    latchAddr = ReadPMHigh(packedOffset++);
    latchData = ReadPMLow(packedOffset++);

    while(latchAddr != 0xFF)
    {
        if(DataEERead(latchAddr) != latchData)
        {
            TBLPAG = savedTBLPAG;
            SetPageWriteError(1);
            return(7);          //Error - data does not match
        }
       latchAddr = ReadPMHigh(packedOffset++);
       latchData = ReadPMLow(packedOffset++);
    }


    //Program page status
    currentOffset = __builtin_tbloffset( &emulationPages) + sizeof(emulationPages[0])*currentPage;
    packedOffset = __builtin_tbloffset( &emulationPages) + sizeof(emulationPages[0])*packedPage;

    latchData = ReadPMLow(currentOffset);
    latchAddr = ReadPMHigh(currentOffset);
    if(packedPage == 0)
    {
        latchData++;        //Increment E/W counter
    }

    if (latchData >= ERASE_WRITE_CYCLE_MAX - 1)
    {
        SetPageExpiredPage(1);
        latchAddr &= 0b11101111;
    }
    WritePMHigh(latchAddr, packedOffset);

    WritePMLow(latchData, packedOffset);

    NVMCON = PROGRAM_WORD;
    UnlockWrite();

    if((latchAddr != ReadPMHigh(packedOffset)) ||
        (latchData != ReadPMLow(packedOffset)))
    {
        TBLPAG = savedTBLPAG;
        SetPageWriteError(1);
        return(7);
    }

    //Erase active page
    ErasePage(currentPage);

    TBLPAG = savedTBLPAG;
    return(GetPageExpiredPage());
}

/************************************************************************
DataEEInit

This routine finds an unexpired page to become an active page. It then
counts the number of active pages. If no active pages are found, the
first unexpired page is initialized for emulation. If one active page is
found, it is assumes a reset occurred and the function does nothing. If
two active pages are found, it is assumes a reset occurred during a pack.
The second page is erased and a pack is called. If three, an error code
is returned as the allocated memory is assumed to be corrupted. This
function must be called prior to any other operation.

Parameters:		None
Return:			Status value (0 for pass)
Side Effects:	Data EE flags may be updated.
************************************************************************/
unsigned char DataEEInit(void)
{
    unsigned char pageCnt;
    unsigned char erasePage;
    unsigned int savedTBLPAG;        //Context save of TBLPAG value. Current and packed page are on same page.
    unsigned int currentPage;
    unsigned int statusOffset;
    int packedPage;         //Array row (PM page) of packed page

    savedTBLPAG = TBLPAG;
    pageCnt = 0;
    erasePage = 0;
    packedPage = 0;

    // Point the table page pointer to the emulation pages
    TBLPAG = __builtin_tblpage(&(emulationPages[0]));

    // Find unexpired page
    for (currentPage = 0;
        (currentPage < NUM_DATA_EE_PAGES) &&
        (GetPageStatus(packedPage, STATUS_EXPIRED) == PAGE_EXPIRED);
        currentPage++) {}

    if (currentPage == NUM_DATA_EE_PAGES)
    {
        TBLPAG = savedTBLPAG;
        SetPageExpiredPage(1);
        return(1);     // Error - All pages expired
    }

    // Count active page(s).
    for (currentPage = 0;(currentPage < NUM_DATA_EE_PAGES);currentPage++)
    {
         if(GetPageStatus(currentPage, STATUS_CURRENT) == PAGE_CURRENT)
         {
            pageCnt++;
         }
    }

    //If no active pages found, initialize page 0
    if(pageCnt == 0)
    {
        ErasePage(0);
        statusOffset = __builtin_tbloffset( &emulationPages);

        NVMCON = PROGRAM_WORD;

        WritePMLow(0, statusOffset);    //New page: unavailable, active, reset count
        WritePMHigh(0xF3, statusOffset);
        UnlockWrite();

        TBLPAG = savedTBLPAG;
        return(0);
    }
    //If one active page, do nothing
    else if(pageCnt == 1)
    {
        TBLPAG = savedTBLPAG;
        return(0);
    }
    //If two active pages, erase second and repack first
    else if(pageCnt == 2)
    {

        if((GetPageStatus(NUM_DATA_EE_PAGES - 1, STATUS_CURRENT) == PAGE_CURRENT) &&
            (GetPageStatus(0, STATUS_CURRENT) == PAGE_CURRENT))
        {
            currentPage = NUM_DATA_EE_PAGES - 1;
            erasePage = 0;
        }
        else
        {
            currentPage = 0;
            while((GetPageStatus(currentPage, STATUS_CURRENT) == PAGE_NOT_CURRENT) &&
                (currentPage < NUM_DATA_EE_PAGES))
            {
                currentPage++;
            }
            erasePage = erasePage + 1;
            if (erasePage == NUM_DATA_EE_PAGES)
            {
                erasePage = 0;
            }
        }
        ErasePage(erasePage);

        if(!GetNextAvailCount())
        {
            PackEE();
        }
        TBLPAG = savedTBLPAG;
        return(0);
    }
    else
    {
        TBLPAG = savedTBLPAG;
        SetPageCorruptStatus(1);
        return(6);
    }
}


/************************************************************************
DataEEWrite

This routine verifies the address is valid. If not, the Illegal Address
flag is set and an error code is returned. It then finds the active page.
If an active page can not be found, the Page Corrupt status bit is set
and an error code is returned. A read is performed, if the data was not
changed, the function exits. If the last location is programmed, the Pack
Skipped error flag is set (one location should always be available). The
data EE information (MSB = address, LSW = data) is programmed and
verified. If the verify fails, the Write Error flag is set. If the write
went into the last location of the page, pack is called. This function
can be called by the user.

Parameters:		Data EE address and data
Return:			Pass or fail status (0 = Pass)
Side Effects:	Data EE flags may be updated. CPU stall occurs for flash
                programming. Pack may be generated.
************************************************************************/
//uint8_T DataEEWrite(unsigned int data, unsigned int addr)
unsigned char DataEEWrite(unsigned int data, unsigned char addr)
{
    int savedTBLPAG;        //Context save of TBLPAG value. Current and packed page are on same page.
    int currentPage;
    int pmOffset;           //Current array (page) offset of selected element (PM 16-bit word)
    unsigned int nextLoc;
    volatile unsigned char latch;
    unsigned char dataEEFlags_sh;

    if(addr >= DATA_EE_SIZE)
    {
        SetPageIllegalAddress(1);
        return(5);
    }

    savedTBLPAG = TBLPAG;
    NVMCON = PROGRAM_WORD;

    // Point the table page pointer to the emulation pages
    TBLPAG = __builtin_tblpage(&(emulationPages[0]));

    // Find the active page.
    for (currentPage = 0;
         (currentPage < NUM_DATA_EE_PAGES) &&
         (GetPageStatus(currentPage, STATUS_CURRENT) == PAGE_NOT_CURRENT);
         currentPage++) {}

    if (currentPage == NUM_DATA_EE_PAGES)
    {
        TBLPAG = savedTBLPAG;
        SetPageCorruptStatus(1);
        return(6);      // Error - no active page
    }

    pmOffset = __builtin_tbloffset(&emulationPages) + sizeof(emulationPages[0]) * currentPage;

    dataEEFlags_sh = dataEEFlags.val;

    //Do not write data if it did not change
    if(DataEERead(addr) == data)
    {
        TBLPAG = savedTBLPAG;
        dataEEFlags.val = dataEEFlags_sh;
        return(0);
    }

    dataEEFlags.val = dataEEFlags_sh;       //Restore status flags
    nextLoc = GetNextAvailCount();

    if(!nextLoc)
    {
        TBLPAG = savedTBLPAG;
        SetPagePackSkipped(1);
        return(4);  //Error - Number of writes exceeds page size
    }

    pmOffset = pmOffset + nextLoc;

    WritePMLow(data, pmOffset);
    WritePMHigh(addr, pmOffset);

    UnlockWrite();

    latch = (ReadPMLow(pmOffset) & 0xFF);

    if(latch != (data & 0xFF))
    {
        TBLPAG = savedTBLPAG;
        SetPageWriteError(1);
        return(7);  //Error - RAM does not match PM
    }

    latch = (ReadPMHigh(pmOffset) & 0xFF);

    if(latch != addr)
    {
        TBLPAG = savedTBLPAG;
        SetPageWriteError(1);
        return(7);  //Error - RAM does not match PM
    }

    pmOffset += 1;
    latch = ((ReadPMLow(pmOffset) >> 8) & 0xFF);

    if(latch != ((data >> 8) & 0xFF))
    {
        TBLPAG = savedTBLPAG;
        SetPageWriteError(1);
        return(7);  //Error - RAM does not match PM
    }

    //Pack if page is full
    if ((nextLoc + 2) == ((NUMBER_OF_INSTRUCTIONS_IN_PAGE) * 2))
    {
        PackEE();
    }

    TBLPAG = savedTBLPAG;

    return(0);
}
