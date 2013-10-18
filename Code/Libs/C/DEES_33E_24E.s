;*************************************************************************
;*
;*   Emulating Data EEPROM for PIC24 Microcontrollers and
;*           dsPIC Digital Signal Controllers
;*
;*************************************************************************
;* FileName:     Flash Operations.s
;* Compiler:     MPLAB C30, v3.30 or higher
;* Company:      Microchip Technology, Inc.
;*
;* Software License Agreement
;*
;* Copyright © 2007 Microchip Technology Inc. All rights reserved.
;*
;* Microchip licenses to you the right to use, modify, copy and distribute
;* Software only when embedded on a Microchip microcontroller or digital
;* signal controller, which is integrated into your product or third party
;* product (pursuant to the sublicense terms in the accompanying license
;* agreement).
;*
;* You should refer to the license agreement accompanying this Software for
;* additional information regarding your rights and obligations.
;*
;* SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY
;* KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
;* WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A
;* PARTICULAR PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE
;* LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY,
;* CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY
;* DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
;* INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST
;* PROFITS OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY,
;* SERVICES, OR ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO
;* ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
;*
;* Author        Date        Comment
;*************************************************************************
;* P. Sinha      2011/01/20  Initial Release
;* Anantha R	 2011/12/20  Modified to ensure proper packing
;************************************************************************/
.global _ReadPMHigh
.global _ReadPMLow
.global _UnlockPM
.global _WritePMHigh
.global _WritePMHighB
.global _WritePMLow
.global _WritePMLowB

.section .text

_ReadPMHigh:
	tblrdh [W0],W0
	return

_ReadPMLow:
	tblrdl [W0],W0
	return		
	
_UnlockPM:
	push	W0
	disi	#5
	mov		#0x55,W0
	mov		W0, NVMKEY
	mov		#0xAA, W0
	mov		W0, NVMKEY
	bset	NVMCON, #15
	nop
	nop
	btsc	NVMCON, #15
	bra		$-2
	pop		W0
	return	
	
; Special handling for dsPIC33E/PIC24E to account for RTSP differences from dsPIC33F/PIC24H while retaining same API

_WritePMHigh:
	; Store current TBLPAG value and passed target address in NVMADRU and NVMADR respectively
	mov		TBLPAG, W2
	mov		W2, NVMADRU
	mov		W1, NVMADR
	; Set up latch pointer depending on whether required operation is Word Program or not
	mov		#0x4001, W4
	mov		NVMCON, W3
	cp		W3, W4
	bra		Z, ahead1
	and		#0xFF, W1
	bra		ahead2
ahead1:
	; If the address to be written is an even word, then keep a copy of the corresponding odd word too
	; If the address to be written is an odd word, then keep a copy of the corresponding even word too
	mov		W1, W4
	btg		W4, #1 
	tblrdh	[W4], W5
	and		#0x03, W1
	; Save the current TBLPAG value and set up TBLPAG to point to the latch area
	push	TBLPAG
	mov		#0xFA, W6
	mov		W6, TBLPAG
	; Perform the required latch write
	tblwth	W0,[W1]
	; If the address to be written is an even word, then write the stored value of the odd word
	; If the address to be written is an odd word, then write the stored value of the even word
	btg		W1, #1 
	tblwth	W5, [W1]
	bra		ahead3
ahead2:
	; Save the current TBLPAG value and set up TBLPAG to point to the latch area
	push	TBLPAG
	mov		#0xFA, W6
	mov		W6, TBLPAG
	tblwth	W0,[W1]
ahead3:
	; Restore the original TBLPAG value
	pop		TBLPAG
	return

_WritePMLow:
	; Store current TBLPAG value and passed target address in NVMADRU and NVMADR respectively
	mov		TBLPAG, W2
	mov		W2, NVMADRU
	mov		W1, NVMADR
	; Set up latch pointer depending on whether required operation is Word Program or not
	mov		#0x4001, W4
	mov		NVMCON, W3
	cp		W3, W4
	bra		Z, ahead4
	and		#0xFF, W1
	bra		ahead5
ahead4:
	; If the address to be written is an even word, then keep a copy of the corresponding odd word too
	; If the address to be written is an odd word, then keep a copy of the corresponding even word too
	mov		W1, W4
	btg		W4, #1 
	tblrdl	[W4], W5
	and		#0x03, W1
	; Save the current TBLPAG value and set up TBLPAG to point to the latch area
	push	TBLPAG
	mov		#0xFA, W6
	mov		W6, TBLPAG
	; Perform the required latch write
	tblwtl	W0,[W1]
	; If the address to be written is an even word, then write the stored value of the odd word
	; If the address to be written is an odd word, then write the stored value of the even word
	btg		W1, #1 
	tblwtl	W5, [W1]
	bra		ahead6
ahead5:
	; Save the current TBLPAG value and set up TBLPAG to point to the latch area
	push	TBLPAG
	mov		#0xFA, W6
	mov		W6, TBLPAG
	tblwtl	W0,[W1]
ahead6:
	; Restore the original TBLPAG value
	pop		TBLPAG
	return
	
.end
