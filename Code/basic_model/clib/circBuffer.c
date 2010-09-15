/*
The MIT License

Copyright (c) 2009 UCSC Autonomous Systems Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

// ==============================================================
// circBuffer.c
// This is the implementation file for the circular buffer. This library
// implements a circular buffer to be used in the serial readers
// for the GPS, IPC and telemetry decoders of the UCSC AutoPilot. 
//
// Comment the DEBUG define int the header prior to compile for dsPIC
//
// Convemtion Notes
// The HEAD, as in any queue points to the next available byte to be READ
// The TAIL, points to the next available position to be written
// 
// Code by: Mariano I. Lizarraga
// First Revision: Aug 16 2008 @ 00:36
// Second Revision: Dec 2 2008 @ 12:11
// ==============================================================

#include "circBuffer.h"

// Constructors - Destructors
// ==========================
// this Function returns a pointer to a new Circular Buffer of size pm_size 

void newCircBuffer (CBRef cB){
	
	// initialize to zero
	int i;
	for (i=0; i<BSIZE; i++){
		cB->buffer[i] = 0;
	}
			
	// initialize the data members
	cB->head = 0;
	cB->tail = 0;
	cB->size = BSIZE;
	cB->overflowCount = 0;

}

// this function frees the Circular Buffer CB Ref
void freeCircBuffer (CBRef* cB){
	// if it is already null, nothing to free
	if (cB == NULL || *cB == NULL) {return;}
			
	// free and nil the pointer
	//free(*cB);
	*cB = NULL;
}

// Accesor Methods
// ===============

// returns the amount of unread bytes in the circular buffer
unsigned int getLength (CBRef cB){	
	// if the circular buffer is not null
	if (cB != NULL){
		if (cB->head <= cB->tail){
			return (cB->tail-cB->head);
		} else{
			return (cB->size + cB->tail - cB->head);
		}		
	}
	else{
		return 0;
	}
	

}

// returns the actual index of the head
int readHead (CBRef cB){
	// if the circular buffer is not null
	if (cB != NULL){
		return (cB->head);
	}
	else{
		return 0;
	}

}

// returns the actual index of the tail
int readTail (CBRef cB){
	// if the circular buffer is not null
	if (cB != NULL){
		return (cB->tail);
	}
	else{
		return 0;
	}

}

// returns the byte (actual value) that the head points to. this
// does not mark the byte as read, so succesive calls to peek will
// always return the same value
unsigned char myPeek(CBRef cB){
	// if the circular buffer is not null
	if (cB != NULL)
	{	
		// if there are bytes in the buffer
		if (getLength(cB) > 0){
			return cB->buffer[cB->head];
		}
	}
	return 0;	
}


// Manipulation Procedures
// ======================
// returns the front of the circular buffer and marks the byte as read
unsigned char readFront (CBRef cB){
	// if the circular buffer is not null
	if (cB != NULL)
	{	
		char retVal;
		// if there are bytes in the buffer
		if (getLength(cB) > 0){
			retVal = cB->buffer[cB->head];
			cB->head = cB->head < (cB->size -1)? cB->head+1: 0;
			return retVal;
		}
		return 128;
	}
	return 254;
}

// writes one byte at the end of the circular buffer, 
// increments overflow count if overflow occurs
unsigned char writeBack (CBRef cB, unsigned char data){
	// if the circular buffer is not null
	if (cB != NULL){			
		if (getLength (cB) == (cB->size -1)){
			cB->overflowCount ++;
			return 1;
		} else {		
			cB->buffer[cB->tail] = data;
			cB->tail = cB->tail < (cB->size -1)? cB->tail+1: 0;
			return 0;
		}
		return 0;
	}
	else{
		return 1;
	}
}

// empties the circular buffer. It does not change the size. use with caution!!
void makeEmpty(CBRef cB){
	if (cB != NULL){
		int i;
		for(i = 0; i < cB->size; ++i)
		{
			cB->buffer[i]= 0;
		}
		cB->head = 0;
		cB->tail = 0;
		cB->overflowCount = 0;
	}
}

// returns the amount of times the CB has overflown;
unsigned char getOverflow(CBRef cB){
	if (cB != NULL){
		return cB->overflowCount;
	}
	return 1;
}
