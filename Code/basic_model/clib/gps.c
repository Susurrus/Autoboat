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
// gps.c
// This is code implements a fully interrupt driven UART reader to
// be used in the UCSC Autopilot project. It makes use of the 
// circular buffer data structure circBuffer.c. It has been 
// written to be implemented in Simulink. It configures UART 1
// at a predefined baud rate, then initializes a circular buffer,
// configures the interrupt and starts the service. 
// The main function gpsRead returns an array where byte 0 indicates
// how many new bytes were read, and byte m indicates how many remain
// in the buffer.
// 
// Code by: Mariano I. Lizarraga
// First Revision: Aug 21 2008 @ 21:15
// ==============================================================

#include "gps.h"
#include "circBuffer.h"

#include <p33fxxxx.h>
#include <uart.h>

struct CircBuffer com1Buffer;
CBRef uartBuffer;
tGpsData gpsControlData;

// UART and Buffer initialization
void uartInit (void){

	
	// Configure and open the port;
	// U1MODE Register
	// ==============
	U1MODEbits.UARTEN	= 0;		// Disable the port		
	U1MODEbits.USIDL 	= 0;		// Stop on idle
	U1MODEbits.IREN		= 0;		// No IR decoder
	U1MODEbits.RTSMD	= 0;		// Ready to send mode (irrelevant)
	U1MODEbits.UEN		= 0;		// Only RX and TX
	U1MODEbits.WAKE		= 1;		// Enable at startup
	U1MODEbits.LPBACK	= 0;		// Disable loopback
	U1MODEbits.ABAUD	= 0;		// Disable autobaud
	U1MODEbits.URXINV	= 0;		// Normal operation (high is idle)
	U1MODEbits.PDSEL	= 0;		// No parity 8 bit
	U1MODEbits.STSEL	= 0;		// 1 stop bit
	U1MODEbits.BRGH 	= 0;		// Low speed mode
	
	// U1STA Register
	// ==============
	U1STAbits.URXISEL	= 2;		// RX interrupt when 3 chars are in
	U1STAbits.OERR		= 0;		// clear overun error
	
	// U1BRG Register
	// ==============
	U1BRG = UCSCAP_UBRGI;			// Set the baud rate for initial config of GPS

	// Enable the port;
	U1MODEbits.UARTEN	= 1;		// Enable the port	
	U1STAbits.UTXEN		= 1;		// Enable TX
	
	// wait for the UART to settle
	int i;
	for( i = 0; i < 32700; i += 1 )
	{
		Nop();
	}

	// Configure the GPS sentences and change the baud Rate
	gpsSentenceConfig();
	
	// Disable the port and TX;
	U1MODEbits.UARTEN	= 0;		// Disable the port	
	
	// U1BRG Register
	// ==============
	U1BRG = UCSCAP_UBRGF;			// Set the baud rate for operation of GPS	
	
	// Enable the port;
	U1MODEbits.UARTEN	= 1;		// Enable the port	
	U1STAbits.UTXEN		= 1;		// Enable TX
	
	for( i = 0; i < 32700; i += 1 )
	{
		Nop();
	}
	
	// Configure the frequency to 5 Hz
	gpsFreqConfig();
	
	// Disable the port and TX;
	U1MODEbits.UARTEN	= 0;		// Disable the port	


	// Initialize the Interrupt  
  	IPC2bits.U1RXIP   = 6;    		// Interrupt priority 6  
  	IFS0bits.U1RXIF   = 0;    		// Clear the interrupt flag
  	IEC0bits.U1RXIE   = 1;    		// Enable interrupts
 
  	// Enable the port;
	U1STAbits.UTXEN		= 0;		// Disable TX	
  	U1MODEbits.UARTEN	= 1;		// Enable the port		
}

void gpsSentenceConfig(void){
	int i,j;
	unsigned char chMsgs [] = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n\0";
	unsigned char chBaudRt [] = "$PMTK251,19200*22\r\n\0";
	
	// Put some huge delays to wait for GPS power-up without the need of a timer
	for( i = 0; i < 750; i += 1 ){
		for( j = 0; j < 32700; j += 1 )
		{
			Nop();
		}
	}
	
	putsUART1((unsigned int *)chMsgs);
	while(BusyUART1());	

	// Put some huge delays to wait for GPS power-up without the need of a timer
	for( i = 0; i < 150; i += 1 ){
		for( j = 0; j < 32700; j += 1 )
		{
			Nop();
		}
	}
	putsUART1((unsigned int *)chBaudRt);
	while(BusyUART1());
	
}

void gpsFreqConfig(void){
	unsigned char chFreq [] = "$PMTK300,200,0,0,0,0*2F\r\n\0";	
	putsUART1((unsigned int *)chFreq);
	while(BusyUART1());	
}

// Interrupt service routine for U1 GPS port
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void){
 
	// Read the buffer while it has data
	// and add it to the circular buffer
	while(U1STAbits.URXDA == 1){
		writeBack(uartBuffer, (unsigned char)U1RXREG);
	}
	
	// If there was an overun error clear it and continue
	if (U1STAbits.OERR == 1){
		U1STAbits.OERR = 0;
	}
	
	// clear the interrupt
	IFS0bits.U1RXIF = 0;
}

void gpsInit(void){
	// initialize the circular buffer
	uartBuffer = (struct CircBuffer* )&com1Buffer;
	newCircBuffer(uartBuffer);
	uartInit();
}

// this function converts one hex ascii character to decimal
// used for the checksum comparison
unsigned char hex2char (unsigned char halfhex){
	if ((halfhex - 48) < 9) {
		return (halfhex -48);
	}
	return (halfhex - 55);
}

float degMinToDeg(unsigned char degrees, float minutes){
	return ((float)degrees + minutes/60.0);
}


// this function reads the serial stream as it comes in
// then assembles full messages, verifies the checksum and
// if valid sends it out to the parser
unsigned char gpsSeparate( unsigned char* outStream)
{
	// Static variables CAREFUL
	static unsigned char outBuf [MSIZE] ={0};
	static unsigned char previousComplete = 1;
	static unsigned char indexLast = 0;
	
	// local variables
	unsigned char i=0;
	unsigned char tmpChksum = 0;
	unsigned char chsmStr_0, chsmStr_1;
	unsigned char isValid =0;
	unsigned char chksumHeader = 0;
	unsigned char tmpIndex = 0;
	unsigned char tmpLen = getLength(uartBuffer);
	// If the previous message was complete, then
	// go over the buffer and advance until you find a dollar sign
	if (previousComplete){
		while (tmpLen>0 && peak(uartBuffer)!= DOLLAR){
			readFront(uartBuffer);
			tmpLen--;
		}
	} 
	
	// read until you find a CR or run out of bytes to read
	while (tmpLen>0 && peak(uartBuffer)!= CR){
		outBuf[indexLast] = readFront(uartBuffer);
		indexLast++;
		tmpLen--;
	}

	// if we found a carriage return, then the message is complete
	if (peak(uartBuffer)== CR){
		// validate the checksum
		chsmStr_0 = hex2char(outBuf[indexLast - 2]);
		chsmStr_1 = hex2char(outBuf[indexLast - 1]);
		// convert the hex checksum to decimal
		tmpChksum = (unsigned char)(chsmStr_0 * 16 + chsmStr_1);
		// verify the validity
		isValid = (tmpChksum == getChecksum(outBuf,indexLast));		

		// turn on the flag of complete stream
		previousComplete = 1;
		// set the outStream valid flag
		outStream[MSIZE-1] = isValid;
		
		// save the index value for copying purposes
		tmpIndex = indexLast;
		
		// reset the array index
		indexLast = 0;			
	} else { // if we did not find a CR, i.e. we ran out of bytes
		// turn off the previousComplete flag
		previousComplete = 0;
	}
	
	// If we found a complete sentence and it is valid
	if(previousComplete && isValid)
	{
		// copy the data to the outstream
		for(i = 1; i <= tmpIndex; ++i)
		{
			outStream[i] = outBuf[i-1];
		}

		// finally compute the type of message
		chksumHeader = getChecksum(outBuf,6);

		// based on the obtained header checksum set the type
		switch (chksumHeader){
			case GGACS: 
				outStream[0] = GGAID;
				break;
			case RMCCS:
				outStream[0] = RMCID;
				break;
			default:
				outStream[0] = UNKID;
				break;
		}
	}
	return tmpLen;
}

void gpsParse(void){
	
	unsigned char inStream[MSIZE];
	unsigned char bufferLen = 11;
	
	memset(inStream,0,MSIZE);
	
	while (bufferLen>10){
		bufferLen = gpsSeparate(inStream);
		
		// if the sentence is valid
		if(inStream[MSIZE-1]==1){
			// parse the data according to the header
			switch (inStream[0]){
				case RMCID:
					parseRMC(inStream);
				break;
				case GGAID:
					parseGGA(inStream);
				break;
			}
		}else{
			// turn the flag off of new data
			gpsControlData.newValue = 0;
		}
		memset(inStream,0,MSIZE);
	}
	
}

void getGpsMainData (float* data){
	gpsParse();
	data[0] = gpsControlData.lat.flData;
	data[1] = gpsControlData.lon.flData;
	data[2] = gpsControlData.height.flData;
	data[3] = (float)gpsControlData.cog.usData;
	data[4] = (float)gpsControlData.sog.usData/100.0;
	
	// Add date info
	tFloatToChar tmp;
	tmp.chData[0] = gpsControlData.day;
	tmp.chData[1] = gpsControlData.month;
	tmp.chData[2] = gpsControlData.year;
	tmp.chData[3] = 0;
	data[5] = tmp.flData;
	
	// Add time info
	tmp.chData[0] = gpsControlData.hour;
	tmp.chData[1] = gpsControlData.min;
	tmp.chData[2] = gpsControlData.sec;
	tmp.chData[3] = 0;
	data[6] = tmp.flData;
	
}


char gpSmbl(char symbl)
{	
	switch (symbl)
	{	case 'N': return 1; /*North*/
		case 'E': return 1; /*East*/
		case 'S': return -1;/*South*/
		case 'W': return -1;/*West*/
		case 'A': return 1; /*Valid or Autonomous*/
		case 'D': return 1; /*Differential*/
		case 'V': return 0; /*Non-Valid*/
		case 'M': return 100; /*Meters*/
		default:  return 0; /*Non-Valid*/
	}
}

// a return value of 1 means the string is done. No more tokens
// This function is stateful, call it once with the String and then with NULL
// similar to strtok but this will support succesive tokens like
// "find,,,the,,commas"

unsigned char myTokenizer ( char* stringToTokenize, char token, char * returnToken){
	static char * pch;
	static char * prevPch;
	static char * lastByte;
	
	// Make sure the return token is "empty"
	memset(returnToken, 0, TOKEN_SIZE);
	
	// get the pointer to next token if it exists 
	// and the stringToTokenize is null
	// Bahavior similar to strtok
	if (stringToTokenize == NULL) {
		pch = strchr(prevPch, token);
	} else {
		pch = strchr(stringToTokenize, token);
		prevPch = stringToTokenize;
		lastByte = stringToTokenize + strlen(stringToTokenize);
	} 
	
	if(pch != NULL){
		memcpy(returnToken, prevPch, pch-prevPch );
		prevPch = pch+1;
	} else {
		memcpy(returnToken, prevPch, lastByte-prevPch );
	}

	return pch == NULL;
}



void parseRMC(unsigned char* stream){
	// declare the local vars
	char token[TOKEN_SIZE];
	char tmp [3] ={0,0,'\0'}, tmp3[4]={0,0,0,'\0'};
	unsigned char chTmp = 0;
	
	
	// initialize tokenizer, let go first token which holds the msg type
	// token = strtok(stream, ",");
	myTokenizer(stream, ',', token); 
	
	// 1.- hhmmss.ssss
	myTokenizer(NULL, ',', token);
	if (strlen(token)>5){
		tmp[0] = token[0]; tmp[1] = token[1];
		gpsControlData.hour = (unsigned char) atoi(tmp);
		tmp[0] = token[2]; tmp[1] = token[3];
		gpsControlData.min = (unsigned char) atoi(tmp);
		tmp[0] = token[4]; tmp[1] = token[5];
		gpsControlData.sec = (unsigned char) atoi(tmp);		
	}
	
	// 2.- Status of position Fix
	myTokenizer(NULL, ',', token);
	if (strlen(token)== 1){
		gpsControlData.fix = gpSmbl((char)token[0]);
	}
	
	// 3.- Latitude
	// ddmm.mmmmmm
	myTokenizer(NULL, ',', token);
	if (strlen(token)>0){
		// get the first two values
		tmp[0] = token[0]; tmp[1] = token[1];
		// get the degrees
		chTmp = (unsigned char)atoi(tmp);
		// make the degrees zero for minutes conversion
		token[0]='0'; token[1]='0';
		// get the float
		gpsControlData.lat.flData =  degMinToDeg(chTmp,atof(token));
		
		// 4.- Latitude Sector
		myTokenizer(NULL, ',', token);
		if (strlen(token)==1){
			// set the sign of the float value
			gpsControlData.lat.flData *= gpSmbl((char)token[0]);
		}
	}
	
	// 5.- Longitude
	// dddmm.mmmmmm
	myTokenizer(NULL, ',', token);
	if (strlen(token)>0){
		// get the first two values
		tmp3[0] = token[0]; tmp3[1] = token[1]; tmp3[2] = token[2];
		// get the degrees
		chTmp = (unsigned char)atoi(tmp3);
		// make the degrees zero for minutes conversion
		token[0]='0'; token[1]='0'; token [2] = '0';
		// get the float
		gpsControlData.lon.flData =  degMinToDeg(chTmp,atof(token));
		
		// 6.- Longitude Sector
		myTokenizer(NULL, ',', token);
		if (strlen(token)==1){
			// set the sign of the float value
			gpsControlData.lon.flData *= gpSmbl((char)token[0]);
		}
	}
	
	// 7.- SOG in knots but gets stored in cm/s CAUTION
	// xx.xx
	myTokenizer(NULL, ',', token);	
	if (strlen(token)>0){
		gpsControlData.sog.usData = (unsigned short) (atof(token)*KTS2MPS*100.0);	
	}
	
	// 8.- COG in degrees
	// xxx.xxx
	myTokenizer(NULL, ',', token);	
	if (strlen(token)>0){
		gpsControlData.cog.usData = (unsigned short) atof(token);	
	}
	
	// 9.- UTC Date
	// ddmmyy
	myTokenizer(NULL, ',', token);	
	if (strlen(token)>5){
		// get day
		tmp[0]= token[0]; tmp[1]=token[1];
		gpsControlData.day = (unsigned char) atoi(tmp);	
		// get month
		tmp[0]= token[2]; tmp[1]=token[3];
		gpsControlData.month = (unsigned char) atoi(tmp);	
		// get year
		tmp[0]= token[4]; tmp[1]=token[5];
		gpsControlData.year = (unsigned char) atoi(tmp);	
	}
	
	// turn the flag on of new data
	gpsControlData.newValue = 1;
}

void parseGGA(unsigned char* stream){
	// declare the local vars
	char token[TOKEN_SIZE];
	char tmp [3] ={0,0,'\0'}, tmp3[4]={0,0,0,'\0'};
	unsigned char chTmp = 0;
	
	// initialize tokenizer, let go first token which holds the msg type
	myTokenizer(stream, ',', token);
	
	// 1.- hhmmss.ssss
	myTokenizer(NULL, ',', token);
	if (strlen(token)>5){
		tmp[0] = token[0]; tmp[1] = token[1];
		gpsControlData.hour = (unsigned char) atoi(tmp);
		tmp[0] = token[2]; tmp[1] = token[3];
		gpsControlData.min = (unsigned char) atoi(tmp);
		tmp[0] = token[4]; tmp[1] = token[5];
		gpsControlData.sec = (unsigned char) atoi(tmp);		
	}	
	// 2.- Latitude
	// ddmm.mmmmmm
	myTokenizer(NULL, ',', token);
	if (strlen(token)>0){
		// get the first two values
		tmp[0] = token[0]; tmp[1] = token[1];
		// get the degrees
		chTmp = (unsigned char)atoi(tmp);
		// make the degrees zero for minutes conversion
		token[0]='0'; token[1]='0';
		// get the float
		gpsControlData.lat.flData =  degMinToDeg(chTmp,atof(token));		
		// 3.- Latitude Sector
		myTokenizer(NULL, ',', token);
		if (strlen(token)==1){
			// set the sign of the float value
			gpsControlData.lat.flData *= gpSmbl((char)token[0]);
		}
	}
	
	// 4.- Longitude
	// ddmm.mmmmmm
	myTokenizer(NULL, ',', token);	
	if (strlen(token)>0){
		// get the first two values
		tmp3[0] = token[0]; tmp3[1] = token[1]; tmp3[2] = token[2];
		// get the degrees
		chTmp = (unsigned char)atoi(tmp3);
		// make the degrees zero for minutes conversion
		token[0]='0'; token[1]='0'; token [2] = '0';
		// get the float
		gpsControlData.lon.flData =  degMinToDeg(chTmp,atof(token));
		
		// 5.- Longitude Sector
		myTokenizer(NULL, ',', token);
		
		if (strlen(token)>0){
			// set the sign of the float value
			gpsControlData.lon.flData *= gpSmbl((char)token[0]);
		}
	}
	
	// 6.- Quality Indicator
	myTokenizer(NULL, ',', token);
	if (strlen(token)== 1){
		gpsControlData.fix = (char)atoi(token);
	}

	// 7.- Sats used in solution
	// xx
	myTokenizer(NULL, ',', token);	
	if (strlen(token)>0){
		gpsControlData.sats = (unsigned char) atoi(token);	
	}
	
	// 8.- HDOP given from 0 to 99.99 but stored from 0 - 990 
	//in integers, i.e HDOP = HDOP_stored/100 CAUTION
	// xx.xx
	myTokenizer(NULL, ',', token);	
	if (strlen(token)>0){
		gpsControlData.hdop.usData = (unsigned short) (atof(token)*10.0);	
	}
	
	// 9.- Altitude above mean sea level given in meters
	// xxx.xxx
	myTokenizer(NULL, ',', token);	
	if (strlen(token)>0){
		gpsControlData.height.flData =  atof(token);	
	}
	
	// turn the flag on of new data
	gpsControlData.newValue = 1;
}
