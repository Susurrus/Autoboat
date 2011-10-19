#include "nmea.h"

#include <string.h>

void buildAndCheckSentence(unsigned char characterIn, char *sentence, unsigned char *sentenceIndex, unsigned char *sentenceState, unsigned char *checksum, void (*processResult)(char *)) {
	// Full specification for NMEA0138 specifies a maximum sentence length
	// of 255 characters. We're going to ignore this for half the length as
	// we shouldn't get anything that big.

	// This contains the function's state of whether
	// it is currently building a sentence.
	// 0 - Awaiting start character ($)
	// 1 - Building sentence
	// 2 - Building first checksum character
	// 3 - Building second checksum character
	
	// We start recording a new sentence if we see a dollarsign.
	// The sentenceIndex is hard-set to 1 so that multiple dollar-signs
	// keep you at the beginning.
	if (characterIn == '$') {
		sentence[0] = characterIn;
		(*sentenceIndex) = 1;
		(*sentenceState) = 1;
	} else if ((*sentenceState) == 1) {
		// Record every character that comes in now that we're building a sentence.
		// Only stop if we run out of room or an asterisk is found.
		sentence[(*sentenceIndex)++] = characterIn;
		if (characterIn == '*') {
			(*sentenceState) = 2;
		} else if ((*sentenceIndex) > 127) {
			// If we've filled up the buffer, ignore the entire message as we can't store it all
			(*sentenceState) = 0;
			(*sentenceIndex) = 0;
		}
	} else if ((*sentenceState) == 2) {
		// Record the first ASCII-hex character of the checksum byte.
		(*checksum) = hex2char(characterIn) << 4;
		(*sentenceState) = 3;
	} else if ((*sentenceState) == 3) {
		// Record the second ASCII-hex character of the checksum byte.
		(*checksum) |= hex2char(characterIn);

		// Now that we've compiled a complete GPS sentence, let's check the checksum and parse it.
		// This code currently only supports RMC and GGA messages.
		unsigned char test = getChecksum(sentence, (*sentenceIndex));
		if ((*checksum) == test) {
			processResult(sentence);
		}
		
		// We clear all state variables here regardless of success.
		(*sentenceIndex) = 0;
		(*sentenceState) = 0;
	}
}

unsigned char myTokenizer(char* stringToTokenize, char token, char * returnToken) {
	static char *pch;
	static char *prevPch;
	static char *lastByte;
	
	// Make sure the return token is "empty"
	// Tokens set to max-length of 15 bytes
	memset(returnToken, 0, 15);
	
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
	
	if (pch != NULL) {
		memcpy(returnToken, prevPch, pch-prevPch );
		prevPch = pch+1;
	} else {
		memcpy(returnToken, prevPch, lastByte-prevPch );
	}

	return pch == NULL;
}

unsigned char getChecksum(char* sentence, unsigned char size) {

    // Loop through all chars to get a checksum
    unsigned char checkSum = 0;
	unsigned char i;
	for (i = 0; i < size; i++) {
		if (sentence[i] == '$') {
			// Ignore the dollar sign
			continue;
		} else if (sentence[i] == '*') {
			// Stop processing before the asterisk
			break;
		} else {
			checkSum ^= sentence[i];
		}
    }
    // Return the checksum 
    return checkSum;
}
