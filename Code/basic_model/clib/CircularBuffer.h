#ifndef _CIRCULAR_BUFFER_H_
#define _CIRCULAR_BUFFER_H_

#define BUFFER_SIZE 256

#define SUCCESS 1
#define STANDARD_ERROR 0
#define SIZE_ERROR -1

typedef struct {
	unsigned short startIndex; // Holds the index of the head of the list. Always points to valid data when empty is false.
	unsigned short endIndex; // Holds the index of the tail of the list. Always points to valid data when empty is false.
	unsigned short size;	// Stores the static size of the buffer.
	unsigned char empty; // A boolean value for whether the buffer is currently empty or not.
	unsigned char overflowCount; // Tracks how often the buffer is attempted to be written to when full.
	unsigned char buffer[BUFFER_SIZE];
} CircularBuffer;

/*
 * Buffer struct management functions.
 */
void InitCircularBuffer(CircularBuffer *b);

/*
 * Property getters.
 */
unsigned short GetLength(CircularBuffer *b);
unsigned char IsEmpty(CircularBuffer *b);
unsigned short GetStartIndex(CircularBuffer *b);
unsigned short GetEndIndex(CircularBuffer *b);
unsigned char GetOverflow(CircularBuffer *b);

/*
 * Buffer manipulation functions.
 */
unsigned char Peek(CircularBuffer *b);
unsigned char DeepPeek(CircularBuffer *b, unsigned short bytes, unsigned char *data);
unsigned char Read(CircularBuffer *b, unsigned char *data);
unsigned char Write(CircularBuffer *b, unsigned char data);

#endif /* _CIRCULAR_BUFFER_H_ */

