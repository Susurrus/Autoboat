#include "CircularBuffer.h"
#include <stddef.h>

#ifdef UNIT_TEST
#include <string.h>
#include <stdio.h>
#include <assert.h>
#endif // UNIT_TEST

/**
 * Initializes the passed CircularBuffer to the proper values. This means both
 * indices are 0, the size is BUFFER_SIZE, and the overflowCount is 0. Note that
 * this function is safe to reuse on an existing circular buffer to reset it.
 */
void InitCircularBuffer(CircularBuffer *b)
{
	
	int i;
	for (i = 0; i < BUFFER_SIZE; ++i) {
		b->buffer[i] = 0;
	}
	
	b->startIndex = 0;
	b->endIndex = 0;
	b->size = BUFFER_SIZE;
	b->empty = 1;
	b->overflowCount = 0;
}

/**
 * Returns the current number of elements sitting in b.
 */
unsigned short GetLength(CircularBuffer *b)
{
	if (b != NULL) {
		if (b->empty) {
			return 0;
		} else if (b->startIndex <= b->endIndex) {
			return b->endIndex - b->startIndex + 1;
		} else {
			return b->size + b->endIndex - b->startIndex + 1;
		}
	}
	return SIZE_ERROR;
}

/**
 * IsEmpty() returns whether the buffer is empty or not. Returns STANDARD_ERROR
 * if the buffer is a null-pointer.
 */
unsigned char IsEmpty(CircularBuffer *b)
{
	if (b != NULL) {
		return b->empty;
	}
	return STANDARD_ERROR;
}


/**
 * Returns the startIndex member of b. Returns STANDARD_ERROR if passed NULL.
 */
unsigned short GetStartIndex(CircularBuffer *b)
{
	if (b != NULL) {
		return b->startIndex;
	}
	return STANDARD_ERROR;
}

/**
 * Returns the endIndex member of b. Returns STANDARD_ERROR if passed NULL.
 */
unsigned short GetEndIndex(CircularBuffer *b)
{
	if (b != NULL) {
		return b->endIndex;
	}
	return STANDARD_ERROR;
}

/**
 * GetOverflow() returns the overflow counter of the passed CircularBuffer. If
 * b is NULL, return 0.
 */
unsigned char GetOverflow(CircularBuffer *b)
{
	if (b != NULL) {
		return b->overflowCount;
	}
	return STANDARD_ERROR;
}

/**
 * Peek() is the same as Read() but it doesn't pop the element off of the
 * queue.
 */
unsigned char Peek(CircularBuffer *b)
{
	if (b != NULL) {
		if (GetLength(b) > 0) {
			return b->buffer[b->startIndex];
		}
	}
	return STANDARD_ERROR;
}

/**
 * DeepPeek() is an extension of Peek() to byte-arrays. Given a desired number of bytes,
 * DeepPeek will copy those bytes into the passed byte-array. If there aren't enough bytes
 * in the buffer, then STANDARD_ERROR is returned. This is also the case if the CircularBuffer
 * pointer is NULL.
 */
unsigned char DeepPeek(CircularBuffer *b, unsigned short bytes, unsigned char* data){
	unsigned short i;
	int tmpHead;
	
	// if the circular buffer is not null
	if (b != NULL) {
		// if there are bytes in the buffer
		if (GetLength(b) >= bytes) {
			tmpHead = b->startIndex;
			for (i = 0;i < bytes;i++) {
				data[i] = b->buffer[tmpHead];
				if (tmpHead < b->size - 1) {
					++tmpHead;
				} else {
					tmpHead = 0;
				}
			}
			return SUCCESS;
		}
	}
	return STANDARD_ERROR;
}

/**
 * Read() is the inverse of Write(), it reads a single value from b
 * and places it in returnValue. It returns STANDARD_ERROR if b was NULL or had
 * no data to return.
 */
unsigned char Read(CircularBuffer *b, unsigned char *returnValue)
{
	if (b != NULL) {
		if (!b->empty) {
			*returnValue = b->buffer[b->startIndex];
			if (GetLength(b) == 1) {
				b->empty = 1;
			} else {
				b->startIndex = b->startIndex < (b->size - 1)?b->startIndex + 1:0;
			}
			return SUCCESS;
		}
	}
	return STANDARD_ERROR;
}

/**
 * Write() writes a new unsigned char into CircularBuffer b. SUCCESS is
 * returned if that value was successfully added. STANDARD_ERROR is returned if
 * the buffer overflows or b was NULL. If the buffer overflows the new item is
 * not inserted.
 */
unsigned char Write(CircularBuffer *b, unsigned char data)
{
	if (b != NULL) {
		if (GetLength(b) == b->size) {
			b->overflowCount++;
			return STANDARD_ERROR;
		} else {
			if (b->empty) {
				b->empty = 0;
			} else {
				b->endIndex = b->endIndex < (b->size - 1) ? b->endIndex + 1: 0;
			}
			b->buffer[b->endIndex] = data;
			return SUCCESS;
		}
	}
	return STANDARD_ERROR;
}

#ifdef UNIT_TEST
int main()
{
	
	printf("Running unit tests. If no message about failed assertions are triggered, then all tests passed.\n");
	
	// Create a new circular buffer.
	CircularBuffer b;
	InitCircularBuffer(&b);
	assert(IsEmpty(&b));
	
	// Add a single item and check.
	Write(&b, 0x56);
	assert(GetLength(&b) == 1);
	assert(!IsEmpty(&b));
	assert(Peek(&b) == 0x56);
	
	// Remove that item and check.
	unsigned char d;
	assert(Read(&b, &d) && d == 0x56);
	assert(GetLength(&b) == 0);
	assert(IsEmpty(&b));
	assert(Peek(&b) == 0);
	
	// Re-initialize the circular buffer to completely reset all indices.
	// Note that if this fails it's actually the next tests that will fail.
	InitCircularBuffer(&b);
	assert(IsEmpty(&b));
	
	// Here we make a 1016 character long string for testing. Testing with the library with BUFFER_SIZE
	// set to larger than 1016 will produce errors.
	char testString[] = "Copyright (C) <year> <copyright holders> Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the \"Software\"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THEAUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.";
	// Fill the buffer to SIZE+1 and check.
	unsigned short i;
	for (i = 0; i < b.size; i++) {
		assert(GetLength(&b) == i);
		assert(Write(&b, testString[i]));
		assert(GetLength(&b) == i + 1);
	}
	assert(!Write(&b, 0x89));
	assert(GetOverflow(&b) == 1);
	
	// Run a deepPeek on the now-full buffer.
	char tmpString[b.size];
	assert(DeepPeek(&b, b.size, tmpString));
	assert(GetLength(&b) == b.size);
	assert(memcmp(testString, tmpString, b.size) == 0);
	assert(!IsEmpty(&b));
	
	// Verify reading of an entire circular-buffer
	for (i = GetLength(&b); i > 0; --i) {
		assert(Read(&b, &d));
		assert(d == testString[b.size - i]);
	}
	assert(GetLength(&b) == 0);
	d = 0x77;
	char d2 = d;
	assert(!Read(&b, &d));
	assert(d == d2);
	
	// Test element insertion when the buffer wraps around.
	assert(Write(&b, 91));
	assert(GetOverflow(&b) == 1); // Overflow is triggered on an earlier test
	assert(!IsEmpty(&b));
	assert(GetLength(&b) == 1);
	assert(Peek(&b) == 91);
	assert(Write(&b, 92));
	assert(!IsEmpty(&b));
	assert(GetLength(&b) == 2);
	assert(Write(&b, 93));
	assert(!IsEmpty(&b));
	assert(GetLength(&b) == 3);
	assert(Write(&b, 94));
	assert(!IsEmpty(&b));
	assert(GetLength(&b) == 4);
	
	// Test DeepPeek on wrapped-around buffers
	unsigned char peekData[4];
	assert(DeepPeek(&b, 4, peekData));
	assert(peekData[0] == 91);
	assert(peekData[1] == 92);
	assert(peekData[2] == 93);
	assert(peekData[3] == 94);
	
	// Test reading now.
	assert(Read(&b, &d) && d == 91);
	assert(Read(&b, &d) && d == 92);
	assert(Read(&b, &d) && d == 93);
	assert(Read(&b, &d) && d == 94);
	assert(!Read(&b, &d) && d == 94);
	
	return 0;
}
#endif // UNIT_TEST
