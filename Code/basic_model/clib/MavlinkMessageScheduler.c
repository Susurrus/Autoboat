/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 */

// Include user headers
#include "MavlinkMessageScheduler.h"
#ifndef UNIT_TEST
#include "mavlink.h"
#endif

// Include system libraries
#include <stdlib.h>
#include <limits.h>
#include <math.h>

// Now if we're unit testing just use our own generated test data. This removes the algorithm testing from the intricacies of changes to MAVLink messages & message sizes.
#ifdef UNIT_TEST
#define MAVLINK_MESSAGE_LENGTHS {9, 31, 12, 0, 14, 28, 3, 32, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 20, 2, 25, 23, 30, 101, 22, 26, 16, 14, 28, 32, 28, 28, 22, 22, 21, 6, 6, 37, 4, 4, 2, 2, 4, 2, 2, 3, 13, 12, 19, 17, 15, 15, 27, 25, 18, 18, 20, 20, 9, 54, 26, 0, 36, 0, 6, 4, 0, 21, 18, 0, 0, 0, 20, 0, 33, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 56, 42, 33, 0, 0, 0, 0, 0, 0, 0, 18, 32, 32, 20, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 6, 21, 0, 0, 0, 0, 0, 0, 0, 4, 18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 36, 30, 18, 18, 51, 9, 0}
#define MAVLINK_NUM_NON_PAYLOAD_BYTES 5
#endif

// Store a linked list of messages to send at every timestep
static SListItem *timesteps[100];

// And keep track of which timestep we're currently on
static uint8_t currentTimestep = 0;

// Keep a data store of how big each message is.
static uint8_t messageSizes[256] = MAVLINK_MESSAGE_LENGTHS;

bool AddMessage(uint8_t id, uint8_t rate)
{
	// Be sure that we only process messages are reasonable rates
	if (rate < 1 || rate > 100) {
		return 0;
	}


	// Just store this for future use as it's used quite often.
	float period = 100.0/((float)rate);
	
	//TODO: Add a verification step that this message can even be safely added.
	// This would require knowing the size of the new message and then comparing
	// the integral of all timestep bytes with MESSAGE_TRANSMISSION_RATE. If it's
	// over the top we should return a failure.
	
	// Now find a decent offset for this message to use for its transfer rate.
	// We first go through every possible offset and determine which one provides
	// the emptiest channel for transmission based on total bytes sent during that
	// transmission window.
	uint8_t bestOffset;
	uint16_t lastCost = USHRT_MAX;
	uint8_t offset;
	for (offset = 0; offset < (uint8_t)period; offset++) {
		uint16_t currentCost = 0;
		float timestep = offset;
		uint8_t i;
		for (i = 0; i < rate && timestep < 100; timestep += period, i++) {
			uint8_t roundedTimestep = floorf(timestep);
			SListItem *j;
			for	(j = timesteps[roundedTimestep]; j; j = j->sibling) {
				currentCost += messageSizes[j->id] + MAVLINK_NUM_NON_PAYLOAD_BYTES;
			}
		}
		// If we've found a better offset, store it.
		if (currentCost < lastCost) {
			bestOffset = offset;
			lastCost = currentCost;
		}
	}
	
	// Finally add this message onto the list of messages to send using the
	// best offset that was found. Messages are prepended to every timestep list
	// of messages that will be transmit. This keep insertion at O(n) worst case.
	
	// First allocate enough messages here. This allows us to check for malloc failure
	// and deallocate successful ones easily. We assume that if one malloc failure
	// occurs, then all the rest will also.
	SListItem *newMessages[rate];
	uint8_t i;
	for (i = 0; i < rate; i++) {
		newMessages[i] = malloc(sizeof(SListItem));
		// If we were able to malloc the item, populate it and continue.
		if (newMessages[i] != NULL) {
			newMessages[i]->id = id;
			newMessages[i]->_transient = false;
		}
		// Otherwise if malloc() has failed, back up through the array and free any ones that
		// were allocated and return failure.
		else {
			uint8_t j;
			for (j = i; j > 0; --j) {
				free(newMessages[j]);
			}
			return false;
		}
	}
	
	// Now if we're here we have all the new messages ready to be inserted into our list,
	// so let's go ahead and insert them.
	float timestep = bestOffset;
	for (i = 0; i < rate && timestep < 100; timestep += period, i++) {
		uint8_t roundedTimestep = floorf(timestep);
		
		// Either add as a new head to an existing list or as a new list.
		newMessages[i]->sibling = timesteps[roundedTimestep];
		timesteps[roundedTimestep] = newMessages[i];
	}
	
	return true;
}

bool AddTransientMessage(uint8_t id)
{
	// Find the smallest bracket in the next second to transmit this message
	uint16_t lastCost = USHRT_MAX;
	uint8_t bestTimestep;
	uint8_t i;
	for (i = 0; i < 100; ++i) {
		uint8_t testTimestep = (currentTimestep + i) % 100;
		// If there's already messages at this timestep, add up their cost.
		if (timesteps[testTimestep]) {
			uint16_t currentCost = 0;
			SListItem *j;
			for	(j = timesteps[testTimestep]; j; j = j->sibling) {
				currentCost += messageSizes[j->id] + MAVLINK_NUM_NON_PAYLOAD_BYTES;
			}
			if (currentCost < lastCost) {
				bestTimestep = testTimestep;
				lastCost = currentCost;
			}
		}
		// But if this is an empty timestep it's optimal and it's chosen and the
		// search ends.
		else {
			bestTimestep = testTimestep;
			break;
		}
	}
	
	// Now that we have the best timestep to add this in, do it.
	SListItem *x = malloc(sizeof(SListItem));
	if (x == NULL) {
		return false;
	}
	
	x->_transient = true;
	x->id = id;
	x->sibling = timesteps[bestTimestep];
	timesteps[bestTimestep] = x;
	
	return true;
}

void RemoveMessage(uint8_t id)
{
	// Now we loop through all of the messages at this timestep. Not the iteration condition
	// for the for-loop. As j has already been freed we cannot safely use it again and so
	// must use lastGoodItem->sibling instead. We do make the assumption, however, that there
	// exists a single message ID per timestep.
	uint8_t i;
	for (i = 0; i < 100; i++) {
	
		// Handle the 0-length list case
		if (!timesteps[i]) {
			continue; 
		}
		// And then the case if this message is the first in the list. Note that this applies
		// to both 1-length and many-length lists but successfully short-circuits in the latter
		// case.
		else if (timesteps[i] && timesteps[i]->id == id) {
			SListItem *newFirstMessage = timesteps[i]->sibling;
			free(timesteps[i]);
			timesteps[i] = newFirstMessage;
		}
		// And finally the case where the message is not the first message.
		else {
			// Store the first message
			SListItem *lastGoodItem = timesteps[i];
			
			// And then loop through the rest removing our specific message and short-circuiting
			// if that's the case.
			SListItem *j;
			for	(j = lastGoodItem->sibling; j; j = j->sibling) {
				if (j->id == id) {
					lastGoodItem->sibling = j->sibling;
					free(j);
					continue;
				} else {
					lastGoodItem = j;
				}
			}
		}
	}
}

void ClearAllMessages(void)
{
	SListItem *tmp;
	uint8_t i;
	for (i = 0; i < 100; i++) {
		SListItem *j;
		for	(j = timesteps[i]; j; j = tmp) {
			tmp = j->sibling;
			free(j);
		}
		timesteps[i] = NULL;
	}
}

SListItem *IncrementTimestep(void)
{
	/// First clean up any non-recurring messages from the previous timestep.
	// Determine what the previous timestep was
	uint8_t cleanupTimestep;
	if (currentTimestep == 0) {
		cleanupTimestep = 99;
	} else {
		cleanupTimestep = currentTimestep - 1;
	}
	
	// And then iterate through the list finding all transient messages and removing
	// them.
	SListItem *previousItem = NULL;
	SListItem *j;
	for	(j = timesteps[cleanupTimestep]; j; j = j->sibling) {
		if (j->_transient) {
			// If we aren't at the start of the list, just cut this element out of the middle
			if (previousItem) {
				previousItem->sibling = j->sibling;
			}
			// Otherwise just move where the start of the list is.
			else {
				timesteps[cleanupTimestep] = j->sibling;
			}
			// And then clean up the transient messages
			free(j);
		} else {
			previousItem = j;
		}
	}

	// Then log the data from the current timestep
	uint8_t tmp = currentTimestep;

	// And then increment the timestep
	if (currentTimestep == 99) {
		currentTimestep = 0;
	} else {
		++currentTimestep;
	}

	// Finally return the logged data for this current timestep
	return timesteps[tmp];
}

void ResetCurrentTimestep(void)
{
	currentTimestep = 0;
}

#ifdef UNIT_TEST
#include <stdio.h>
#include <assert.h>

/**
 * This function prints out all of the messages for every timestep. Useful for debugging with
 * gdb when it can be called at any point manually via `call PrintAllTimesteps()`. Otherwise
 * this function isn't used by any code.
 */
void PrintAllTimesteps(void)
{
	puts("Timesteps:\n");
	uint8_t i;
	for (i = 0; i < 100; i++) {
		uint16_t timestepSize = 0;
		SListItem *j;
		for	(j = timesteps[i]; j; j = j->sibling) {
			timestepSize += messageSizes[j->id] + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}
		printf(" [%d](size=%d): ", i, timestepSize);
		for	(j = timesteps[i]; j; j = j->sibling) {
			printf("%d, ", j->id);
		}
		puts("\n");
	}
}

int main(void)
{
	// First perform a basic functionality test by inserting a single 100Hz message that should occupy all timesteps.
	{
		assert(AddMessage(33, 100));
		uint8_t i;
		for (i = 0; i < 100; i++) {
			SListItem *x = IncrementTimestep();
			assert(x); // Check that a message ended up here.
			assert(x->id == 33); // Check that it's the right message
			assert(!x->sibling); // Check that there's only one message at this timestep.
		}
		
		// Then remove that message and confirm that every timestep is clear.
		RemoveMessage(33);
		for (i = 0; i < 100; i++) {
			SListItem *x = IncrementTimestep();
			assert(!x);
		}
	}
	
	// Check that the range of rates accepted by AddMessage is correct.
	{
		assert(!AddMessage(11, 0));
		assert(!AddMessage(66, 101));
		
		// And check that messages weren't actually added also.
		uint8_t i;
		for (i = 0; i < 100; i++) {
			SListItem *x = IncrementTimestep();
			assert(!x);
		}
	}
	
	// Test resetting the current timestep.
	{
		// First add a single 55 message at just the 0-timestep.
		assert(AddMessage(55, 1));
		SListItem *x = IncrementTimestep();
		assert(x);
		assert(x->id == 55);
		assert(!x->sibling);
		uint8_t i;
		for (i = 1; i < 50; i++) {
			SListItem *x = IncrementTimestep();
			assert(!x); // Check that a message ended up here.
		}
		
		// Now attempt to reset the timestep since we're halfway through the timesteps
		ResetCurrentTimestep();
		
		// And check everything again 
		x = IncrementTimestep();
		assert(x);
		assert(x->id == 55);
		assert(!x->sibling);
		for (i = 1; i < 50; i++) {
			SListItem *x = IncrementTimestep();
			assert(!x); // Check that a message ended up here.
		}
		
		// And cleanup
		RemoveMessage(55);
		ResetCurrentTimestep();
	}
	
	// Now test handling of a bunch of different types of messages.
	{
		// Then include 100 1Hz messages and confirm that the different messages all end up by themselves in a single timestep.
		uint8_t i;
		for (i = 0; i < 100; i++) {
			assert(AddMessage(i, 1));
		}
		for (i = 0; i < 100; i++) {
			SListItem *x = IncrementTimestep();
			assert(!(x->sibling)); // Check that there's only one message
			assert(x->id == i); // Check that it's the correct message
		}
		
		// And then add another message and check that it was added appropriately.
		// We don't add a message that was already used as that violates our assumption
		// that only one message of any given id exists at any single timestep.
		assert(AddMessage(100, 1));
		
		// We confirm that this was correct by finding where our message ended up
		// and then check that this bucket was occupied by a 0-length message (and
		// so was the first of the smallest timesteps, which is what's chosen).
		for (i = 0; i < 100; i++) {
			SListItem *x = IncrementTimestep();
			if (x && x->id == 100) {
				assert(messageSizes[x->sibling->id] == 0);
				break;
			}
		}
		
		// Now clear the list and confirm that it's empty.
		ClearAllMessages();
		for (i = 0; i < 100; i++) {
			SListItem *x = IncrementTimestep();
			assert(!x);
		}
	}
	
	// Test that all acceptable rates are handled correctly.
	// NOTE: All tests until now used fairly safe transmission rates.
	{
		uint8_t i;
		for (i = 1; i < 100; i++) {
			assert(AddMessage(97, i));
			
			uint8_t counter = 0;
			uint8_t j;
			for (j = 0; j < 100; j++) {
				SListItem *x = IncrementTimestep();
				if (x && x->id == 97) {
					++counter;
				}
			}
			
			assert(counter == i);
			RemoveMessage(97);
			ResetCurrentTimestep();
		}
	}
	
	// Check transient message handling.
	{
		// First add a 100Hz message, change the current timestep, and then check that the message
		// is added to the next timestep.
		assert(AddMessage(111, 100));
		
		uint8_t i;
		for (i = 0; i < 23; i++) {
			IncrementTimestep();
		}
		
		assert(AddTransientMessage(143));
		
		SListItem *x = IncrementTimestep();
		assert(x);
		assert(x->id == 143);
		
		// Now that this transient message has been handled if we loop around again it should be gone.
		// We also should also not encounter this message until then.
		for (i = 0; i < 99; i++) {
			x = IncrementTimestep();
			assert(x);
			assert(x->id == 111);
			assert(!x->sibling);
		}
		
		// Back to the original timestep + 100 steps. We shouldn't see the transient message here again.
		x = IncrementTimestep();
		assert(x);
		assert(x->id == 111);
		assert(!x->sibling);
	}
	
	// Now attempt a realistic message scheduling scenario.
	// I don't actually do any automated checking here, but this can
	// be useful to confirm things by hand.
	{
		//ClearAllMessages();
		//assert(AddMessage(0, 1)); // Heartbeat at 1Hz
		//assert(AddMessage(1, 1)); // System status at 1Hz
		//assert(AddMessage(30, 10)); // Attitude at 10Hz
		//assert(AddMessage(32, 10)); // Local position at 10Hz
		//assert(AddMessage(74, 4)); // VFR_HUD at 4Hz
		//assert(AddMessage(24, 1)); // GPS at 1Hz
		//assert(AddMessage(171, 10)); // State data at 10Hz
		//assert(AddMessage(161, 2)); // DST800 data at 2Hz
		//assert(AddMessage(162, 2)); // Revo GS compass data at 2Hz
		//assert(AddMessage(170, 4)); // Status and errors at 4Hz
		//assert(AddMessage(160, 2)); // WSO100 data at 2Hz
		//assert(AddMessage(150, 4)); // RUDDER_RAW at 4Hz
		//PrintAllTimesteps();
	}
	
	// And display success!
	puts("All tests passed successfully.\n");
	return EXIT_SUCCESS;
}

#endif
