/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 */

// Include user headers
#include "MessageScheduler.h"

// Include system libraries
#include <stdlib.h>
#include <limits.h>
#include <math.h>

// These constants are used for indexing into `MessageSchedule.schedule->Timesteps` for
// dealing with either the transient or the repeating messages.
#define MSCHED_REPEATING 0
#define	MSCHED_TRANSIENT 1

bool AddMessageRepeating(MessageSchedule *schedule, uint8_t id, uint8_t rate)
{
	// Be sure that we only process messages are reasonable rates
	if (rate < 1 || rate > 100) {
		return false;
	}

	// Just store this for future use as it's used quite often.
	float period = 100.0/((float)rate);
	
	// Find out which internal message ID we should use. If one isn't found,
	// return an error.
	int mid = -1;
	int i;
	for (i = 0; i < schedule->MessageTypes; ++i) {
		if (schedule->MessageIds[i] == id) {
			mid = i;
			break;
		}
	}
	if (mid == -1) {
		return false;
	}

	//TODO: Add a verification step that this message can even be safely added.
	// This would require knowing the size of the new message and then comparing
	// the integral of all timestep bytes with MESSAGE_TRANSMISSION_RATE. If it's
	// over the top we should return a failure.
	
	// Now find a decent offset for this message to use for its transfer rate.
	// We first go through every possible offset and determine which one provides
	// the emptiest channel for transmission based on total bytes sent during that
	// transmission	window.
	
	// So we search through every offset and check the number of bytes transmit in a given offset.
	uint8_t bestOffset;
	uint16_t lastCost = USHRT_MAX;
	uint8_t offset;
	for (offset = 0; offset < (uint8_t)period; offset++) {
		uint16_t currentCost = 0;
		uint8_t timestep = offset;
		uint8_t i;
		for (i = 0; i < rate && timestep < 100; timestep += period, i++) {
			int currentTimestepWord = (timestep & 0x70) >> 4;
			uint16_t timestepIndex = 1 << (timestep & 0x0F);
			// Add up the cost of every message at this timestep
			uint16_t j;
			for	(j = 0; j < schedule->MessageTypes; ++j) {
				// We only count repeating messages as transient messages will disappear and
				// not be a factor over the long-term.
				if (schedule->Timesteps[j][MSCHED_REPEATING][currentTimestepWord] & timestepIndex) {
					currentCost += schedule->MessageSizes[j];
				}
			}
		}
		// If we've found a better offset, store it.
		if (currentCost < lastCost) {
			bestOffset = offset;
			lastCost = currentCost;
		}
	}
	
	// Finally add this message onto the list of messages to send using the
	// best offset that was found.
	uint8_t timestep = bestOffset;
	for (i = 0; i < rate && timestep < 100; timestep += period, i++) {
		int currentTimestepWord = (timestep & 0x70) >> 4;
		uint16_t timestepIndex = 1 << (timestep & 0x0F);
		schedule->Timesteps[mid][MSCHED_REPEATING][currentTimestepWord] |= timestepIndex;
	}

	return true;
}

// TODO: Only add this message between now and the next time this message is scheduled.
// There's no reason to transmit this one-off message if it comes after a regularly scheduled
// one. Really I should just add these as long as it's below the bandwidth limit for a single
// timestep.
bool AddMessageOnce(MessageSchedule *schedule, uint8_t id)
{
	// Find out which internal message ID we should use. If one isn't found,
	// return an error.
	int mid = -1;
	int i;
	for (i = 0; i < schedule->MessageTypes; ++i) {
		if (schedule->MessageIds[i] == id) {
			mid = i;
			break;
		}
	}
	if (mid == -1) {
		return false;
	}

	// Find the smallest bracket in the next second to transmit this message
	uint16_t lastCost = USHRT_MAX;
	uint8_t bestTimestep;
	for (i = 0; i < 100; ++i) {
		uint8_t testTimestep = (schedule->CurrentTimestep + i) % 100;
		int currentTimestepWord = (testTimestep & 0x70) >> 4;
		uint16_t timestepIndex = 1 << (testTimestep & 0x0F);
		// Add up the cost of every message at this timestep. This time we count transient
		// messages.
		uint16_t j;
		uint16_t currentCost = 0;
		for	(j = 0; j < schedule->MessageTypes; ++j) {
			if ((schedule->Timesteps[j][MSCHED_REPEATING][currentTimestepWord] | 
				schedule->Timesteps[j][MSCHED_TRANSIENT][currentTimestepWord]) & 
				timestepIndex) {
				currentCost += schedule->MessageSizes[j];
			}
		}
		
		// Now if this is the best timestep, choose this one. If the cost is > 0, we keep searching
		// for a lower-cost timestep.
		if (currentCost == 0) {
			bestTimestep = testTimestep;
			break;
		} else if (currentCost < lastCost) {
			bestTimestep = testTimestep;
			lastCost = currentCost;
		}
	}
	
	// Now that we have the best timestep to add this in, do it.
	int currentTimestepWord = (bestTimestep & 0x70) >> 4;
	uint16_t timestepIndex = 1 << (bestTimestep & 0x0F);
	schedule->Timesteps[mid][MSCHED_TRANSIENT][currentTimestepWord] |= timestepIndex;

	return true;
}

void RemoveMessage(MessageSchedule *schedule, uint8_t id)
{
	// Find out which internal message ID we should use. If one isn't found,
	// return an error.
	int mid = -1;
	int i;
	for (i = 0; i < schedule->MessageTypes; ++i) {
		if (schedule->MessageIds[i] == id) {
			mid = i;
			break;
		}
	}
	if (mid == -1) {
		return;
	}
	
	// Now clear that message from the schedule.
	schedule->Timesteps[mid][MSCHED_REPEATING][0] = 0;
	schedule->Timesteps[mid][MSCHED_REPEATING][1] = 0;
	schedule->Timesteps[mid][MSCHED_REPEATING][2] = 0;
	schedule->Timesteps[mid][MSCHED_REPEATING][3] = 0;
	schedule->Timesteps[mid][MSCHED_REPEATING][4] = 0;
	schedule->Timesteps[mid][MSCHED_REPEATING][5] = 0;
	schedule->Timesteps[mid][MSCHED_REPEATING][6] = 0;
	schedule->Timesteps[mid][MSCHED_REPEATING][7] = 0;
	schedule->Timesteps[mid][MSCHED_TRANSIENT][0] = 0;
	schedule->Timesteps[mid][MSCHED_TRANSIENT][1] = 0;
	schedule->Timesteps[mid][MSCHED_TRANSIENT][2] = 0;
	schedule->Timesteps[mid][MSCHED_TRANSIENT][3] = 0;
	schedule->Timesteps[mid][MSCHED_TRANSIENT][4] = 0;
	schedule->Timesteps[mid][MSCHED_TRANSIENT][5] = 0;
	schedule->Timesteps[mid][MSCHED_TRANSIENT][6] = 0;
	schedule->Timesteps[mid][MSCHED_TRANSIENT][7] = 0;
}

void ClearSchedule(MessageSchedule *schedule)
{
	// Remove all repeating and transient messages.
	uint8_t i;
	for (i = 0; i < schedule->MessageTypes; ++i) {
		schedule->Timesteps[i][MSCHED_REPEATING][0] = 0;
		schedule->Timesteps[i][MSCHED_REPEATING][1] = 0;
		schedule->Timesteps[i][MSCHED_REPEATING][2] = 0;
		schedule->Timesteps[i][MSCHED_REPEATING][3] = 0;
		schedule->Timesteps[i][MSCHED_REPEATING][4] = 0;
		schedule->Timesteps[i][MSCHED_REPEATING][5] = 0;
		schedule->Timesteps[i][MSCHED_REPEATING][6] = 0;
		schedule->Timesteps[i][MSCHED_REPEATING][7] = 0;
		schedule->Timesteps[i][MSCHED_TRANSIENT][0] = 0;
		schedule->Timesteps[i][MSCHED_TRANSIENT][1] = 0;
		schedule->Timesteps[i][MSCHED_TRANSIENT][2] = 0;
		schedule->Timesteps[i][MSCHED_TRANSIENT][3] = 0;
		schedule->Timesteps[i][MSCHED_TRANSIENT][4] = 0;
		schedule->Timesteps[i][MSCHED_TRANSIENT][5] = 0;
		schedule->Timesteps[i][MSCHED_TRANSIENT][6] = 0;
		schedule->Timesteps[i][MSCHED_TRANSIENT][7] = 0;
	}

	// And finally clear the current timestep. This concludes all state.
	schedule->CurrentTimestep = 0;
}

uint8_t GetMessagesForTimestep(MessageSchedule *schedule, uint8_t *messages)
{
	/// First clean up any non-recurring messages from the previous timestep.
	// Determine what the previous timestep was
	uint8_t cleanupTimestep;
	if (schedule->CurrentTimestep == 0) {
		cleanupTimestep = 99;
	} else {
		cleanupTimestep = schedule->CurrentTimestep - 1;
	}

	// Now clear all transients for the last timestep. A check isn't performed
	// as it's probably faster just to clear them all instead.
	int i;
	int currentTimestepWord = (cleanupTimestep & 0x70) >> 4;
	uint16_t timestepIndex = 1 << (cleanupTimestep & 0x0F);
	for	(i = 0; i < schedule->MessageTypes; ++i) {
		schedule->Timesteps[i][MSCHED_TRANSIENT][currentTimestepWord] &= ~timestepIndex;
	}

	// Finally return the logged data for this current timestep. We check both the repeating
	// and the transient timesteps and add a message if it occurs in either. This prevents
	// duplicate message transmission during a single timestep. This means the most messages that
	// can be transmit during a single timestep is 1 of every message, which seems a reasonable
	// limit.
	uint8_t messageCount = 0;
	currentTimestepWord = (schedule->CurrentTimestep & 0x70) >> 4;
	timestepIndex = 1 << (schedule->CurrentTimestep & 0x0F);
	for	(i = 0; i < schedule->MessageTypes; ++i) {
		if ((schedule->Timesteps[i][MSCHED_REPEATING][currentTimestepWord] | 
		    schedule->Timesteps[i][MSCHED_TRANSIENT][currentTimestepWord]) & 
			timestepIndex) {
			messages[messageCount++] = schedule->MessageIds[i];
		}
	}

	// And finally increment the timestep
	if (schedule->CurrentTimestep == 99) {
		schedule->CurrentTimestep = 0;
	} else {
		++schedule->CurrentTimestep;
	}

	return messageCount;
}

void ResetTimestep(MessageSchedule *schedule)
{
	schedule->CurrentTimestep = 0;
}

#ifdef UNIT_TEST
#include <stdio.h>
#include <assert.h>
#include <stdio.h>

/**
 * This function prints out all of the messages for every timestep. Useful for debugging with
 * gdb when it can be called at any point manually via `call PrintAllschedule->Timesteps()`. Otherwise
 * this function isn't used by any code.
 */
void PrintAllTimesteps(const MessageSchedule *schedule)
{
	puts("schedule->Timesteps:\n");
	uint8_t i;
	for (i = 0; i < schedule->MessageTypes; ++i) {
		printf("%3d: %04x %04x %04x %04x %04x %04x %04x %04x\n", schedule->MessageIds[i], schedule->Timesteps[i][MSCHED_REPEATING][0],
		                                                            schedule->Timesteps[i][MSCHED_REPEATING][1],
		                                                            schedule->Timesteps[i][MSCHED_REPEATING][2],
		                                                            schedule->Timesteps[i][MSCHED_REPEATING][3],
		                                                            schedule->Timesteps[i][MSCHED_REPEATING][4],
		                                                            schedule->Timesteps[i][MSCHED_REPEATING][5],
		                                                            schedule->Timesteps[i][MSCHED_REPEATING][6],
		                                                            schedule->Timesteps[i][MSCHED_REPEATING][7]);
	}
}

// Set up the necessary constants for the messages.
enum {
	MSG_ID_1 = 31,
	MSG_ID_2 = 0,
	MSG_ID_3 = 2,
	MSG_ID_4 = 78,
	MSG_ID_5 = 152
};
enum {
	MSG_ID_1_SIZE = 9,
	MSG_ID_2_SIZE = 31,
	MSG_ID_3_SIZE = 12,
	MSG_ID_4_SIZE = 14,
	MSG_ID_5_SIZE = 28
};

// Set up a schedule struct and supporting data structures.
#define NUM_MSGS 5
uint16_t tsteps[NUM_MSGS][2][8] = {};
uint8_t mIds[NUM_MSGS] = {MSG_ID_1, MSG_ID_2, MSG_ID_3, MSG_ID_4, MSG_ID_5};
uint8_t mSizes[NUM_MSGS] = {MSG_ID_1_SIZE, MSG_ID_2_SIZE, MSG_ID_3_SIZE, MSG_ID_4_SIZE, MSG_ID_5_SIZE};
MessageSchedule sched = {
	NUM_MSGS,
	mIds,
	mSizes,
	0,
	tsteps
};

int main(void)
{

	// First perform a basic functionality test by inserting a single 100Hz message that should occupy all timesteps.
	{
		assert(AddMessageRepeating(&sched, MSG_ID_1, 100));
		uint8_t i;
		for (i = 0; i < 100; i++) {
			uint8_t msgs[NUM_MSGS];
			uint8_t count = GetMessagesForTimestep(&sched, msgs);
			assert(count == 1);          // Check that only a single message ended up at this timestep
			assert(msgs[0] == MSG_ID_1); // Check that it's the right message
		}
		
		// Then remove that message and confirm that every timestep is clear.
		RemoveMessage(&sched, MSG_ID_1);
		for (i = 0; i < 100; i++) {
			uint8_t msgs[NUM_MSGS] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
			uint8_t count = GetMessagesForTimestep(&sched, msgs);
			assert(count == 0);      // Check that there are no messages at this timestep
			assert(msgs[0] == 0xFF); // Should still be the original data
		}
	}

	// Test that ClearSchedule() works.
	{
		assert(AddMessageRepeating(&sched, MSG_ID_1, 100));
		uint8_t i;
		for (i = 0; i < 100; i++) {
			uint8_t msgs[NUM_MSGS];
			uint8_t count = GetMessagesForTimestep(&sched, msgs);
			assert(count == 1);          // Check that only a single message ended up at this timestep
			assert(msgs[0] == MSG_ID_1); // Check that it's the right message
		}
		
		// Then remove that message and confirm that every timestep is clear.
		ClearSchedule(&sched);
		for (i = 0; i < 100; i++) {
			uint8_t msgs[NUM_MSGS] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
			uint8_t count = GetMessagesForTimestep(&sched, msgs);
			assert(count == 0);      // Check that there are no messages at this timestep
			assert(msgs[0] == 0xFF); // Should still be the original data
		}
	}

	// Check that the range of rates accepted by AddMessage is correct.
	{
		assert(!AddMessageRepeating(&sched, MSG_ID_2, 0));
		assert(!AddMessageRepeating(&sched, MSG_ID_4, 101));
		
		// And check that messages weren't actually added also.
		uint8_t i;
		for (i = 0; i < 100; i++) {
			uint8_t msgs[NUM_MSGS];
			uint8_t count = GetMessagesForTimestep(&sched, msgs);
			assert(!count);
		}
	}

	// Test resetting the current timestep.
	{
		// First add a single 55 message at just the 0-timestep.
		assert(AddMessageRepeating(&sched, MSG_ID_3, 1));
		uint8_t msgs[NUM_MSGS];
		uint8_t count = GetMessagesForTimestep(&sched, msgs);
		assert(count == 1);
		assert(msgs[0] == MSG_ID_3);
		uint8_t i;
		for (i = 1; i < 50; i++) {
			count = GetMessagesForTimestep(&sched, msgs);
			assert(!count); // Check that there aren't any more messages.
		}
		
		// Now attempt to reset the timestep since we're halfway through the timesteps
		ResetTimestep(&sched);
		
		// And check everything again 
		count = GetMessagesForTimestep(&sched, msgs);
		assert(count == 1);
		assert(msgs[0] == MSG_ID_3);
		for (i = 1; i < 50; i++) {
			count = GetMessagesForTimestep(&sched, msgs);
			assert(!count); // Check that there aren't any more messages.
		}
		
		// And cleanup
		ClearSchedule(&sched);
	}

	// Now test handling of a bunch of different types of messages.
	{
		uint16_t tsteps[101][2][8] = {};
		uint8_t mIds[101] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100};
		uint8_t mSizes[101] = {
			1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1};
		MessageSchedule sched = {
			101,
			mIds,
			mSizes,
			0,
			tsteps
		};
		// Then include 100 1Hz messages and confirm that the different messages all end up by themselves in a single timestep.
		uint8_t i;
		for (i = 0; i < 100; i++) {
			assert(AddMessageRepeating(&sched, i, 1));
		}
		uint8_t msgs[NUM_MSGS];
		uint8_t count;
		for (i = 0; i < 100; i++) {
			count = GetMessagesForTimestep(&sched, msgs);
			assert(count == 1); // Check that there's only one message
			assert(msgs[0] == i); // Check that it's the correct message
		}
		
		// And then add another message and check that it was added appropriately.
		// We don't add a message that was already used as that violates our assumption
		// that only one message of any given id exists at any single timestep.
		assert(AddMessageRepeating(&sched, 100, 1));
		
		// We confirm that this was correct by finding where our message ended up
		// and then check that this bucket was occupied by a 0-length message (and
		// so was the first of the smallest timesteps, which is what's chosen).
		count = GetMessagesForTimestep(&sched, msgs);
		assert(count == 2); // Check that there's only one message
		assert(msgs[0] == 100 || msgs[1] == 100);
		assert(msgs[0] == 0 || msgs[1] == 0);
		
		// Now clear the list and confirm that it's empty.
		ClearSchedule(&sched);
		for (i = 0; i < 100; i++) {
			count = GetMessagesForTimestep(&sched, msgs);
			assert(!count); // Check that there's only one message
		}
	}

	// Test that all acceptable rates are handled correctly.
	// NOTE: All tests until now used fairly safe transmission rates.
	{
		uint16_t tsteps[101][2][8] = {};
		uint8_t mIds[101] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100};
		uint8_t mSizes[101] = {
			1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1};
		MessageSchedule sched = {
			101,
			mIds,
			mSizes,
			0,
			tsteps
		};
		uint8_t i;
		for (i = 1; i < 100; i++) {
			assert(AddMessageRepeating(&sched, 97, i));
			
			uint8_t counter = 0;
			uint8_t j;
			for (j = 0; j < 100; j++) {
				uint8_t msgs[101];
				uint8_t count = GetMessagesForTimestep(&sched, msgs);
				for (;count;--count) {
					if (msgs[0] == 97) {
						++counter;
						break;
					}
				}
			}
			
			assert(counter == i);
			ClearSchedule(&sched);
		}
	}
	
	// Check transient message handling.
	{
		// Initialize our schedule.
		uint16_t tsteps[2][2][8] = {};
		uint8_t mIds[2] = {111, 143};
		uint8_t mSizes[2] = {1,1};
		MessageSchedule sched = {
			2,
			mIds,
			mSizes,
			0,
			tsteps
		};
	
		// First add a 100Hz message, change the current timestep, and then check that the message
		// is added to the next timestep.
		assert(AddMessageRepeating(&sched, 111, 100));
		
		uint8_t msgs[2];
		uint8_t i;
		for (i = 0; i < 23; i++) {
			GetMessagesForTimestep(&sched, msgs);
		}
		
		assert(AddMessageOnce(&sched, 143));
		
		uint8_t count = GetMessagesForTimestep(&sched, msgs);
		assert(count == 2);
		assert(msgs[1] = 143);
		
		// Now that this transient message has been handled if we loop around again it should be gone.
		// We also should also not encounter this message until then.
		for (i = 0; i < 99; i++) {
			count = GetMessagesForTimestep(&sched, msgs);
			assert(count == 1);
			assert(msgs[0] == 111);
		}
		
		// Back to the original timestep + 100 steps. We shouldn't see the transient message here again.
		count = GetMessagesForTimestep(&sched, msgs);
		assert(count == 1);
		assert(msgs[0] == 111);
	}

	// Now attempt a realistic message scheduling scenario.
	// I don't actually do any automated checking here, but this can
	// be useful to confirm things by hand.
	{
		// Initialize our schedule.
		uint16_t tsteps[12][2][8] = {};
		uint8_t mIds[12] = {0, 1, 30, 32, 74, 24, 171, 161, 162, 170, 160, 150};
		uint8_t mSizes[12] = {9, 31, 28, 28, 20, 30, 19, 22, 10, 4, 36, 7};
		MessageSchedule sched = {
			12,
			mIds,
			mSizes,
			0,
			tsteps
		};
		assert(AddMessageRepeating(&sched, 0, 1)); // Heartbeat at 1Hz
		assert(AddMessageRepeating(&sched, 1, 1)); // System status at 1Hz
		assert(AddMessageRepeating(&sched, 30, 10)); // Attitude at 10Hz
		assert(AddMessageRepeating(&sched, 32, 10)); // Local position at 10Hz
		assert(AddMessageRepeating(&sched, 74, 4)); // VFR_HUD at 4Hz
		assert(AddMessageRepeating(&sched, 24, 1)); // GPS at 1Hz
		assert(AddMessageRepeating(&sched, 171, 10)); // State data at 10Hz
		assert(AddMessageRepeating(&sched, 161, 2)); // DST800 data at 2Hz
		assert(AddMessageRepeating(&sched, 162, 2)); // Revo GS compass data at 2Hz
		assert(AddMessageRepeating(&sched, 170, 4)); // Status and errors at 4Hz
		assert(AddMessageRepeating(&sched, 160, 2)); // WSO100 data at 2Hz
		assert(AddMessageRepeating(&sched, 150, 4)); // RUDDER_RAW at 4Hz
		puts("The scheduling for a realistic message transmission scenario.");
		PrintAllTimesteps(&sched);
	}
	
	// And display success!
	puts("\nAll tests passed successfully.");
	return EXIT_SUCCESS;
}

#endif
