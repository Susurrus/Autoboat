/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * DESCRIPTION:
 * This library contains a message scheduler designed for transmission of both repeating messages
 * and one off messages. It attempts to schedule new messages over timesteps with the lowest amount
 * of data necessary for transmission. This library is timestep agnostic, but only supports 100
 * timesteps at this point.
 *
 * REQUIREMENTS:
 * This library has no prerequisites outside of the C standard library.
 *
 * USAGE:
 * 1) Set your code to call `IncrementTimestep()` at 100Hz and to process the list of message IDs.
 * 2) Add an initialization function to add all desired repeating messages using AddMessage*().
 * 3) Add support for a heap (probably at least 512). This library relies on malloc() and free(). 
 * 4) That's it! If you'd like to change the dispatched messages you may at any time.
 *
 * TESTING:
 * A unit-testing framework is built-in to this library and available by running with the UNIT_TEST
 * preprocessor macro defined. For example: 
 *   `gcc MessageScheduler.c -DUNIT_TEST -g -lm`
 */
#ifndef _MESSAGE_SCHEDULER_H_
#define _MESSAGE_SCHEDULER_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * This struct stores all of the state information necessary for the message schedular to operate.
 * Note that the IDs used for messages must be sequential and start at 0!
 */
typedef struct {
	// Number of messages. Equal the the first dimensions of the *Messages[] arrays.
	const uint8_t MessageTypes;
	// An ID for every message type.
	uint8_t *MessageIds;
	// The size in bytes of every message type. Contains `MessageTypes` entries.
	uint8_t *MessageSizes;
	// Tracks the current timestep that we're executing at.
	uint8_t CurrentTimestep;
	// Keep a 128-bit schedule for every message. First dimension is messagetype,
	// second is repeating or transient messages, and the last is the actual bitfield
	// storing the boolean values for each timestep.
	// We use 16-bit integers here because this is likely running on a 16-bit MCU and
	// this will be substantially faster than doing 32-bit operations.
	uint16_t (*Timesteps)[2][8];
} MessageSchedule;

/// These functions handle adding/removing messages from the schedule.

/**
 * Adds the specified message at the given rate (in Hz, from 1 to 100) to the dispatcher.
 */
bool AddMessageRepeating(MessageSchedule *schedule, uint8_t id, uint8_t rate);

/**
 * Adds a one-time message to the dispatcher. Note that sequential calls to this function may not
 * persist that ordering within the dispatcher as messages are placed in the lowest cost bin first.
 */
bool AddMessageOnce(MessageSchedule *schedule, uint8_t id);

/**
 * Removes all messages of a given ID from the dispatcher. This will apply to transient AND
 * repeating messages.
 */
void RemoveMessage(MessageSchedule *schedule, uint8_t id);

/**
 * Clears all messages currently handled by the dispatcher.
 */
void ClearSchedule(MessageSchedule *schedule);

/// These functions deal with the timesteps within a given schedule.

/**
 * This is the actual dispatching function. It is called to determine the messages that should be
 * transmit. It returns the ID of the messages scheduled for this timestep into `messages`. This
 * array should therefore be at least `MessageTypes` long.
 * @return The number of messages at this specific timestep.
 */
uint8_t GetMessagesForTimestep(MessageSchedule *schedule, uint8_t messages[]);

/**
 * Reset the current timestep. This has no other affect on the dispatcher.
 */
void ResetTimestep(MessageSchedule *schedule);

#endif // _MESSAGE_SCHEDULER_H_
