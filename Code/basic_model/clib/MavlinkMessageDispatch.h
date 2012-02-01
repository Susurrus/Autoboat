/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * DESCRIPTION:
 * This library contains a message dispatcher for use with MAVLink. It allows for repeated and
 * one-off messages to be smartly allocated during all the possible timesteps for transmission.
 * This reduces bursting and minimizes problems with buffer overflow.
 *
 * REQUIREMENTS:
 * This library requires Sealion's MAVLink to be included so that "sealion/mavlink.h" is
 * accessible.
 *
 * USAGE:
 * 1) Set your code to call IncrementTimestep() at 100Hz and to process the list of message IDs.
 * 2) Add an initialization function to add all desired repeating messages using AddMessage().
 * 3) Add support for a heap (probably at least 512). This library relies on malloc() and free(). 
 * 4) That's it! If you'd like to change the dispatched messages you may at any time.
 *
 * TESTING:
 * A unit-testing framework is built-in to this library and available by running with the UNIT_TEST
 * preprocessor macro defined. For example: 
 *   `gcc MavlinkMessageDispatch.c -DUNIT_TEST -g -lm`
 */

#include <inttypes.h>
#include <stdbool.h>

/**
 * This is a singly-linked list implementation specific to this library.
 */
typedef struct SListItem {
	// ID of the MAVLink message associated with this timestep
	uint8_t id;
	// Boolean indicating whether this is a transient or non-recurring message.
	// NOTE: Should not be used by external code.
	bool _transient;
	// A pointer to the next element in the list. NULL if there isn't one.
	struct SListItem *sibling;
} SListItem;

/**
 * Adds the specified message at the given rate (in Hz, from 1 to 100) to the dispatcher.
 */
bool AddMessage(uint8_t id, uint8_t rate);

/**
 * Adds a one-time message to the dispatcher. Note that sequential calls to this function may not
 * persist that ordering within the dispatcher as messages are placed in the lowest cost bin first.
 */
bool AddTransientMessage(uint8_t id);

/**
 * Removes all messages of a given ID from the dispatcher. This will apply to transient AND
 * repeating messages.
 */
void RemoveMessage(uint8_t id);

/**
 * Clears all messages currently handled by the dispatcher.
 */
void ClearAllMessages(void);

/**
 * This is the actual dispatching function that will retrieve the list of messages that are
 * scheduled for the current timestep. It also increments the timestep so that repeated calls
 * fetch sequential timesteps.
 */
SListItem *IncrementTimestep(void);

/**
 * Reset the current timestep. This has no other affect on the dispatcher.
 */
void ResetTimestep(void);
