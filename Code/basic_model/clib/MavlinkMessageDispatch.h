#include <inttypes.h>

// A unit-testing framework is built-in to this library and available by running with the UNIT_TEST preprocessor macro defined. For example: `gcc MavlinkMessageDispatch.c -std=c99 -DUNIT_TEST -g -lm`

typedef struct SListItem {
	uint8_t id;
	struct SListItem *sibling;
} SListItem;

uint8_t AddMessage(uint8_t id, uint8_t rate);

uint8_t AddOneTimeMessage(uint8_t id);

void RemoveMessage(uint8_t id);

void ClearAllMessages(void);

SListItem *IncrementTimestep(void);

void ResetTimestep(void);
