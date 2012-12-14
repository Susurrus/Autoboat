// This file contains the constant definitions as well as the required
// data structures to work with on the ECAN module
#ifndef ECAN_DEFINES_H
#define ECAN_DEFINES_H

#include "stdint.h"

// Message Types either a data message or a remote transmit request
enum can_msg_type {
	CAN_MSG_DATA = 0,
	CAN_MSG_RTR
};

// CAN frame type: either extended (29-bit header) or standard (11-bit header)
enum can_frame_type {
	CAN_FRAME_EXT = 0,
	CAN_FRAME_STD
};

// Data structures
typedef struct {
	uint32_t id;           // The 11-bit or 29-bit message ID
	uint8_t  buffer;       // An internal-use variable referring to buffer this message was received into/sent from.
	uint8_t  message_type; // The message type. See can_msg_type.
	uint8_t  frame_type;   // The frame type. See can_frame_type.
	uint8_t  payload[8];   // The message payload. Stores between 0 and 8 bytes of data.
	uint8_t  validBytes;   // Indicates how many bytes are valid within payload.
} CanMessage;

typedef union {
	CanMessage message;
	uint8_t     bytes[sizeof(CanMessage)];
} CanUnion;

#endif /* ECAN_DEFINES_H */
