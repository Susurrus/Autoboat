#ifndef HIL_NODE_H
#define HIL_NODE_H

/**
 * Declare flags for use with checking the `nodeStatus` variable declared in `Node.h`
 */
enum HIL_STATUS_FLAGS {
	// If the node is currently receiving UDP telemetry from the PC.
	NODE_STATUS_FLAG_HIL_ACTIVE    = 0x0001,
	// Set when the rudder subsystem is actively transmitting. This also indicates when sensor
	// feedback mode is engaged.
	NODE_STATUS_FLAG_RUDDER_ACTIVE = 0x0002,
	// This bit will be set if the ACS300 board is actively transmitting.
	NODE_STATUS_FLAG_PROP_ACTIVE   = 0x0004,
	// This bit will be set if the RC board is actively transmitting.
	NODE_STATUS_FLAG_RC_ACTIVE     = 0x0008
};

/**
 * Define a different FATAL_ERROR() macro that's specific to the HIL node. This one disables both
 * timers before entering the FATAL_ERROR loop.
 */
#define HIL_FATAL_ERROR() do { \
    T2CONbits.TON = 0;         \
    T4CONbits.TON = 0;         \
    _TRISA3 = 0;               \
    _LATA3 = 1;                \
    while (1);                 \
} while (0)

/**
 * A resettable ECAN-triggered error, likely from not having a receiving node on the CAN bus and the
 * ECAN hardware triggering an error bit. This macro will block until there are no more ECAN
 * transmission errors.
 */
#define HIL_ECAN_TRY(x) do {                \
    EcanStatus s;                           \
    do {                                    \
        x;                                  \
        s = Ecan1GetErrorStatus();          \
    } while (s.TxError != ECAN_ERROR_NONE); \
} while (0)

/**
 * Initialization function, configures everything the node needs.
 */
void HilNodeInit(void);

/**
 * Callback for Timer4 triggering every 250ms
 */
void HilNodeBlink(void);

/**
 * Callback for Timer2 triggering every 10ms.
 */
void HilNodeTimer100Hz(void);

/**
 * Process any received CAN messages coming through ECAN1. Store incoming
 * data into the hilDataToTransmit struct.
 */
uint8_t CanReceiveMessages(void);

#endif // HIL_NODE_H
