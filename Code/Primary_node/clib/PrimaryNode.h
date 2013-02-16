#ifndef PRIMARY_NODE_H
#define PRIMARY_NODE_H

/**
 * This enum declares the bitflags used for the nodeStatus variable in Node.h.
 */
enum PRIMARY_NODE_STATUS {
	PRIMARY_NODE_STATUS_AUTOMODE    = 0x0001,
	PRIMARY_NODE_STATUS_UNUSED      = 0x0002,
	PRIMARY_NODE_STATUS_ECAN_TX_ERR = 0x0004,
	PRIMARY_NODE_STATUS_ECAN_RX_ERR = 0x0008
};

/**
 * This enum declares the bitflags used for the nodeErrors variable in Node.h.
 */
enum PRIMARY_NODE_RESET {
	PRIMARY_NODE_RESET_STARTUP          = 0x0001,
	PRIMARY_NODE_RESET_UNUSED1          = 0x0002,
	PRIMARY_NODE_RESET_UNUSED2          = 0x0004,
	PRIMARY_NODE_RESET_GPS_DISCON       = 0x0008,
	PRIMARY_NODE_RESET_MANUAL_OVERRIDE  = 0x0010,
	PRIMARY_NODE_RESET_CALIBRATING      = 0x0020,
	PRIMARY_NODE_RESET_UNCALIBRATED     = 0x0040,
	PRIMARY_NODE_RESET_ESTOP            = 0x0080
};

// Calculate the BRG register value necessary for 115200 baud with a 80MHz clock.
#define BAUD115200_BRG_REG 21

/**
 * Initialize all of the C-libraries necessary for the Pimary Node.
 */
void PrimaryNodeInit(void);

#endif // PRIMARY_NODE_H