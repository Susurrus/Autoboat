#ifndef PRIMARY_NODE_H
#define PRIMARY_NODE_H

/**
 * This enum declares the bitflags used for the nodeStatus variable in Node.h.
 */
enum PRIMARY_NODE_STATUS {
	PRIMARY_NODE_STATUS_AUTOMODE    = 0x0001, // Vehicle is in autonomous mode, versus manual control mode.
	PRIMARY_NODE_STATUS_UNUSED      = 0x0002,
	PRIMARY_NODE_STATUS_ECAN_TX_ERR = 0x0004, // Error in CAN transmission
	PRIMARY_NODE_STATUS_ECAN_RX_ERR = 0x0008, // ERROR in CAN reception
	PRIMARY_NODE_STATUS_GPS_DISCON  = 0x0010  // GPS is disconnected/not locked on.
};

/**
 * This enum declares the bitflags used for the nodeErrors variable in Node.h.
 */
enum PRIMARY_NODE_RESET {
	PRIMARY_NODE_RESET_STARTUP          = 0x0001, // Active during the first 5 seconds of bootup.
	PRIMARY_NODE_RESET_UNUSED1          = 0x0002,
	PRIMARY_NODE_RESET_UNUSED2          = 0x0004,
	PRIMARY_NODE_RESET_UNUSED3          = 0x0008,
	PRIMARY_NODE_RESET_MANUAL_OVERRIDE  = 0x0010, // Manual override has been engaged by the secondary controller
	PRIMARY_NODE_RESET_CALIBRATING      = 0x0020, // The rudder is undergoing calibration.
	PRIMARY_NODE_RESET_UNCALIBRATED     = 0x0040, // The rudder is uncalibrated.
	PRIMARY_NODE_RESET_ESTOP            = 0x0080  // The system is in emergency-stop mode, actuators are centered and stopped, system will not respond to any commands; it's dead in the water.
};

// Calculate the BRG register value necessary for 115200 baud with a 80MHz clock.
#define BAUD115200_BRG_REG 21

/**
 * Initialize all of the C-libraries necessary for the Primary Node.
 */
void PrimaryNodeInit(void);

#endif // PRIMARY_NODE_H