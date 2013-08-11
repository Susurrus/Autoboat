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
	PRIMARY_NODE_STATUS_GPS_DISCON  = 0x0010  // GPS is disconnected or not locked on.
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

// Large data store of many internal/misc variables that are output via MAVLink.
extern MavlinkData internalVariables;

/**
 * Initialize all of the C-libraries necessary for the Primary Node.
 */
void PrimaryNodeInit(void);

/**
 * Clear the GPS and Rudder internal data structures when the system goes into reset mode. This is
 * useful primarily when testing the primary controller and the node is left on through multiple
 * test runs.
 */
void ClearStateWhenErrors(void);

/**
 * Set the primary status indicator LED to always blink
 */
void SetStatusModeLed(void);

/**
 * Change the status of the reset mode LED depending on system state.
 */
void SetResetModeLed(void);

/**
 * Change the status of the autonomous mode LED depending on system state.
 */
void SetAutoModeLed(void);

/**
 * Perform a bunch of processing on a manual rudder angle input including binning the final value.
 * @param rc A rudder angle in radians.
 */
float ProcessManualRudderCommand(float rc);

/**
 * Perform a bunch of processing on a manual throttle input including binning the final value.
 * @param tc The commanded throttle in units from -1000 (full reverse) to 1000 (full forward)
 */
int16_t ProcessManualThrottleCommand(int16_t tc);

/**
 * Initialize ADC system for detecting power usage.
 */
void PrimaryNodeAdcInit(void);

#endif // PRIMARY_NODE_H