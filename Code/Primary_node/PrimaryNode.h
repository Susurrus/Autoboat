#ifndef PRIMARY_NODE_H
#define PRIMARY_NODE_H

/**
 * This enum declares the bitflags used for the nodeStatus variable in Node.h.
 */
enum PRIMARY_NODE_STATUS {
	PRIMARY_NODE_STATUS_AUTOMODE             = 0x0001, // Vehicle is in autonomous mode, versus manual control mode.
	PRIMARY_NODE_STATUS_RTB                  = 0x0002, // System is in return-to-base mode
	PRIMARY_NODE_STATUS_ECAN_TX_ERR          = 0x0004, // Error in CAN transmission
	PRIMARY_NODE_STATUS_ECAN_RX_ERR          = 0x0008, // ERROR in CAN reception
	PRIMARY_NODE_STATUS_GPS_INVALID          = 0x0010, // The GPS is no longer giving good readings
        PRIMARY_NODE_STATUS_RC_NODE_DISCONNECTED = 0x0020  // The RC node is missing from the CAN bus.
};

/**
 * This enum declares the bitflags used for the nodeErrors variable in Node.h.
 */
enum PRIMARY_NODE_RESET {
	PRIMARY_NODE_RESET_STARTUP          = 0x0001, // Active during the first 5 seconds of bootup.
	PRIMARY_NODE_RESET_GCS_DISCONNECTED = 0x0002, // The groundcontrol station has been disconnected for >= 30s.
	PRIMARY_NODE_RESET_GPS_DISCONNECTED = 0x0004, // The GPS unit has had a bad fix for >= 10s.
	PRIMARY_NODE_RESET_UNUSED           = 0x0008,
	PRIMARY_NODE_RESET_MANUAL_OVERRIDE  = 0x0010, // Manual override has been engaged by the secondary controller.
	PRIMARY_NODE_RESET_CALIBRATING      = 0x0020, // The rudder is undergoing calibration.
	PRIMARY_NODE_RESET_UNCALIBRATED     = 0x0040, // The rudder is uncalibrated.
	PRIMARY_NODE_RESET_ESTOP_OR_ACS300_DISCON = 0x0080, // The system is in emergency-stop mode, actuators are centered and stopped, system will not respond to any commands; it's dead in the water. This can also occur if the CAN connection to the ACS300 is lost/unavailable.
};

// Define what reset modes will trigger the return-to-base functionality. In our case it only makes
// sense to trigger RTB when the groundstation or GPS have been disconnected too long, or when the
// e-stop is pulled.
#define RTB_RESET_MASK (PRIMARY_NODE_RESET_GCS_DISCONNECTED | \
                        PRIMARY_NODE_RESET_GPS_DISCONNECTED | \
                        PRIMARY_NODE_RESET_ESTOP_OR_ACS300_DISCON)

// Calculate the BRG register value necessary for 115200 baud with a 80MHz clock.
#define BAUD115200_BRG_REG 21

typedef struct {
	float primaryManualRudderCommand;
	float secondaryManualRudderCommand;
	float autonomousRudderCommand;
	int16_t primaryManualThrottleCommand;
	int16_t secondaryManualThrottleCommand;
	int16_t autonomousThrottleCommand;
} ActuatorCommands;
extern ActuatorCommands currentCommands;

typedef enum {
    PRIMARY_MODE_MANUAL,
    PRIMARY_MODE_AUTONOMOUS
} PrimaryNodeMode;

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
 * Provides a helper function for updating the autonomous mode of the vehicle. Additional actions
 * are done by a MavlinkGlue helper function.
 * @param newMode True to put the vehicle into autonomous mode, False otherwise.
 */
void SetAutoMode(PrimaryNodeMode newMode);

/**
 * Provides a helper function for retrieving the automode boolean value from the nodeStatus bitfield.
 * I'm trying to minimize direct accesses/manipulation of the nodeStatus variable.
 * @returns A boolean of whether the vehicle is in autonomous mode or not.
 */
PrimaryNodeMode GetAutoMode(void);

/**
 * This macro provides a shortcut for checking if the vehicle is current in under autonomous control.
 */
#define IS_AUTONOMOUS() (GetAutoMode() == PRIMARY_MODE_AUTONOMOUS)

/**
 * Call at 100Hz to transmit a NODE_STATUS can message at 2Hz.
 */
void TransmitNodeStatus2Hz(void);

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
 * Check if the current mission has changed and announce over MAVLink if it has.
 */
void CheckMissionStatus(void);

/**
 * Initialize ADC system for detecting power usage.
 */
void PrimaryNodeAdcInit(void);

/**
 * Returns the sensed power rail voltage. Accuracy should be about 1%.
 * @return The measured power rail voltage in volts.
 */
float GetPowerRailVoltage(void);

/**
 * Returns the current draw on the attached power rain. Accuracy should be about 1%.
 * @return The measured power rail current in Amperes.
 */
float GetPowerRailCurrent(void);

/**
 * Returns the actual values driving the actuators. This is a 4-way mux between all methods of control
 * (primary manual, secondary manual, and autonomous) and no-control output.
 * @param rudderAngle[out] The rudder angle (rads)
 * @param throttle[out] The throttle setting (1/1023 * 100%)
 */
void GetCurrentActuatorCommands(float *rudderAngle, int16_t *throttle);

#endif // PRIMARY_NODE_H
