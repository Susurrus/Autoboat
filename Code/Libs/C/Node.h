#ifndef CAN_NODE_H
#define CAN_NODE_H

/**
 * This library provides several interfaces for working with the CAN
 * node hardware developed by the ASL. This includes a standard status
 * message and backend data stores.
 *
 * Expected usage includes for a new node (eg. hil_node) to create a
 * .c file (HilNode.c) and to create an init function (HilNodeInit(void))
 * and to populate the nodeId variable with a value from the CAN_NODE_ID
 * enum. This function should also do any necessary non-PIC-peripheral
 * configuration. Additionally its header file (HilNode.h) should fully 
 * define the bits in the status and errors global variables defined in
 * this library.
 */

#include <stdint.h>
#include <xc.h>

/**
 * This enum declares the IDs for every node that is in this
 * system. Stored in the variable nodeId.
 * @see nodeId
 */
enum CAN_NODE_ID {
    CAN_NODE_PRIMARY_CONTROLLER = 1,
    CAN_NODE_RUDDER_CONTROLLER  = 2,
    CAN_NODE_RC                 = 3,
    CAN_NODE_POWER_SENSOR       = 4,
    CAN_NODE_HIL                = 5,
    CAN_NODE_IMU_SENSOR         = 6,
    CAN_NODE_GYRO_SENSOR        = 7,
    CAN_NODE_ATTITUDE_SENSOR    = 8
};

// Specify how many individual nodes there are:
#define NUM_NODES 8

// Specify desired CAN baud rate. All nodes must communicate at this to sit on
// the shared bus.
#define NODE_CAN_BAUD 250000

/**
 * This macro provides a way to handle fatal errors on the CAN node, where a red error LED is
 * available. This macro turns that LED on then sits and spins in a forever-loop.
 */
#define FATAL_ERROR() _TRISA3=0;_LATA3=1;while(1)

/**
 * This bitfield stores the various status bits for each CAN node.
 * Details on what each bit means can be found in the main header
 * files for each node.
 */
extern uint16_t nodeStatus;

/**
 * This bitfield stores the various error bits for each CAN node.
 * Any errors generally means a shutdown of the node or at least it
 * not performing its regular actions.
 * Details on what each bit means can be found in the main header
 * files for each node.
 */
extern uint16_t nodeErrors;

/**
 * This variable stores the onboard system time in units of 0.01s.
 * It is managed by external code and merely declared in this library.
 */
extern uint32_t nodeSystemTime;

/**
 * The ID for the given node.
 * @see CAN_NODE_ID enum.
 */
extern uint8_t nodeId;

/**
 * The CPU load for a given node. Units are in percent, so valid values are from 0 to 100. UINT8_MAX
 * indicates that this parameter is invalid/unmeasured.
 */
extern uint8_t nodeCpuLoad;

/**
 * The temperature from the onboard temp sensor for a given node. Units are in degrees Celsius,
 * with INT8_MAX being invalid/unmeasured.
 */
extern int8_t nodeTemp;

/**
 * The raw voltage input into the onboard 5V regulator. Units are 0.1V with UINT8_MAX meaning
 * invalid/unmeasured.
 */
extern uint8_t nodeVoltage;

/**
 * Transmit a CAN_MESSAGE_STATUS message using the status and error
 * variables declared in this library.
 */
void NodeTransmitStatus(void);

#endif // CAN_NODE_H
