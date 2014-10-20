#ifndef ATTITUDE_NODE_H
#define ATTITUDE_NODE_H

/**
 * Declare flags for use with checking the `nodeStatus` variable declared in `Node.h`
 */
typedef enum {
    // If the node is currently receiving IMU telemetry.
    ATTITUDE_NODE_STATUS_FLAG_IMU_ACTIVE = 0x0001
} AttitudeNodeSTatus;

/**
 * Initialize all IMU-node-specific functionality.
 * @param f_osc The oscillator frequency that the processor is operating at.
 */
void AttitudeNodeInit(uint32_t f_osc);

/**
 * This function contains all calls that should be called continuously on the IMU node.
 */
void RunContinuousTasks(void);

/**
 * This function does all the work that should be done at .01s intervals.
 */
void Run100HzTasks(void);

#endif // ATTITUDE_NODE_H
