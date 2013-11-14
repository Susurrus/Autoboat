/**
 * Initialize all IMU-node-specific functionality.
 * @param f_osc The oscillator frequency that the processor is operating at.
 */
void ImuNodeInit(uint32_t f_osc);

/**
 * This function contains all calls that should be called continuously on the IMU node.
 */
void RunContinuousTasks(void);

/**
 * This function does all the work that should be done at .01s intervals.
 */
void Run100HzTasks(void);
