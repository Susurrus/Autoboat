This project exists for testing the MAVLink corruption issue I have onboard the boat. It creates a minimal environment for communication Missions to QGC. The corruption bug most commonly occurs during this process, possibly as a conflict between rx/tx buffers or interrupts.

At this point I have reduced the code down to just outputting a variety of MAVLink messages along with implementing the Mission protocol (removed the parameter one). Corruption occurs often enough that it's still an issue, but easily reproducible by refreshing the Mission list in QGC.

Next steps:
 * Remove more code to drop the code size to be the small code memory model.
