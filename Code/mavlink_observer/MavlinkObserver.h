#ifndef MAVLINK_OBSERVER_H
#define MAVLINK_OBSERVER_H

/**
 * Initialization function, configures everything the node needs.
 */
void MavlinkObserverInit(void);

/**
 * Callback for Timer2 triggering every 1s.
 */
void MavlinkObserverTimer1Hz(void);

#endif // MAVLINK_OBSERVER_H
