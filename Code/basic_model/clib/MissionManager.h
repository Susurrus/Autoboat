#ifndef __MISSION_MANAGER_H__
#define __MISSION_MANAGER_H__

/*
 * This file defines a mission manager for use with an autopilot. This code is organized to
 * follow the mission definitions set forth in MAVLink version 1.0 and follows similar
 * conventions.
 *
 * This code was designed for embedded systems and so no dynamic allocation occurs within it. This
 * does, however, limit the number of possible missions that can be stored to MAX_MISSIONS.
 *
 * This code was designed to be platform/autopilot independent and provide a complete and self-contained
 * mission management solution. It therefore also contains its own test code. Just set the macro 
 * UNIT_TEST and compile just MissionManager.c and run. Note that it relies on stdio.h in this mode.
 * Ex: gcc MissionManager.c -D UNIT_TEST -std=c99
 */

#include <stdint.h>
#include <stdbool.h>

// Define some boolean values for us if necessary
#ifndef __bool_true_and_false_are_defined
#warning Boolean types not enabled. You should recompile with c99 enabled.
#define bool uint8_t
#define true 1
#define false 0
#endif

// Put an upper bound on the number of missions/waypoints available.
// Helps when statically-allocating memory
#define MAX_MISSIONS 15

// Mission struct. Each one of these fully defines a waypoint.
typedef struct {
	float coord1; // For local missions: north; global missions: latitude
	float coord2; // For local missions: east; global missions: longitude
	float coord3; // For local missions: down; global missions: altitude
	uint8_t refFrame; // The reference frame of the mission. See MAV_FRAME in MAVLink:mavlink_types.h
	uint8_t action; // The required action for this mission. See MAV_CMD in MAVLink:common.xml
	float param1; // For NAV missions, the radius in which the mission is accepted as reached, in meters.
	float param2; // For NAV missions, time that the vessel should stay inside the PARAM1 radius before advancing, in milliseconds.
	float param3; // For LOITER missions, radius of the orbit about the mission point in meter. If positive the orbit should be clockwise, negative indicated counter-clockwise.
	float param4; // For NAV and LOITER missions this is the yaw orientation in degrees eastward from north.
	uint8_t autocontinue; // Boolean for whether upon reaching this waypoint the vessel should switch to the next one.
} Mission;

/// Mission list management functions.

// Update a single mission within the list.
bool UpdateMission(uint8_t index, Mission *m);

/**
 * Appends a mission to the list. If the list is full,
 * a -1 will be returned. Otherwise this function returns
 * the new mission list length.
 */
int8_t AppendMission(Mission *m);

// Clear all missions. Follow standard error reporting where true is success and false is failure.
bool ClearMissionList(void);

// Retried a mission via the specified index. Fills the provided Mission struct with the information.
// A boolean false is returned if the index is invalid given the current list, otherwise true is returned.
bool GetMission(uint8_t index, Mission *m);

// Returns the number of missions currently stored.
uint8_t GetMissionCount(void);

// Returns the index of the currently active mission. If there is no current mission, then the return value will be 255 (0xFF).
uint8_t GetCurrentMission(void); 

// Sets the current mission to the specified mission by its index. If an invalid index is specified, false is returned.
bool SetCurrentMission(uint8_t index);

#endif // __MISSION_MANAGER_H__
