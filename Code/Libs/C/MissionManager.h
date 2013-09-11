#ifndef MISSION_MANAGER_H
#define MISSION_MANAGER_H

#include "Types.h"

// Specify how many missions this manager can handle.
#ifndef MISSION_MANAGER_MAX_MISSIONS
#define MISSION_MANAGER_MAX_MISSIONS 16
#endif

// Use this struct to track missions
typedef struct {
  float coordinates[3];
  float otherCoordinates[3];
  uint8_t refFrame;
  uint8_t action;
  float parameters[4];
  bool autocontinue;
} Mission;

void MissionInit(void);

void MissionSetCurrentIndex(uint8_t index);
int16_t MissionGetCurrentIndex(void);

bool MissionGet(uint8_t index, Mission *m);
void MissionAppend(const Mission *m);

void MissionSetStart(const Mission *start);
void MissionGetStart(Mission *start);

uint8_t GetMissionCount(void);
void MissionClearList(void);

#endif // MISSION_MANAGER_H
