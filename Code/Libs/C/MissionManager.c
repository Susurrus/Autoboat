#include "MissionManager.h"

// Global data store for all stored missions
typedef struct {
  uint8_t currentIndex;
  uint8_t size;
  bool updated;
  uint8_t maxSize;
  Mission startingPoint;
  Mission missions[MISSION_MANAGER_MAX_MISSIONS];
} MissionList;
static MissionList mList;

void MissionAppend(const Mission *m)
{
  if (mList.size < mList.maxSize) {
    ++mList.size;
    mList.updated = true;

    mList.missions[mList.size] = *m;
  }
}

void MissionSetStart(const Mission *newStartingPoint)
{
  mList.startingPoint = *newStartingPoint;
}

void MissionClear(void)
{
	mList.size = 0;

	mList.updated = true;

	Mission newStartingPoint = {};

	MissionSetStart(&newStartingPoint);
}

void MissionInit(void)
{
	mList.maxSize = MISSION_MANAGER_MAX_MISSIONS;
}

void MissionSetCurrentIndex(uint8_t index)
{
	if (index < mList.size) {
		mList.currentIndex = index;
	}
}

int16_t MissionGetCurrentIndex(void)
{
	if (mList.size > 0) {
		return (int8_t)mList.currentIndex;
	} else {
		return -1;
	}
}

/* Output and update for atomic system: '<S62>/GetMission' */
bool MissionGet(uint8_t index, Mission *m)
{
	if (index < mList.size) {
		*m = mList.missions[index];
		return true;
	} else {
		return false;
	}
}

void MissionGetStart(Mission *startingPoint)
{
  *startingPoint = mList.startingPoint;
}

uint8_t MissionGetCount(void)
{
	return mList.size;
}
