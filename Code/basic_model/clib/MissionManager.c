#include "MissionManager.h"

#include <string.h>

// Include stdio.h for the unit test mode.
#ifdef UNIT_TEST
#include <stdio.h>
#include <assert.h>
#endif // UNIT_TEST

// Store the mission list globally.
static MissionList mList;

// Update a single mission within the list.
bool UpdateMission(uint8_t index, Mission *m) {
	if (index < mList.size) {
		memcpy(&mList.missionList[index], m, sizeof(Mission));
		return true;
	} else {
		return false;
	}
}

// Insert a mission into the list. Valid indices are 0 through the current number of missions and using other indices will result in false being returned from this.
int8_t AppendMission(Mission *m) {
	if (mList.size < MAX_MISSIONS) {
		memcpy(&mList.missionList[mList.size], m, sizeof(Mission));
		return mList.size++;
	} else {
		return -1;
	}
}

// Clear all missions. Follow standard error reporting where true is success and false is failure.
bool ClearMissionList(void) {
	mList.size = 0;
	return true;
}

// Retried a mission via the specified index. Fills the provided Mission struct with the information.
// A boolean false is returned if the index is invalid given the current list, otherwise true is returned.
bool GetMission(uint8_t index, Mission *m) {
	if (index < mList.size) {
		memcpy(m, &mList.missionList[index], sizeof(Mission));
		return true;
	} else {
		return false;
	}
}

uint8_t GetMissionCount(void) {
	return mList.size;
}

// Returns the index of the currently active mission. If there is no current mission, then the return value will be 255 (0xFF).
uint8_t GetCurrentMission(void) {
	return mList.currentMissionIndex;
}

// Sets the current mission to the specified mission by its index. If an invalid index is specified, false is returned.
// There's no comparison with MAX_MISSIONS as the size of the mission list should have already been limited by that.
bool SetCurrentMission(uint8_t index) {
	if (index < mList.size) {
		mList.currentMissionIndex = index;
		return true;
	} else {
		return false;
	}
}

// Include a main() when compiling as a unit test.
#ifdef UNIT_TEST

bool MissionIsEqual(Mission *m1, Mission *m2) {
	return (m2->coord1 == m1->coord1 &&
	       m2->coord2 == m1->coord2 &&
	       m2->coord3 == m1->coord3 &&
	       m2->refFrame == m1->refFrame &&
	       m2->action == m1->action &&
	       m2->param1 == m1->param1 &&
	       m2->param2 == m1->param2 &&
	       m2->param3 == m1->param3 &&
	       m2->param4 == m1->param4 &&
	       m2->autocontinue == m1->autocontinue
	);
}

int main() {
	printf("Testing MissionManager.c. Failed assertions will output below. If none, all tests passed.\n");
	
	// Confirm that things initialize correctly.
	assert(GetCurrentMission() == 0 && GetMissionCount() == 0);
	
	// Try to add a single empty mission and confirm that works as expected.
	Mission m1 = {
		100,
		20,
		0,
		1,
		1,
		10,
		0,
		0,
		0,
		true
	};

	
	Mission m2 = {
		250,
		250,
		100,
		1,
		2,
		30,
		0,
		0,
		0,
		true
	};

	Mission tmpMission;

	// This is a basic test of inserting a mission.
	AppendMission(&m1);
	assert(GetCurrentMission() == 0);
	assert(GetMissionCount() == 1);
	assert(!GetMission(1, &tmpMission));
	assert(GetMission(0, &tmpMission));
	assert(MissionIsEqual(&m1, &tmpMission));

	// Now clear the list and confirm that it's empty. This is done by checking
	// that the current mission is reset to 0 along with the mission count. Finally
	// we try to pull the first mission and confirm that we can't.
	ClearMissionList();
	assert(GetCurrentMission() == 0 && GetMissionCount() == 0);
	Mission m4 = {
		134,
		456,
		124,
		1,
		2,
		30,
		0,
		0,
		0,
		true
	};
	Mission m5 = {
		134,
		456,
		124,
		1,
		2,
		30,
		0,
		0,
		0,
		true
	};
	assert(!GetMission(0, &m5));
	assert(MissionIsEqual(&m4, &m5));

	// Try to switch the current mission. Here we insert two missions and try to switch the
	// current mission to the second one.
	ClearMissionList();
	AppendMission(&m1);
	AppendMission(&m1);
	assert(SetCurrentMission(1));
	assert(!SetCurrentMission(2));
	assert(GetCurrentMission() == 1);
	assert(SetCurrentMission(0));
	assert(!SetCurrentMission(MAX_MISSIONS));
	
	// Confirm that updating the mission works as expected.
	ClearMissionList();

	AppendMission(&m1);
	AppendMission(&m1);
	assert(GetMissionCount() == 2);

	UpdateMission(1, &m2);
	assert(GetMissionCount() == 2);

	assert(GetMission(0, &tmpMission));
	assert(MissionIsEqual(&tmpMission, &m1));
	assert(GetMission(1, &tmpMission));
	assert(MissionIsEqual(&tmpMission, &m2));
	
	// Now try to fill up the entire list and confirm that it can get to MAX_MISSIONS before failing.
	ClearMissionList();
	int i, g;
	for (i=0;i<MAX_MISSIONS;i++) {
		assert((g = AppendMission(&m1)) != -1);
		assert(i == g);
		if (i != MAX_MISSIONS - 1) {
			assert(!UpdateMission(MAX_MISSIONS - 1, &m2));
		}
		assert(GetMission(i, &tmpMission));
		assert(MissionIsEqual(&tmpMission, &m1));
	}
	assert(AppendMission(&m1) == -1);
	assert(!UpdateMission(MAX_MISSIONS, &m2));
	

	return 0;
}

#endif // UNIT_TEST
