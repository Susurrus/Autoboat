/*
 * File: MissionManager.h
 *
 * Code generated for Simulink model 'controller'.
 *
 * Model version                  : 1.2450
 * Simulink Coder version         : 8.1 (R2011b) 08-Jul-2011
 * TLC version                    : 8.1 (Jul  9 2011)
 * C/C++ source code generated on : Mon Feb 10 13:10:07 2014
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
 * Emulation hardware selection:
 *    Differs from embedded hardware (Microchip->dsPIC)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include <stdint.h>

#ifndef RTW_HEADER_MissionManager_h_
#define RTW_HEADER_MissionManager_h_
#include "Missions.h"
#include "rtwtypes.h"
#ifndef controller_COMMON_INCLUDES_
# define controller_COMMON_INCLUDES_
#include <math.h>
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "rt_defines.h"
#endif                                 /* controller_COMMON_INCLUDES_ */

#include "controller_types.h"

typedef struct {
  uint8_T currentIndex;
  uint8_T size;
  boolean_T updated;
  uint8_T maxSize;
  Mission startingPoint;
  Mission missions[16];
} MissionList;

// Create our mission list
extern MissionList mList;
extern void lla2ltp(const int32_t[3], float[3]);

extern void controller_IfActionSubsystem1(uint8_T rtu_index, const Mission
  rtu_missions[16], const Mission *rtu_mission, Mission rty_rv_m[16], int8_T
  *rty_rv_s);
extern void controller_IfActionSubsystem(const Mission rtu_0[16], int8_T
  *rty_rv_found, Mission rty_rv_missions[16]);
extern void AppendMission(const Mission *rtu_mission, int8_T *rty_newSize);
extern void controller_AppendMission(const Mission *rtu_mission);
extern void SetStartingPoint(const Mission *rtu_startingPoint);
extern void ClearMissionList(void);
extern void MissionInit(void);
extern void SetCurrentMission(uint8_T rtu_index);
extern void GetCurrentMission(int8_T *rty_index);
extern void controller_IfActionSubsystem1_g(const Mission rtu_missions[16],
  uint8_T rtu_index, boolean_T *rty_rv, Mission *rty_mission);
extern void controller_IfActionSubsystem_l(const Mission rtu_missions[16],
  boolean_T *rty_rv, Mission *rty_mission);
extern void GetMission(uint8_T rtu_index, Mission *rty_Mission, boolean_T
  *rty_found);
extern void GetStartingPoint(Mission *rty_startingPoint);
extern void controller_GetMission(uint8_T rtu_index, Mission *rty_Mission,
  boolean_T *rty_found);
extern void GetMissionCount(uint8_T *rty_size);

#endif                                 /* RTW_HEADER_MissionManager_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
