/*
 * File: MissionManager.c
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

#include "MissionManager.h"

/* Include model header file for global data */
#include "rtwtypes.h"

// Create our mission list
MissionList mList = {};

void lla2ltp(const int32_T x[3], float y[3])
{

}

/*
 * Output and update for action system:
 *    '<S9>/If Action Subsystem1'
 *    '<S10>/If Action Subsystem1'
 */
void controller_IfActionSubsystem1(uint8_T rtu_index, const Mission
  rtu_missions[16], const Mission *rtu_mission, Mission rty_rv_m[16], int8_T
  *rty_rv_s)
{
  uint8_T rtb_Bias_n;
  int16_T i;

  /* Bias: '<S17>/Bias' */
  rtb_Bias_n = (uint8_T)((uint16_T)rtu_index + (uint16_T)((uint8_T)1U));

  /* DataStoreWrite: '<S17>/Data Store Write' */
  mList.size = rtb_Bias_n;

  /* DataStoreWrite: '<S17>/Data Store Write1' incorporates:
   *  Constant: '<S17>/Constant'
   */
  mList.updated = TRUE;

  /* Assignment: '<S17>/Assignment' */
  for (i = 0; i < 16; i++) {
    rty_rv_m[i] = rtu_missions[i];
  }

  rty_rv_m[(int32_T)rtu_index] = *rtu_mission;

  /* End of Assignment: '<S17>/Assignment' */

  /* DataTypeConversion: '<S17>/Data Type Conversion' */
  *rty_rv_s = (int8_T)rtb_Bias_n;
}

/*
 * Output and update for action system:
 *    '<S9>/If Action Subsystem'
 *    '<S10>/If Action Subsystem'
 */
void controller_IfActionSubsystem(const Mission rtu_0[16], int8_T *rty_rv_found,
  Mission rty_rv_missions[16])
{
  int16_T i;

  /* SignalConversion: '<S16>/OutportBufferForrv_found' incorporates:
   *  Constant: '<S16>/Constant'
   */
  *rty_rv_found = (-1);

  /* Inport: '<S16>/missions' */
  for (i = 0; i < 16; i++) {
    rty_rv_missions[i] = rtu_0[i];
  }

  /* End of Inport: '<S16>/missions' */
}

/* Output and update for atomic system: '<S4>/AppendMission' */
void AppendMission(const Mission *rtu_mission, int8_T *rty_newSize)
{
  /* local block i/o variables */
  Mission rtb_missions_d[16];
  Mission rtb_Merge2[16];
  uint8_T rtb_size;
  int16_T i;

  /* BusSelector: '<S9>/Bus Selector' incorporates:
   *  DataStoreRead: '<S9>/Data Store Read'
   */
  rtb_size = mList.size;
  for (i = 0; i < 16; i++) {
    rtb_missions_d[i] = mList.missions[i];
  }

  /* End of BusSelector: '<S9>/Bus Selector' */

  /* If: '<S9>/If' incorporates:
   *  DataStoreRead: '<S9>/Data Store Read'
   */
  if (rtb_size < mList.maxSize) {
    /* Outputs for IfAction SubSystem: '<S9>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S17>/Action Port'
     */
    controller_IfActionSubsystem1(rtb_size, rtb_missions_d, &(*rtu_mission),
      rtb_Merge2, &(*rty_newSize));

    /* End of Outputs for SubSystem: '<S9>/If Action Subsystem1' */
  } else {
    /* Outputs for IfAction SubSystem: '<S9>/If Action Subsystem' incorporates:
     *  ActionPort: '<S16>/Action Port'
     */
    controller_IfActionSubsystem(rtb_missions_d, &(*rty_newSize), rtb_Merge2);

    /* End of Outputs for SubSystem: '<S9>/If Action Subsystem' */
  }

  /* End of If: '<S9>/If' */

  /* DataStoreWrite: '<S9>/Data Store Write' */
  for (i = 0; i < 16; i++) {
    mList.missions[i] = rtb_Merge2[i];
  }

  /* End of DataStoreWrite: '<S9>/Data Store Write' */
}

/* Output and update for atomic system: '<S4>/AppendMission1' */
void controller_AppendMission(const Mission *rtu_mission)
{
  /* local block i/o variables */
  Mission rtb_missions_j[16];
  Mission rtb_Merge2_o[16];
  int8_T rtb_Merge1_k;
  uint8_T rtb_size_f;
  int16_T i;

  /* BusSelector: '<S10>/Bus Selector' incorporates:
   *  DataStoreRead: '<S10>/Data Store Read'
   */
  rtb_size_f = mList.size;
  for (i = 0; i < 16; i++) {
    rtb_missions_j[i] = mList.missions[i];
  }

  /* End of BusSelector: '<S10>/Bus Selector' */

  /* If: '<S10>/If' incorporates:
   *  DataStoreRead: '<S10>/Data Store Read'
   */
  if (rtb_size_f < mList.maxSize) {
    /* Outputs for IfAction SubSystem: '<S10>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S20>/Action Port'
     */
    controller_IfActionSubsystem1(rtb_size_f, rtb_missions_j, &(*rtu_mission),
      rtb_Merge2_o, &rtb_Merge1_k);

    /* End of Outputs for SubSystem: '<S10>/If Action Subsystem1' */
  } else {
    /* Outputs for IfAction SubSystem: '<S10>/If Action Subsystem' incorporates:
     *  ActionPort: '<S19>/Action Port'
     */
    controller_IfActionSubsystem(rtb_missions_j, &rtb_Merge1_k, rtb_Merge2_o);

    /* End of Outputs for SubSystem: '<S10>/If Action Subsystem' */
  }

  /* End of If: '<S10>/If' */

  /* DataStoreWrite: '<S10>/Data Store Write' */
  for (i = 0; i < 16; i++) {
    mList.missions[i] = rtb_Merge2_o[i];
  }

  /* End of DataStoreWrite: '<S10>/Data Store Write' */
}

/*
 * Output and update for atomic system:
 *    '<S11>/SetStartingPoint'
 *    '<S50>/SetStartingPoint'
 */
void SetStartingPoint(const Mission *rtu_startingPoint)
{
  /* DataStoreWrite: '<S22>/Data Store Write' */
  mList.startingPoint = *rtu_startingPoint;
}

/* Output and update for atomic system: '<S4>/ClearMissionList' */
void ClearMissionList(void)
{
  /* local block i/o variables */
  Mission rtb_BusCreator_p;

  /* DataStoreWrite: '<S11>/Data Store Write' incorporates:
   *  Constant: '<S11>/Constant'
   */
  mList.size = ((uint8_T)0U);

  /* DataStoreWrite: '<S11>/Data Store Write1' incorporates:
   *  Constant: '<S11>/Constant1'
   */
  mList.updated = TRUE;

  /* BusCreator: '<S11>/Bus Creator' incorporates:
   *  Constant: '<S11>/Constant2'
   *  Constant: '<S11>/Constant3'
   *  Constant: '<S11>/Constant4'
   *  Constant: '<S11>/Constant5'
   *  Constant: '<S11>/Constant6'
   *  Constant: '<S11>/Constant7'
   */
  rtb_BusCreator_p.coordinates[0L] = 0.0F;
  rtb_BusCreator_p.coordinates[1L] = 0.0F;
  rtb_BusCreator_p.coordinates[2L] = 0.0F;
  rtb_BusCreator_p.otherCoordinates[0L] = 0.0F;
  rtb_BusCreator_p.otherCoordinates[1L] = 0.0F;
  rtb_BusCreator_p.otherCoordinates[2L] = 0.0F;
  rtb_BusCreator_p.refFrame = ((uint8_T)0U);
  rtb_BusCreator_p.action = ((uint8_T)0U);
  rtb_BusCreator_p.parameters[0L] = 0.0F;
  rtb_BusCreator_p.parameters[1L] = 0.0F;
  rtb_BusCreator_p.parameters[2L] = 0.0F;
  rtb_BusCreator_p.parameters[3L] = 0.0F;
  rtb_BusCreator_p.autocontinue = TRUE;

  /* Outputs for Atomic SubSystem: '<S11>/SetStartingPoint' */
  SetStartingPoint(&rtb_BusCreator_p);

  /* End of Outputs for SubSystem: '<S11>/SetStartingPoint' */
}

/* Output and update for atomic system: '<S4>/Mission Init' */
void MissionInit(void)
{
  /* DataStoreWrite: '<S12>/Data Store Write' incorporates:
   *  Constant: '<S12>/Constant'
   */
  mList.maxSize = ((uint8_T)16U);
}

/*
 * Output and update for atomic system:
 *    '<S4>/SetCurrentMission'
 *    '<S49>/SetCurrentMission'
 *    '<S50>/SetCurrentMission'
 */
void SetCurrentMission(uint8_T rtu_index)
{
  /* If: '<S13>/If' incorporates:
   *  DataStoreRead: '<S13>/Data Store Read'
   */
  if (rtu_index < mList.size) {
    /* Outputs for IfAction SubSystem: '<S13>/If Action Subsystem' incorporates:
     *  ActionPort: '<S23>/Action Port'
     */
    /* DataStoreWrite: '<S23>/Data Store Write' */
    mList.currentIndex = rtu_index;

    /* End of Outputs for SubSystem: '<S13>/If Action Subsystem' */
  }

  /* End of If: '<S13>/If' */
}

/*
 * Output and update for atomic system:
 *    '<S31>/GetCurrentMission'
 *    '<S32>/GetCurrentMission'
 */
void GetCurrentMission(int8_T *rty_index)
{
  /* If: '<S34>/If' incorporates:
   *  DataStoreRead: '<S34>/Data Store Read2'
   */
  if (mList.size > 0) {
    /* Outputs for IfAction SubSystem: '<S34>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S40>/Action Port'
     */
    /* DataTypeConversion: '<S40>/Data Type Conversion' incorporates:
     *  DataStoreRead: '<S40>/Data Store Read1'
     */
    *rty_index = (int8_T)mList.currentIndex;

    /* End of Outputs for SubSystem: '<S34>/If Action Subsystem1' */
  } else {
    /* Outputs for IfAction SubSystem: '<S34>/If Action Subsystem' incorporates:
     *  ActionPort: '<S39>/Action Port'
     */
    /* SignalConversion: '<S39>/OutportBufferForindex' incorporates:
     *  Constant: '<S39>/Constant'
     */
    *rty_index = (-1);

    /* End of Outputs for SubSystem: '<S34>/If Action Subsystem' */
  }

  /* End of If: '<S34>/If' */
}

/*
 * Output and update for action system:
 *    '<S35>/If Action Subsystem1'
 *    '<S44>/If Action Subsystem1'
 */
void controller_IfActionSubsystem1_g(const Mission rtu_missions[16], uint8_T
  rtu_index, boolean_T *rty_rv, Mission *rty_mission)
{
  /* SignalConversion: '<S42>/OutportBufferForrv' incorporates:
   *  Constant: '<S42>/Constant'
   */
  *rty_rv = TRUE;

  /* Selector: '<S42>/Selector' */
  *rty_mission = rtu_missions[(int32_T)rtu_index];
}

/*
 * Output and update for action system:
 *    '<S35>/If Action Subsystem'
 *    '<S44>/If Action Subsystem'
 */
void controller_IfActionSubsystem_l(const Mission rtu_missions[16], boolean_T
  *rty_rv, Mission *rty_mission)
{
  /* SignalConversion: '<S41>/OutportBufferForrv' incorporates:
   *  Constant: '<S41>/Constant'
   */
  *rty_rv = FALSE;

  /* Selector: '<S41>/Selector' incorporates:
   *  Constant: '<S41>/Constant1'
   */
  *rty_mission = rtu_missions[(int32_T)((uint8_T)0U)];
}

/* Output and update for atomic system: '<S31>/GetMission' */
void GetMission(uint8_T rtu_index, Mission *rty_Mission, boolean_T *rty_found)
{
  /* local block i/o variables */
  Mission rtb_missions_k[16];
  int16_T i;

  /* BusSelector: '<S35>/Bus Selector' incorporates:
   *  DataStoreRead: '<S35>/Data Store Read'
   */
  for (i = 0; i < 16; i++) {
    rtb_missions_k[i] = mList.missions[i];
  }

  /* End of BusSelector: '<S35>/Bus Selector' */

  /* If: '<S35>/If' incorporates:
   *  DataStoreRead: '<S35>/Data Store Read'
   */
  if (rtu_index < mList.size) {
    /* Outputs for IfAction SubSystem: '<S35>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S42>/Action Port'
     */
    controller_IfActionSubsystem1_g(rtb_missions_k, rtu_index, &(*rty_found), &(*
      rty_Mission));

    /* End of Outputs for SubSystem: '<S35>/If Action Subsystem1' */
  } else {
    /* Outputs for IfAction SubSystem: '<S35>/If Action Subsystem' incorporates:
     *  ActionPort: '<S41>/Action Port'
     */
    controller_IfActionSubsystem_l(rtb_missions_k, &(*rty_found), &(*rty_Mission));

    /* End of Outputs for SubSystem: '<S35>/If Action Subsystem' */
  }

  /* End of If: '<S35>/If' */
}

/* Output and update for atomic system: '<S36>/GetStartingPoint' */
void GetStartingPoint(Mission *rty_startingPoint)
{
  /* DataStoreRead: '<S43>/Data Store Read' */
  *rty_startingPoint = mList.startingPoint;
}

/* Output and update for atomic system: '<S37>/GetMission' */
void controller_GetMission(uint8_T rtu_index, Mission *rty_Mission, boolean_T
  *rty_found)
{
  /* local block i/o variables */
  Mission rtb_missions_m[16];
  int16_T i;

  /* BusSelector: '<S44>/Bus Selector' incorporates:
   *  DataStoreRead: '<S44>/Data Store Read'
   */
  for (i = 0; i < 16; i++) {
    rtb_missions_m[i] = mList.missions[i];
  }

  /* End of BusSelector: '<S44>/Bus Selector' */

  /* If: '<S44>/If' incorporates:
   *  DataStoreRead: '<S44>/Data Store Read'
   */
  if (rtu_index < mList.size) {
    /* Outputs for IfAction SubSystem: '<S44>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S46>/Action Port'
     */
    controller_IfActionSubsystem1_g(rtb_missions_m, rtu_index, &(*rty_found), &(*
      rty_Mission));

    /* End of Outputs for SubSystem: '<S44>/If Action Subsystem1' */
  } else {
    /* Outputs for IfAction SubSystem: '<S44>/If Action Subsystem' incorporates:
     *  ActionPort: '<S45>/Action Port'
     */
    controller_IfActionSubsystem_l(rtb_missions_m, &(*rty_found), &(*rty_Mission));

    /* End of Outputs for SubSystem: '<S44>/If Action Subsystem' */
  }

  /* End of If: '<S44>/If' */
}

/* Output and update for atomic system: '<S32>/GetMissionCount' */
void GetMissionCount(uint8_T *rty_size)
{
  /* DataStoreRead: '<S48>/Data Store Read' */
  *rty_size = mList.size;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
