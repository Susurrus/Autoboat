/*
 * File: controller_types.h
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

#ifndef RTW_HEADER_controller_types_h_
#define RTW_HEADER_controller_types_h_
#include "rtwtypes.h"
#include "MissionManager.h"
#ifndef _DEFINED_TYPEDEF_FOR_struct_Hj6A0b43p4nGcJ1RP8ThEE_
#define _DEFINED_TYPEDEF_FOR_struct_Hj6A0b43p4nGcJ1RP8ThEE_

typedef struct {
  real32_T coordinates[3];
  real32_T otherCoordinates[3];
  uint8_T refFrame;
  uint8_T action;
  real32_T parameters[4];
  boolean_T autocontinue;
} struct_Hj6A0b43p4nGcJ1RP8ThEE;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_struct_GwoBuiHdYF9UGsRWqSjnYE_
#define _DEFINED_TYPEDEF_FOR_struct_GwoBuiHdYF9UGsRWqSjnYE_

typedef struct {
  real32_T coordinates[3];
  real32_T otherCoordinates[3];
  uint8_T refFrame;
  uint8_T action;
  real32_T parameters[4];
  boolean_T autocontinue;
} struct_GwoBuiHdYF9UGsRWqSjnYE;

#endif

/* Forward declaration for rtModel */
typedef struct RT_MODEL_controller RT_MODEL_controller;

#endif                                 /* RTW_HEADER_controller_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
