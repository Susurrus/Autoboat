/*
 * File: InternalVariables.h
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

#ifndef RTW_HEADER_InternalVariables_h_
#define RTW_HEADER_InternalVariables_h_
#include "rtwtypes.h"

typedef struct {
  real32_T LocalPosition[3];
  real32_T Velocity[3];
  real32_T Heading;
  real32_T Speed;
  real32_T L2Vector[3];
  real32_T Acmd;
  real32_T wp0[3];
  real32_T wp1[3];
} InternalVariables;

#endif                                 /* RTW_HEADER_InternalVariables_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
