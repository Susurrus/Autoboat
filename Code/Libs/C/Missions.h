#ifndef MISSIONS_H
#define MISSIONS_H

#include <stdint.h>

typedef struct {
  float coordinates[3]; // The mission location in the coordinates as specified by refFrame.
  float otherCoordinates[3]; // The mission location in the local frame if this mission was originally in global coordinates and vice-versa.
  uint8_t refFrame; // One of MAV_FRAME_*
  uint8_t action;
  float parameters[4];
  uint8_t autocontinue; // Boolean indicating whether to continue to the next mission after this one.
} Mission;

#endif
