#ifndef MISSIONS_H
#define MISSIONS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
  float coordinates[3];
  float otherCoordinates[3];
  uint8_t refFrame;
  uint8_t action;
  float parameters[4];
  bool autocontinue;
} Mission;

#endif