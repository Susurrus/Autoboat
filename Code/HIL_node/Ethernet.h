#ifndef ETHERNET_H
#define ETHERNET_H

#include <stdint.h>
#include "TCPIP Stack/TCPIP.h"

void EthernetInit(void);
void EthernetRun(void (*ProcessData)(BYTE *data, WORD dataLen));
void EthernetTransmit(BYTE *data, WORD dataLen);

#endif // ETHERNET_H