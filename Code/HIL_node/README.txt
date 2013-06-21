This code implements a CAN to UDP bridge for use in HIL simulation. It runs atop  a CAN node with a ENC28J60 Ethernet shield. It emulates all CAN-based sensors and sits between the primary controller for the SeaSlug and its Simulink simulation.

This code builds on the TCP/IP stack provided by Microchip and available directly from them. While none of their library code has been modified for this project, it has been included here to simplify things.

# Usage
This code should be compiled for the CAN node and programmed onboard. That node should have an ethernet shield and be connected to a laptop (it has a DHCP server so IP address assignment is automatic) as well as to the primary controller node via CAN.

# Compilation

Compilation is done completely via Microchip's MPLAB X IDE and the XC16 compiler.

Required include directories:
 ./TCPIP
 .
 ../Libs/C 

Required source files:
 ./*.c
 ./TCPIP/TCPIP Stack/*.c
 ../Libs/C/[Acs300,CanMessages,CircularBuffer,Ecan1,MessageScheduler,Nmea2000,Nmea2000Encode,Node,Rudder,Timer2,Timer3,Timer4].c
