#ifndef TCPIPCONFIG_H
#define TCPIPCONFIG_H

// Use the DHCP server application from the TCPIP library. Note that this DHCP server only supports
// a single client.
#define STACK_USE_DHCP_SERVER

// Configure network settings like hostname, MAC, IP address, etc.
#define HOSTNAME			"UDP2SERIALBRIDGE"

#define MAC_BYTE1            (0x00)	// Use the default of 00-04-A3-00-00-00
#define MAC_BYTE2            (0x04)	// if using an ENCX24J600, MRF24WB0M, or
#define MAC_BYTE3            (0xA3)	// PIC32MX6XX/7XX internal Ethernet
#define MAC_BYTE4            (0x00)	// controller and wish to use the
#define MAC_BYTE5            (0x00)	// internal factory programmed MAC
#define MAC_BYTE6            (0x00)	// address instead.

#define IP_ADDR_BYTE1        (10ul)
#define IP_ADDR_BYTE2        (0ul)
#define IP_ADDR_BYTE3        (0ul)
#define IP_ADDR_BYTE4        (1ul)

#define MASK_BYTE1           (255ul)
#define MASK_BYTE2           (255ul)
#define MASK_BYTE3           (255ul)
#define MASK_BYTE4           (0ul)

#define GATE_BYTE1           IP_ADDR_BYTE1
#define GATE_BYTE2           IP_ADDR_BYTE2
#define GATE_BYTE3           IP_ADDR_BYTE3
#define GATE_BYTE4           IP_ADDR_BYTE4

#define PRIMARY_DNS_BYTE1	IP_ADDR_BYTE1
#define PRIMARY_DNS_BYTE2	IP_ADDR_BYTE2
#define PRIMARY_DNS_BYTE3	IP_ADDR_BYTE3
#define PRIMARY_DNS_BYTE4	IP_ADDR_BYTE4

#define SECONDARY_DNS_BYTE1	(0ul)
#define SECONDARY_DNS_BYTE2	(0ul)
#define SECONDARY_DNS_BYTE3	(0ul)
#define SECONDARY_DNS_BYTE4	(0ul)

// Use UDP.
#define STACK_USE_UDP

// Specify the server port to use for receiving UDP data messages.
#define UDP_SERVER_PORT 14550

// Specify the server port to use for transmitting UDP data messages.
#define UDP_CLIENT_PORT 14551

// Set the maximum number of UDP sockets. 3 covers DHCP, an input UDP socket, and an output UDP
// socket.
#define MAX_UDP_SOCKETS 3

// Specify that the stack will be used as a client. This is necessary for the IP_ADDRESS connection
// features of the UDPOpenEX() function.
#define STACK_CLIENT_MODE

// The max HTTP connections must also be specified even though we're not using it because the
// Microchip TCPIP stack is dumb.
#define MAX_HTTP_CONNECTIONS 1

#endif
