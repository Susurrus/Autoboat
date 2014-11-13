// Define this as the file containing main before including configuration files.
#include "HardwareProfile.h"
#include "TCPIPConfig.h"

#include "Ethernet.h"
#include "Node.h"

#define THIS_IS_STACK_APPLICATION
#include "TCPIP Stack/TCPIP.h"
#include "TCPIP Stack/StackTsk.h"
#include "GenericTypeDefs.h"

#include <stdio.h>

// Declare AppConfig structure and some other supporting stack variables. This is a global variable
// used by all of the TCPIP stack to pull common info.
APP_CONFIG AppConfig = {};

// Private helper functions.
static void InitAppConfig(void);
static void InitBoard(void);

// Declare a module-level variable for use as a temporary variable when processing packets. This
// will be filled with the info of the remote node after a UDP packet is received.
static NODE_INFO remoteNode;

// The local socket used for the UDP server for data connections.
static UDP_SOCKET localServerSocket = INVALID_UDP_SOCKET;

// The remote socket for transmitting UDP packets to. It's set as soon as the first proper UDP data
// packet is received.
static UDP_SOCKET remoteServerSocket = INVALID_UDP_SOCKET;

void EthernetInit(void)
{
    // Initialize hardware
    InitBoard();

    // Initialize stack-related hardware components that may be 
    // required by the UART configuration routines
    TickInit();

    // Initialize Stack and application related NV variables into AppConfig.
    InitAppConfig();

    // Seed the LFSRRand() function.
    LFSRSeedRand(GenerateRandomDWORD());

	// Initialize the MAC library.
    MACInit();

	// Initialize the ARP library.
    ARPInit();

	// Initialize UDP.
    UDPInit();

    // Open up a socket for our UDP server.
    localServerSocket = UDPOpenEx(0, UDP_OPEN_SERVER, UDP_SERVER_PORT, UDP_SERVER_PORT);
    if (localServerSocket == INVALID_UDP_SOCKET) {
        FATAL_ERROR();
    }

    // Open up a socket for our UDP client.
    remoteServerSocket = UDPOpenEx(0, UDP_OPEN_IP_ADDRESS, UDP_CLIENT_PORT, UDP_CLIENT_PORT);
    if (remoteServerSocket == INVALID_UDP_SOCKET) {
        FATAL_ERROR();
    }
}

void EthernetRun(void (*ProcessData)(BYTE *data, WORD dataLen))
{
    // Now that all items are initialized, begin the co-operative
    // multitasking loop.  This infinite loop will continuously 
    // execute all stack-related tasks, as well as your own
    // application's functions.  Custom functions should be added
    // at the end of this loop.
    // Note that this is a "co-operative mult-tasking" mechanism
    // where every task performs its tasks (whether all in one shot
    // or part of it) and returns so that other tasks can do their
    // job.
    // If a task needs very long time to do its job, it must be broken
    // down into smaller pieces so that other tasks can have CPU time.

	// First prepare all UDP stuff.
	UDPTask();

	WORD dataCount;
	IP_ADDR tempLocalIP;
	BYTE cFrameType;
	BYTE cIPFrameType;

	// And then process incoming data ending this loop once a valid packet is received.
	int cont = 1;
	while (cont) {
		// Before fetching new data, be sure all old UDP data is discarded.
		UDPDiscard();

		// Fetch a packet. We stop receiving data if no data is waiting.
		if (!MACGetHeader(&remoteNode.MACAddr, &cFrameType)) {
			break;
		}

		// And now process the packet.
		switch(cFrameType) {
			// Process any ARP packets. These are used for determining a MAC-to-IP mapping.
			case MAC_ARP:
				ARPProcess();
				break;

			// Process any IP packets (of which UDP is a type).
			case MAC_IP:
				// If the received packet is not a valid IP packet, ignore it.
				if (!IPGetHeader(&tempLocalIP, &remoteNode, &cIPFrameType, &dataCount)) {
					break;
				}

				// Now if we've found a valid UDP packet, quit processing data.
				if (cIPFrameType == IP_PROT_UDP) {
					// Stop processing data if we came upon a complete UDP packet.
					if (UDPProcess(&remoteNode, &tempLocalIP, dataCount)) {
						cont = 0;
					}
				}

				break;
		}
	}

	// Send new UDP data to processing function.
	if (ProcessData) {
		WORD dataLen = 0;
		if ((dataLen = UDPIsGetReady(localServerSocket))) {
			BYTE data[dataLen];
			if (UDPGetArray(data, dataLen) == dataLen) {
				ProcessData(data, dataLen);
			}
		}
	}

	// Now run the DHCP server task. This needs to be run continuously as there's no way to tell
	// if any clients are connected to perform the lease once post-init. The default lease time
	// is also super short, 60s, so the server needs to be able to process new DHCP stuff.
	DHCPServerTask();
}

void EthernetTransmit(BYTE *data, WORD dataLen)
{
	// If a remote server socket exists and the local socket is allowing new data, send it.
	if (remoteServerSocket != INVALID_UDP_SOCKET) {
		if (UDPIsPutReady(remoteServerSocket) >= dataLen) {
			if (UDPPutArray(data, dataLen) < dataLen) {
				FATAL_ERROR();
			}
			UDPFlush();
		}
	}
}

static void InitBoard(void)
{
	// Deassert all SPI chip select lines so there isn't any problem with initialization order.
    ENC_CS_IO = 1;
    ENC_CS_TRIS = 0;
}

static void InitAppConfig(void)
{
	// Set the node's MAC address
	AppConfig.MyMACAddr.v[0] = MAC_BYTE1;
	AppConfig.MyMACAddr.v[1] = MAC_BYTE2;
	AppConfig.MyMACAddr.v[2] = MAC_BYTE3;
	AppConfig.MyMACAddr.v[3] = MAC_BYTE4;
	AppConfig.MyMACAddr.v[4] = MAC_BYTE5;
	AppConfig.MyMACAddr.v[5] = MAC_BYTE6;

	// Set the node's IP address
	AppConfig.MyIPAddr.Val = IP_ADDR_BYTE1 | IP_ADDR_BYTE2<<8 | IP_ADDR_BYTE3<<16 | IP_ADDR_BYTE4 << 24;

	// Se the default IP address for this node
	AppConfig.DefaultIPAddr.Val = AppConfig.MyIPAddr.Val;

	// Set the DNS mask and default mask
	AppConfig.MyMask.Val = MASK_BYTE1 | MASK_BYTE2<<8 | MASK_BYTE3<<16 | MASK_BYTE4<<24;
	AppConfig.DefaultMask.Val = AppConfig.MyMask.Val;

	// Set the default gateway
	AppConfig.MyGateway.Val = GATE_BYTE1 | GATE_BYTE2<<8 | GATE_BYTE3<<16 | GATE_BYTE4<<24;

	// Set the DNS servers.
	AppConfig.PrimaryDNSServer.Val = PRIMARY_DNS_BYTE1 | PRIMARY_DNS_BYTE2<<8  | PRIMARY_DNS_BYTE3<<16  | PRIMARY_DNS_BYTE4<<24;
	AppConfig.SecondaryDNSServer.Val = SECONDARY_DNS_BYTE1 | SECONDARY_DNS_BYTE2<<8  | SECONDARY_DNS_BYTE3<<16  | SECONDARY_DNS_BYTE4<<24;

	// Load the default NetBIOS Host Name
	memcpypgm2ram(AppConfig.NetBIOSName, (const void*)HOSTNAME, 16);
	FormatNetBIOSName(AppConfig.NetBIOSName);
}
