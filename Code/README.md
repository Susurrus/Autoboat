1. Introduction
===============

This folder contains various projects containing code pertaining to the Overboat.

2. Projects
===========

basic_model
-----------
Implements a crude inverse-bicycle model for the boat that resides in Boat_sim.mdl.

HIL_node
--------
Provides a CAN interface with the basic_model simulator via a serial connection (that needs a serial-to-UDP bridge). Emulates many of the other nodes in order to fool the primary_node into normal operations.

Imu_node
--------
Provides a CAN interface to the Revolution GS IMU. Broadcasts all incoming IMU data via CAN.

Libs
----
Contains all shared code used by this project.
 
Paper_sim
---------
A simulation model built off of Pérez and Blanke 2002. Relient on many coefficients that need to be found experimentally, so it's not very accurate right now. Hopefully this will be the approach taken for the final boat model.

Power_node
----------
This is another Simulink/Lubin's Blockset-based dsPIC33f project. This runs on a CAN node and translates the sensor values for the current/voltage sensor on the main power rail into CAN messages. These are then processed by the primary controller node.

Primary_node
------------
This node implements the same controller as the basic_model simulator. It has an onboard interface to an analog power sensor, but otherwise all vessel communications are done over CAN. Its groundstation output is via the UART cocnector on the CAN node.

Requirements for compilation include the ECAN_dspic library (hosted at Github under the Susurrus account) along with the MicroSimulink-Library (also hosted at Github under the Susurrus account).

Additional requirements for compiling this node
 * The stdint.h library included with the XC16 compiler needs to be modified with header guards for the uintmax_t/intmax_t variables.
 * The `stdint.h` file should be duplicated as the `inttypes.h` file in order for MAVLink to compile.
 * It requires a SeaSlug-compiled MAVLink library.
 * A heap size of 1024 bytes should be specified when the project is compiled in MPLAB X. I don't know a way to specify a heap within Simulink using Lubin's blockset, so this is the ONLY way to compile this currently.

RC_node
-------
Provides a CAN interface for an RC receiver using a CAN node. It aso has a custom shield providing a plug-in interface for Spektrum's AR6100e receiver.

Rudder_node
----------------
This also runs aboard a CAN node. It integrates with the rudder (a stepper motor driver board, potentiometer position sensor, and port/starbord limit switches) and provides a CAN interface to it. Handles calibration as well. This has the same compilation requirements as `basic_model`.

