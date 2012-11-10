1. Introduction
===============

This folder contains various projects containing code pertaining to the Overboat.

2. Projects
===========

basic_model
-----------

Implements a crude inverse-bicycle model for the boat that resides in Boat_sim.mdl. Based mostly off of the wg1 code. Provides an estimate of energy usage and gain from solar panels. Includes autonomous waypoint controller from previous waypoint_control model.

This simulation code is extended by a compilable version of the controller. This has been designed to run on a CAN_node version 2.0. Requirements for compilation include the ECAN_dspic library (hosted at Github under the Susurrus account) along with the MicroSimulink-Library (also hosted at Github under the Susurrus account).

There are some more requirements:
 * The stdint.h library included with the C30/XC16 compiler needs to be modified with header guards for the uintmax_t/intmax_t variables.
 * This library should be duplicated as the `inttypes.h` file in order for MAVLink to compile.
 * It requires a SeaSlug-compiled MAVLink library.
 * A heap size of 1024 bytes should be specified when the project is compiled in MPLAB X. I don't know a way to specify a heap within Simulink using Lubin's blockset, so this is the ONLY way to compile this currently.

Power Node
----------

This is another Simulink/Lubin's Blockset-based dsPIC33f project. This runs aboard a 1.x version of the CAN node and translates the sensor values for the current/voltage sensor on the main power rail into CAN messages. These are then processed by the primary controller node.

Paper_sim
---------

A simulation model built off of Pérez and Blanke 2002. Relient on many coefficients that need to be found experimentally, so it's not very accurate right now. Hopefully this will be the approach taken for the final boat model.

Rudder Subsystem
----------------

This also runs aboard a CAN node (version 2.0). It integrates with the rudder and provides a CAN interface to it. Handles calibration as well. This has the same compilation requirements as `basic_model`.
