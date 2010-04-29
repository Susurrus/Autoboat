1. Introduction
===============

This folder contains various projects containing code pertaining to the Overboat.

2. Projects
===========

RC_control
----------

A model for running on the Explorer16 dsPIC development board. Allows for radio control of the Overboat. Currently only operational for the rudder. Main propulsion is relient on an installed ECAN/LIN PICtail and use of the ECAN blockset to be included with Lubin's blockset.

Paper_sim
---------

A simulation model built off of Pérez and Blanke 2002. Relient on many coefficients that need to be found experimentally, so it's not very accurate right now. Hopefully this will be the approach taken for the final boat model.

basic_model
-----------

Implements a crude inverse-bicycle model for the boat. Based mostly off of the wg1 code. Provides an estimate of energy usage and gain from solar panels. Includes autonomous waypoint controller from previous waypoint_control model.