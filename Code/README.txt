This folder contains various projects containing code pertaining to the Overboat.

Projects:

RC_control - A model for running on the Explorer16 dsPIC development board. Allows for radio control of the Overboat. Currently only operational for the rudder. Main propulsion is relient on an installed ECAN/LIN PICtail and use of the ECAN blockset to be included with Lubin's blockset.

waypoint_control - A simulation model designed for exploring basic waypoint guidance. Used merely for educational purposes as the work beyond the boat plant will be replaced by SLUGS code.

Paper_sim - A simulation model built off of Pérez and Blanke 2002. Relient on many coefficients that need to be found experimentally, so it's not very accurate right now. Hopefully this will be the approach taken for the final boat model.

basic_model - Implements a crude inverse-bicycle model for the boat. Based mostly off of the wg1 code. Provides an estimate of energy reserves.