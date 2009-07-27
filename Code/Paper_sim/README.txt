This is a boat model based on the following paper:

Pérez and Blanke. Mathematical Ship Modeling for Control Applications.  (2002) pp. 1-22

This model uses constants defined in boat_constants.m as the coefficients for the boat dynamics. Most of the 
constants are currently made up or loosely approximated (eg. the intertias are based on a cuboid roughly the 
volume of the boat).

Currently the model tops out at 1.93m/s when at maximum throttle. This model is stable.

There are many properties of the boat that need to be experimentally determined to improve this model and are 
listed under the experimental constants in boat_consants.m.

The model Currently only implements a couple of forces/moments:

Drag (Using wg1 sim model)
Thrust (Using wg1 sim model)
Rudder moment (using wg1 sim model)
