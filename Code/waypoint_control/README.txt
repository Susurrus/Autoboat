This folder contains all files needed to run the waypoint control model, a simulation model testing waypoint control algorithms.

Files:
boat_constants.m     - A file of variables needed to run the model. They range from the simulation's initial conditions to the physical aspects of the boat.
render_boat.m        - A file containing the function render_boat() that draws a realistic yellow boat on a new figure.
waypoint_control.mdl - The model simulating a waypoint control algorithm.
README.txt           - This file.

This model is based off of the simple_model and will be used to study a waypoint guidance system. Initial testing will be of a system that can direct the boat towards a single waypoint and stop on top of it using proportional control.