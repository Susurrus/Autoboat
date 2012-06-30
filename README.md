Autoboat
========

This repository contains code used for running the Autoboat, a 20' robotic surface vessel currently in development at the University of Santa Cruz under the directive of Professor Gabriel Elkaim. This respository is therefore likely to be of little interest to outsiders, though much of the support code is fairly modular. Groundstation software for this vessel is provided via the QGroundControl project and is not contained within this repository.

File layout
-----------
 * CAD - Contains CAD designs of both mechanical and electrical systems. The mechanical systems have been modeled in SolidWorks 2010 while the electrical schematics have been generated using KiCAD.
 * Code - This contains the software used for controlling the boat. Primarily Simulink and C control its operation and MATLAB and Python scripts assist in data parsing and analysis
 * Documentation - Various documents regarding this project are stored in this directory.

Main project
------------

The main project code is housed within /Code/basic\_model/code\_gen.mdl and /Code/basic\_model/Boat\_sim.mdl.

Boat_sim.mdl provides a simulator for testing my vehicle/environmental model and the vehicle controllers. It can run solely on a computer or via a UDP port provide the vehicle/environmental simulation to an external controller for use in Hardware-in-the-Loop (HIL) testing. Relies on the MicroSimulink library (<https://github.com/Susurrus/MicroSimulink-Library>).

code\_gen.mdl contains all of the controller logic. It is organized as hardware dependent code that wraps the main controller block that is shared with Boat_sim.mdl (via the make.m script). This is designed to run aboard Microchip's Explorer16 with the dsPIC33FJ256GP710A onboard. It required both the ECAN/LIN PICtail daughterboard and the prototyping daughterboard to be available for complete hardware integration. It depends on both the MicroSimulink library (<https://github.com/Susurrus/MicroSimulink-Library>), Lubin's PIC blockset (<http://www.kerhuel.eu/wiki/Simulink_-_Embedded_Target_for_PIC>), and the ECAN blockset (<https://github.com/Susurrus/ECAN_dspic>). All of these dependencies must be correctly installed and on the path along with Microchip's C30 compiler for this project to compile to an executable.

Microsoft's Visual Studio 2010 is also required in order to compile the project. It's available for free online.

In addition to these Simulink libraries, the MAVLink C library for the Sealion must be installed. Download it from <https://github.com/mavlink/mavlink> into the same directory that this repository is contained in so that it looks like:
/MicroSimulink-Library
/ECAN_dspic
/Autoboat
/mavlink

You'll need Python 2.7 to run the MAVLink generator from within the mavlink directory:

```$ python mavgen.py --lang=C --wire-protocol=1.0 -o include ../Autoboat/Code/basic_model/Extras/sealion.xml```

This will put the MAVLink code for the Sealion dialect into ```/mavlink/include``` where it can be referenced by code_gen.mdl.

The current version of MAVLink will not compile with the C30 compiler unless the ```stdint.h``` header file within the C30 includes directory is duplicated as an ```inttypes.h``` file.
 
Further documentation
---------------------

A wiki has been hosted to provide details on this project at <http://byron.soe.ucsc.edu/autoboat/wg2/>.