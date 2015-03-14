#1/bin/bash
# This script updates the C-library version of MAVLink in use by the SeaSlug.
# Inputs:
#   * The seaslug.xml file.
#
# Requirements
#   * A mavlink repository sibling to the Autoboat one.
#   * An internet connection to update the MAVLink project
#
# Actions
#   * Copies the seaslug.xml file into the mavlink project
#   * Generates files into a sibling folder to the Autoboat and MAVLink repositories
#   * Copies those files into the Autoboat repository
PROJ_DIR=~/Projects
AUTOBOAT_DIR=$PROJ_DIR/Autoboat
MAVLINK_DIR=$PROJ_DIR/mavlink
AUTOBOAT_MAVDIR=$AUTOBOAT_DIR/Code/Libs/MAVLink

# Update the MAVLink repository
cd $MAVLINK_DIR
git reset --hard HEAD
git pull upstream master

# Copy over the most recent SeaSlug message definitions
cp $AUTOBOAT_MAVDIR/seaslug.xml $MAVLINK_DIR/message_definitions/v1.0/seaslug.xml

# Generate new MAVLink library headers
rm -r $PROJ_DIR/mavlink_c_seaslug
python -m pymavlink.tools.mavgen --lang=C -o $PROJ_DIR/mavlink_c_seaslug $MAVLINK_DIR/message_definitions/v1.0/seaslug.xml

# Clean up the old MAVLink Python library for our dialect
rm $MAVLINK_DIR/pymavlink/dialects/v10/seaslug.py

# Copy these files over to the Autoboat repository
rm $AUTOBOAT_MAVDIR/*.h
rm -r $AUTOBOAT_MAVDIR/common
rm -r $AUTOBOAT_MAVDIR/seaslug
cp -r $PROJ_DIR/mavlink_c_seaslug/* $AUTOBOAT_MAVDIR
