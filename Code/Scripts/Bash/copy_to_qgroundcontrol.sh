#!/bin/bash
# This script updates the C-library version of MAVLink in use by qgroundcontrol. It updates the Autoboat repository and copies the relevant MAVLink files over the ones in the qgroundcontrol directory.
# It also assumes that in the same PROJ_DIR as the Autoboat repository is a mavlink
# and qgroundcontrol directory. The MAVLink project will be automatically updated, but QGC will
# not as that may not be desired.
PROJ_DIR=~/Projects
AUTOBOAT_DIR=$PROJ_DIR/Autoboat
AUTOBOAT_MAVDIR=$AUTOBOAT_DIR/Code/Libs/MAVLink
MAVLINK_DIR=$PROJ_DIR/mavlink
QGROUNDCONTROL_DIR=$PROJ_DIR/qgroundcontrol
QGROUNDCONTROL_MAVDIR=$QGROUNDCONTROL_DIR/libs/mavlink/include/mavlink/v1.0

# Update the Autoboat repository
cd $AUTOBOAT_DIR
git reset --hard HEAD
git pull origin master

# Overwrite QGroundControl's MAVLink files with this latest version
rm -r $QGROUNDCONTROL_MAVDIR/*
cp -r $AUTOBOAT_MAVDIR/* $QGROUNDCONTROL_MAVDIR

# Copy the SeaSlug MAVLink definitions file to the MAVLink project
cp $AUTOBOAT_MAVDIR/seaslug.xml $MAVLINK_DIR/message_definitions/v1.0/

# And make sure we delete any old pymavlink versions of the SeaSlug message definitions
rm $MAVLINK_DIR/pymavlink/dialects/v10/seaslug.py
