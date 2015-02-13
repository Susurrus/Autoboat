# This script updates the C-library version of MAVLink in use by qgroundcontrol. It updates the Autoboat repository and copies the relevant MAVLink files over the ones in the qgroundcontrol directory.
# It also assumes that in the same PROJ_DIR as the Autoboat repository is a mavlink
# and qgroundcontrol directory. The MAVLink project will be automatically updated, but QGC will
# not as that may not be desired.
PROJ_DIR=~/Projects
AUTOBOAT_DIR=$PROJ_DIR/Autoboat
AUTOBOAT_MAVDIR=$AUTOBOAT_DIR/Code/Libs/MAVLink
QGROUNDCONTROL_DIR=$PROJ_DIR/qgroundcontrol
QGROUNDCONTROL_MAVDIR=$QGROUNDCONTROL_DIR/libs/mavlink/include/mavlink/v1.0

# Update the Autoboat repository
cd $AUTOBOAT_DIR
git reset --hard HEAD
git pull origin master

# Overwrite QGroundControl's MAVLink files with this latest version
rm -r $QGROUNDCONTROL_MAVDIR/*
cp -r $AUTOBOAT_MAVDIR/* $QGROUNDCONTROL_MAVDIR
