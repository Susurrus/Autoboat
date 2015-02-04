# This script updates the C-library version of MAVLink in use by the SeaSlug. So the generated
# header files will be copied into both the Autoboat and qgroundcontrol repository
# It assumes that the Autoboat repository is up-to-date and the reference.
# It also assumes that in the same PROJ_DIR as the Autoboat repository is a mavlink
# and qgroundcontrol directory. The MAVLink project will be automatically updated, but QGC will
# not as that may not be desired.
PROJ_DIR=~/Projects
AUTOBOAT_DIR=$PROJ_DIR/Autoboat
MAVLINK_DIR=$PROJ_DIR/mavlink
QGROUNDCONTROL_DIR=$PROJ_DIR/qgroundcontrol
AUTOBOAT_MAVDIR=$AUTOBOAT_DIR/Code/Libs/MAVLink
QGROUNDCONTROL_MAVDIR=$QGROUNDCONTROL_DIR/libs/mavlink/include/mavlink/v1.0

# Update the MAVLink repository
cd $PROJ_DIR/mavlink
git reset --hard HEAD
git pull upstream master

# Copy over the most recent SeaSlug message definitions
cp $AUTOBOAT_MAVDIR/seaslug.xml $MAVLINK_DIR/message_definitions/v1.0/seaslug.xml

# Generate new MAVLink library headers
rm -r $PROJ_DIR/mavlink_c_seaslug
python -m pymavlink.tools.mavgen --lang=C -o $PROJ_DIR/mavlink_c_seaslug $MAVLINK_DIR/message_definitions/v1.0/seaslug.xml

# Copy these files over to the Autoboat repository
rm -r $AUTOBOAT_MAVDIR/*.h $AUTOBOAT_MAVDIR/common $AUTOBOAT_MAVDIR/seaslug
cp -r $PROJ_DIR/mavlink_c_seaslug/* $AUTOBOAT_MAVDIR

# Prompt the user if they want to update the QGroundControl project before overwriting its
# mavlink library
while true
do
  # Prompt the user, and read command line argument
  read -p "Update the qgrouncontrol project from git before copying MAVLink headers? [Y/n] " answer

  # Process user input
  case $answer in
   [yY]* ) cd $QGROUNDCONTROL_DIR
           git pull upstream master
           break;;

   [nN]* ) break;;

   * )     echo "Enter Y or n.";;
  esac
done

# Overwrite QGroundControl's MAVLink files with this latest version
rm -r $QGROUNDCONTROL_MAVDIR/*
cp -r $PROJ_DIR/mavlink_c_seaslug/* $QGROUNDCONTROL_MAVDIR
