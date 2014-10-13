# This script plots position data for an entire file or a single autonomous run. Additionally it outputs the number of autonomous runs and their length when run on the entire file.

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import sys

from helpers import *


if __name__ == '__main__':

    if len(sys.argv) < 2:
        print("ERROR: Arguments required - CSV file [, autonomous run number]")
        sys.exit(1)

    # Read in the program arguments from the command line
    csv_file = sys.argv[1]
    if len(sys.argv) > 2:
        auto_run = int(sys.argv[2])
    else:
        auto_run = None

    # Extract only the necessary data from the CSV file
    data = GetDataForFields(csv_file, ['HEARTBEAT.base_mode', 'NODE_STATUS.primary_errors', 'NODE_STATUS.primary_status', 'WAYPOINT_STATUS.last_wp_north', 'WAYPOINT_STATUS.last_wp_east', 'WAYPOINT_STATUS.next_wp_north', 'WAYPOINT_STATUS.next_wp_east', 'LOCAL_POSITION_NED.y', 'LOCAL_POSITION_NED.x'])

    # Plot just the data for this specific autonomous run if specified
    if auto_run:
        # And then find the specific range of local position data for this data run
        start_time, end_time = GetTimespanForRun(data, auto_run)
        local_pos_valid_indices = np.logical_and(data['LOCAL_POSITION_NED.y'][0] >= start_time, data['LOCAL_POSITION_NED.y'][0] <= end_time)

        # Plot the vehicle track for this run
        plt.scatter(data['LOCAL_POSITION_NED.y'][1,local_pos_valid_indices], data['LOCAL_POSITION_NED.x'][1,local_pos_valid_indices])
        plt.axis('equal')
        plt.title("Vehicle track for auto run {}".format(auto_run))
        plt.ylabel('North (m)')
        plt.xlabel('East (m)')
        plt.show()
    else:
        # Display the various autonomous runs
        i = 0
        for run in GetAutonomousTimespans(data):
            length = run[1] - run[0]
            minutes = int(length // 60)
            seconds = int(length - minutes * 60)
            print("Auto run {:2}: {:02d}:{:02d}".format(i, minutes, seconds))
            i += 1

        # And plot the track data for the entire datafile
        plt.scatter(data['LOCAL_POSITION_NED.y'][1,:], data['LOCAL_POSITION_NED.x'][1,:])
        plt.axis('equal')
        plt.title("Vehicle track for datafile '{}'".format(csv_file))
        plt.ylabel('North (m)')
        plt.xlabel('East (m)')
        plt.show()

