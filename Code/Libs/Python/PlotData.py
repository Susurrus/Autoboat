# This file plots the data for given field(s) from .CSV files of recorded runs. With multiple fields specified, they all get plotted together.
# Additionally, if the first argument is a number, it will plot this data for only that autonomous section

import numpy as np
import matplotlib.pyplot as plt
import sys

from helpers import GetDataForFields, GetTimespanForRun

if __name__ == '__main__':

    if len(sys.argv) < 3:
        print("ERROR: Arguments required - CSV file, field_name1[,field_name2] (MESSAGE_TYPE.FIELD_NAME), [autonomous run number]")
        sys.exit(1)

    # Read in the program arguments from the command line
    csv_file = sys.argv[1]
    field_names = sys.argv[2].split(',')
    if len(sys.argv) > 3:
        auto_run = int(sys.argv[3])
    else:
        auto_run = None

    # Get all the data from the CSV file
    data = GetDataForFields(csv_file)

    # Check that the user provided a valid field name
    for f in field_names:
        if f not in data:
            print("ERROR: Invalid field name '{}'. '{}' only contains the following fields: {}".format(f, csv_file, ','.join(data.keys())))
            sys.exit(1)

    # Now plot the data, showing the whole dataset if no autonomous run number was specified
    if auto_run:
        # Determine the range of values to plot
        start_time, end_time = GetTimespanForRun(data, auto_run)
        plt.figure()
        for f in field_names:
            valid_indices = np.logical_and(data[f][0] >= start_time, data[f][0] <= end_time)

            plt.plot(data[f][0,valid_indices] - data[f][0,0], data[f][1,valid_indices])
        plt.title('{} for auto run {}'.format(','.join(field_names), auto_run))
        plt.xlabel('Time (s)')
        if len(field_names) > 1:
            plt.legend(field_names)
        plt.show()
    else:
        plt.figure()
        for f in field_names:
            plt.plot(data[f][0,:] - data[f][0,0], data[f][1,:])
        plt.title("'{}' in '{}'".format(', '.join(field_names), csvfile))
        plt.xlabel('Time (s)')
        if len(field_names) > 1:
            plt.legend(field_names)
        plt.show()

    # Preprocess the data
    # Subtract off the timestamp and convert to seconds
#    plt.scatter(data['LOCAL_POSITION_NED.y'][1,:], data['LOCAL_POSITION_NED.x'][1,:])
#    plt.axis('equal')
#    plt.show()
