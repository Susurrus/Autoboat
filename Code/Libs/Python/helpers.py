# This file contains helper functions for data analysis.

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import sys

# Declare a not-a-number constant
NaN = float('NaN')

def GetDataForFields(csv_file, fields=[]):
    """
    Returns a dictionary of data indexed by field names.

    Each field data has 2 columns of floats: the timestamp and then the data.

    If field names are provided in the second argument, only the desired data is returned.
    """

    # Load the file and check what column we'll be processing
    data = {}
    with open(csv_file, 'r') as f:
        # Assume the first line are CSVs of column headers.
        column_names = f.readline().split(',')
        column_names[-1] = column_names[-1][:-1]
        assert column_names[0] == 'timestamp', "Expected timestamp to be the first column header. Aborting."
        for col_name in column_names:
            data[col_name] = [[], []] # Save a timestamp/data column for each data

        # Now extract all data
        line = f.readline()
        while line:
            # Find any valid data, and save the timestamp/value pair
            columns = f.readline().split(',')
            columns[-1] = columns[-1][:-1]
            for i in range(len(columns)):
                if columns[i]:
                    data[column_names[i]][0].append(float(columns[0]))
                    data[column_names[i]][1].append(float(columns[i]))

            # And read the next line
            line = f.readline()

    # Make sure we have a valid data columns
    for field in fields:
        if field not in column_names:
            print(data_name, columns[1])
            print("ERROR: Column not found in CSV file")
            sys.exit(1)

    # And convert all data vectors to numpy arrays to simplify usage
    for key in data.keys():
        data[key] = np.array(data[key])

    # Finally reshape the output array to only contain the desired data
    if fields:
        out_data = {}
        for field in fields:
            out_data[field] = data[field]
        return out_data
    else:
        return data

# Taken from http://www.astropython.org/snippet/2010/11/Interpolation-without-SciPy
def interpolate(xin, yin, xout, method='linear'):
    """
    Interpolate the curve defined by (xin, yin) at points xout. The array
    xin must be monotonically increasing. The output has the same data type as
    the input yin.

    :param yin: y values of input curve
    :param xin: x values of input curve
    :param xout: x values of output interpolated curve
    :param method: interpolation method ('linear' | 'nearest')

    @:rtype: numpy array with interpolated curve
    """
    lenxin = len(xin)

    i1 = np.searchsorted(xin, xout)
    i1[ i1==0 ] = 1
    i1[ i1==lenxin ] = lenxin-1

    x0 = xin[i1-1]
    x1 = xin[i1]
    y0 = yin[i1-1]
    y1 = yin[i1]

    if method == 'linear':
        return (xout - x0) / (x1 - x0) * (y1 - y0) + y0
    elif method == 'nearest':
        return np.where(np.abs(xout - x0) < np.abs(xout - x1), y0, y1)
    else:
        raise ValueError('Invalid interpolation method: %s' % method)# Python code here

def GetAutonomousTimespans(data):
    """Returns the first and last time for the desired autonomous run section."""

    # And then get the timesteps that the vessel is autonomous, but under secondary manual control
    # And manual override from NODE_STATUS @ 1Hz for the primary node.
    manual_override = np.bool_(np.int_(data['NODE_STATUS.primary_errors'][1]) & 0x10)
    auto_mode = np.bool_(np.int_(data['NODE_STATUS.primary_status'][1]) & 0x1)
    override_time, oti = np.unique(data['NODE_STATUS.primary_errors'][0], True)
    manual_override = manual_override[oti]
    auto_mode = auto_mode[oti]

    # Now find all the times we're in autonomous mode and it's not being overridden
    automode = auto_mode & ~manual_override

    # Find the start and end times of each autonomous mode segment, returning them as a list of
    # (START_TIME, END_TIME) tuples
    timespan_endpoint_indices = np.bool_(np.diff(automode) != 0)
    timespans = override_time[timespan_endpoint_indices]
    timespans_out = []
    for i in range(0, len(timespans), 2):
        timespans_out.append((timespans[i], timespans[i+1]))

    return timespans_out

def GetTimespanForRun(data, auto_run):
    """Get the timespans for a specific autonomous run number."""
    timespans = GetAutonomousTimespans(data)
    return timespans[auto_run]
