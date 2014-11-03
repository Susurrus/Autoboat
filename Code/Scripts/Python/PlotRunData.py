# This file plots the desired data from a given autonomous run.

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import sys

NaN = float('NaN')

if __name__ == '__main__':
    # Read in the program arguments from the command line
    csv_file = sys.argv[1]
    auto_run = int(sys.argv[2])
    data_name = sys.argv[3]
    
    # Load the file and check what column we'll be processing
    data = {}
    with open(csv_file, 'r') as f:
        # Assume the first line are CSVs of column headers.
        column_names = f.readline().split(',')
        column_names[-1] = column_names[-1][:-1]
        assert column_names[0] == 'timestamp', "Expected timestamp to be the first column header. Aborting."
        for col_name in column_names:
            data[col_name] = [[], []] # Save a timestamp/data column for each data

        print(list(data.keys()))
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
    # FIXME
    try:
        pass
    except:
        print(data_name, columns[1])
        print("ERROR: Column not found in CSV file")
        sys.exit(1)

    # And convert all data vectors to numpy arrays to simplify usage
    for key in data.keys():
        data[key] = np.array(data[key])

    # Preprocess the data
    # Subtract off the timestamp and convert to seconds
    plt.scatter(data['LOCAL_POSITION_NED.y'][1,:], data['LOCAL_POSITION_NED.x'][1,:])
    plt.axis('equal')
    plt.show()
