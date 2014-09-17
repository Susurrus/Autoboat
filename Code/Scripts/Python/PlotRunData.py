# This file plots the desired data from a given autonomous run.

import numpy as np
import matplotlib.pyplot as plt
import sys

NaN = float('NaN')

if __name__ == '__main__':
    # Read in the program arguments from the command line
    csv_file = sys.argv[1]
    auto_run = int(sys.argv[2])
    data_name = sys.argv[3]
    
    # Load the file and check what column we'll be processing
    with open(csv_file, 'r') as f:
        columns = f.readline().split(',')
        columns[-1] = columns[-1][:-1]
        
        # Make sure we have a valid data column
        try:
            data_index = columns.index(data_name)
            timestamp_index = columns.index('timestamp')
            base_mode_index = columns.index('HEARTBEAT.base_mode')
            primary_errors_index = columns.index('NODE_STATUS.primary_errors')
        except:
            print(data_name, columns[1])
            print("ERROR: Column not found in CSV file")
            sys.exit(1)

        # Grab this columns data over every timestep
        data = []
        line = f.readline()
        while line:
            # Log this line's data
            line_data = line.split(',')
            timestamp = float(line_data[timestamp_index])
            try:
                base_mode = float(line_data[base_mode_index])
            except:
                base_mode = NaN
            try:
                primary_errors = float(line_data[primary_errors_index])
            except:
                primary_errors = NaN
            try:
                desired_data = float(line_data[data_index])
            except:
                desired_data = NaN
            pulled_data = (timestamp, base_mode, primary_errors, desired_data)
            data.append(pulled_data)
            
            # And load the next line
            line = f.readline()
        
        # Numpy-ify it and plot it
        data = np.array(data)
        
        # Preprocess the data
        # Subtract off the timestamp and convert to seconds
        data[:,0] = data[:,0] - data[0][0]
        plt.scatter(range(len(data[:,0])), data[:,3])
        plt.show()