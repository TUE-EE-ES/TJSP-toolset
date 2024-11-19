"""
Example running the LBBD-based CFTFJSSP
"""

from library.decompositions.cftfjssp_decomposition import *

import os
import time

# Specify location of data file
filepath = os.path.dirname(os.path.abspath(__file__)) + '/data/CFTFJSSP/'
filename = 'Lyu_EX53-3.data'

# Parse data file
model_data = ModelData(filepath, filename, ModelType.CFTFJSSP)

# Setup Logic-Based Benders Decomposition
benders = DecompositionCFTFJSSP("CFTJSSP", model_data)
benders.set_iterations(1000)

# Probe start time
start = time.time()

# Solve LBBD-based CFTFJSSP
benders.solve(print_output=True)

print(benders.get_objective())

# Calculate the end time and time taken
end = time.time()
length = end - start

# Show the results
print("Finished in", length, "seconds!")
benders.print_figure()
