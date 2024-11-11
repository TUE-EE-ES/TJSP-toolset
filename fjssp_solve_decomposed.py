from library.optimization_problems.tfjssp import *
from library.optimization_problems.fjssp import *
from library.optimization_problems.tjssp import *
from library.optimization_problems.cfr import *
from library.decompositions.fjssp_decomposition import *

import os
import time

filepath = os.path.dirname(os.path.abspath(__file__)) + '/data/'
filename = 'FJSSP_default.data'
model_data = ModelData(filepath, filename, ModelType.FJSSP)

benders = DecompositionFJSSP("FJSSP", model_data)

benders.set_iterations(1000)

start = time.time()

benders.solve(True)

# Calculate the end time and time taken
end = time.time()
length = end - start

# Show the results : this can be altered however you like
print("Finished in", length, "seconds!")

benders.save_figure("","fjssp")
benders.print_figure()