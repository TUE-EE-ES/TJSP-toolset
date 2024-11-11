from library.optimization_problems.tfjssp import *
from library.optimization_problems.fjssp import *
from library.optimization_problems.tfjssp2 import *
from library.optimization_problems.tjssp import *
from library.optimization_problems.cfr import *
from library.decompositions.fjssp_decomposition import *
from library.decompositions.cftfjssp_decomposition import *

import os


filepath = os.path.dirname(os.path.abspath(__file__)) + '/data/'
filename = 'TFJSSP_default.data'
model_data = ModelData(filepath, filename, ModelType.TFJSSP)

model = TFJSSP(
    "TFJSSP",
    model_data,
)

model.setup_model()

model.solve(print_output = True)

model.print_figure()