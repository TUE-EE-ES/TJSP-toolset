from library.optimization_problems.tfjssp import *
from library.optimization_problems.fjssp import *
from library.optimization_problems.tjssp import *
from library.optimization_problems.cfr import *
from library.decompositions.fjssp_decomposition import *
from library.decompositions.cftfjssp_decomposition import *

import os


filepath = os.path.dirname(os.path.abspath(__file__)) + '/data/'
filename = 'CFR_default.data'
model_data = ModelData(filepath, filename, ModelType.CFR)

model = CFR(
    "CFR",
    model_data,
    ProblemType.FEASIBILITY_PROBLEM
)

model.setup_model(
    [Transfer(endNode=13, startingTime=0, deadlineTime=3), Transfer(endNode=6, startingTime=3, deadlineTime=6), Transfer(endNode=16, startingTime=6, deadlineTime=10)],
    [Transfer(endNode=4, startingTime=0, deadlineTime=3), Transfer(endNode=11, startingTime=3, deadlineTime=6), Transfer(endNode=13, startingTime=6, deadlineTime=10)])
model.solve(print_output = True)

model.print_figure()
