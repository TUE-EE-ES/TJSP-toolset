'''
Benchmark builder for Liu2023.
'''

from library.benchmark import *

import os

jobs = os.path.dirname(os.path.abspath(__file__)) + '/data/benchmarks/liu2023/jobs/'
layouts = os.path.dirname(os.path.abspath(__file__)) + '/data/benchmarks/liu2023/layouts/'
results = os.path.dirname(os.path.abspath(__file__)) + '/liu2023/'
os.makedirs(os.path.dirname(results), exist_ok=True)

build_TFMS_benchmark(jobs, layouts, [2, 3, 4], results)
