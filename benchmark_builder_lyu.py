'''
Benchmark builder for Lyu2019.
'''

from library.benchmark import *

import os

jobs = os.path.dirname(os.path.abspath(__file__)) + '/data/benchmarks/lyu2019/jobs/'
layouts = os.path.dirname(os.path.abspath(__file__)) + '/data/benchmarks/lyu2019/layouts/'
results = os.path.dirname(os.path.abspath(__file__)) + '/lyu2019/'
os.makedirs(os.path.dirname(results), exist_ok=True)

build_TFMS_benchmark(jobs, layouts, [1, 2, 3, 4, 5, 6, 7], results)
