'''
Benchmark builder example.
'''

from library.benchmark import *

import os

path = os.path.dirname(os.path.abspath(__file__))
jobs = path + '/data/benchmarks/XXXXX/jobs/'
layouts = path + '/data/benchmarks/XXXXX/layouts/'
results = path + '/benchmark example/'
os.makedirs(os.path.dirname(results), exist_ok=True)

build_TFMS_benchmark(jobs, layouts, [1, 2, 3, 4], results)
