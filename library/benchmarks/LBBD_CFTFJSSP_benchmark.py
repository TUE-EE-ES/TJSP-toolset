from library.benchmark import *
from library.decompositions.cftfjssp_decomposition import *

class LBBD_CFTFJSSP_Benchmark(TCFMSBenchmark):

    def run(self):
        filepath = self.results_folder + 'experiments/'
        for i, filename in enumerate(os.listdir(filepath)):

            model_data = ModelData(filepath, filename, ModelType.CFTFJSSP)

            benders = DecompositionCFTFJSSP("CFTJSSP", model_data)

            start = time.time()
            benders.solve(100)
            # Calculate the end time and time taken
            end = time.time()
            length = end - start

            # Show the results : this can be altered however you like
            print("Finished in", length, "seconds!")

