import csv

from .benders_decomposition import *

import time
import re

def build_TFMS_benchmark(jobs_folder, layout_folder, vehicle_list, destination_folder):
    """
    Builds a benchmark for a given job set, shop layout, and a set of vehicles.
    """
    os.makedirs(os.path.dirname(destination_folder + 'experiments/'), exist_ok=True)

    if not os.listdir(destination_folder + 'experiments/'):
        for i, folderame1 in enumerate(os.listdir(jobs_folder)):
            for m, filename1 in enumerate(os.listdir(jobs_folder + folderame1 + '/')):
                for j, foldername2 in enumerate(os.listdir(layout_folder)):
                    for n, filename2 in enumerate(os.listdir(layout_folder + foldername2 + '/')):
                        if folderame1 == foldername2:
                            for vehicle in vehicle_list:
                                # Reading data from file1
                                with open(jobs_folder + folderame1 + '/' + filename1) as fp:
                                    data = fp.read()

                                # Reading data from file2
                                with open(layout_folder + foldername2 + '/' + filename2) as fp:
                                    data2 = fp.read()

                                # Adding the amount of vehicles to the job data
                                enter = re.search('\n', data)
                                enter = enter.start()
                                data = list(data)
                                data.insert(enter, ' ')
                                data.insert(enter + 1, str(vehicle))

                                # Merging 2 files
                                # To add the data of file2
                                # from next line
                                data = "".join(data)
                                data += "\n"

                                data += data2

                                os.makedirs(os.path.dirname(destination_folder + 'experiments/'), exist_ok=True)

                                experiment_name = ('EX'
                                                   + str(os.path.splitext(filename1)[0])
                                                   + str(os.path.splitext(filename2)[0])
                                                   )

                                if len(vehicle_list) > 1:
                                    experiment_name += '-' + str(vehicle)

                                with open(destination_folder + 'experiments/' + experiment_name + '.data', 'w') as fp:
                                    fp.write(data)
    else:
        raise TypeError('Destination folder must be empty')


class TFMSBenchmark(ABC):
    def __init__(self, model, model_type, destination_folder):
        self.model = model
        self.model_type = model_type
        self.destination_folder = destination_folder
        self.experiments_folder = destination_folder + 'experiments/'

    # @abstractmethod
    def run(self):

        result_data = [[]]

        for i, filename in enumerate(os.listdir(self.experiments_folder)):


            result_data.append([])

            model_data = ModelData(self.experiments_folder, filename, self.model_type)

            model = self.model(filename, model_data)
            print("solving " + os.path.splitext(filename)[0])

            # Calculate the start time and time taken
            start = time.time()

            if isinstance(model, OptimizationModel):
                model.setup_model()

            if isinstance(model, LogicBasedBendersDecomposition):
                model.set_iterations(1000)

            try:
                model.solve(False)
            except:
                print("Failed after " + str(start) + " seconds")

            # Calculate the end time and time taken
            end = time.time()
            length = end - start

            # Inittiialize headers
            if i == 0:
                result_data[i].append('EX')

                if hasattr(model_data, 'NR_JOBS'):
                    result_data[i].append('NR_JOBS')
                if hasattr(model_data, 'NR_MACHINES'):
                    result_data[i].append('NR_MACHINES')
                if hasattr(model_data, 'NR_VEHICLES'):
                    result_data[i].append('NR_VEHICLES')
                if hasattr(model_data, 'NR_ROWS'):
                    result_data[i].append('GRID SIZE')

                result_data[i].append('COMPUTATION TIME')
                result_data[i].append('SOLVE STATUS')
                result_data[i].append('OBJECTIVE')

            # Gather all relevant data:
            result_data[i+1].append(os.path.splitext(filename)[0])

            if hasattr(model_data, 'NR_JOBS'):
                result_data[i+1].append(model_data.NR_JOBS)
            if hasattr(model_data, 'NR_MACHINES'):
                result_data[i+1].append(model_data.NR_MACHINES)
            if hasattr(model_data, 'NR_VEHICLES'):
                result_data[i+1].append(model_data.NR_VEHICLES)
            if hasattr(model_data, 'NR_ROWS'):
                result_data[i+1].append(str(model_data.NR_ROWS) + 'x' + str(model_data.NR_COLS))

            result_data[i+1].append(length)

            # Check objective
            # if model.problem_type == ProblemType.OPTIMIZATION_PROBLEM:

            try:
                result_data[i + 1].append(model.res.solve_status)
            except:
                result_data[i + 1].append('Unknown')

            try:
                result_data[i + 1].append(model.get_objective())
            except:
                result_data[i + 1].append("Null")

            os.makedirs(os.path.dirname(self.destination_folder + 'resultsindividual/'), exist_ok=True)
            with open(self.destination_folder + 'resultsindividual/' + os.path.splitext(filename)[0] +'.csv' , mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerows(result_data)

            try:
                # Save results
                os.makedirs(os.path.dirname(self.destination_folder + 'solutions/'), exist_ok=True)
                model.write_solution(self.destination_folder + 'solutions/', os.path.splitext(filename)[0])

                # Write figures
                os.makedirs(os.path.dirname(self.destination_folder + 'figures/'), exist_ok=True)
                model.save_figure(self.destination_folder + 'figures/', os.path.splitext(filename)[0])
            except:
                print("Nothing to write")

        os.makedirs(os.path.dirname(self.destination_folder + 'result/'), exist_ok=True)
        with open(self.destination_folder + 'result/output.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(result_data)



