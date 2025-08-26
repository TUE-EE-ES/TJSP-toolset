from library.optimization_model import *
from enum import Enum
import time
class LogicBasedBendersDecomposition(ABC):
    def __init__(self, name, master_problem, sub_problem):
        """
        Initialize Logic-Based Bender Decomposition (LBBD) variables
        """
        self.name = name
        self.benders_cuts = []
        self.print_output = False
        self.master_problem_objective_value_list = []

        self.sub_problem_optimal_result = None
        self.sub_problem_optimal_value = None

        self.master_problem_extract = None

        self.res = None
        self.iteration = None

        self.iterations = 100 # Set default number of iterations


        if not isinstance(master_problem, OptimizationModel):
            raise TypeError('Master problem must be an instance of OptimizationModel')
        if not isinstance(sub_problem, OptimizationModel) and not isinstance(sub_problem, LogicBasedBendersDecomposition):
            raise TypeError('Sub-problem must be an instance of OptimizationModel or BendersDecomposition')

        self.master_problem = master_problem
        self.sub_problem = sub_problem

    def set_iterations(self, iterations):
        """
        Specify max number of iterations before solving terminates
        """
        self.iterations = iterations

    def solve(self, print_output, timeout=3600*5):
        """
        Solves the LBBD procedure
        """
        abort = False
        end_iteration = None
        self.print_output = print_output
        start_time = time.time()
        # Start iterative procedure
        for iteration in range(self.iterations):
            self.iteration = iteration

            # Initialize new master and sub problem ILOG Models
            self.master_problem.model = CpoModel(self.master_problem.model.name)

            # TODO: Accept other LBBD's
            if isinstance(self.sub_problem.model, Model):
                self.sub_problem.model = Model(self.sub_problem.model.name)
            else:
                self.sub_problem.model = CpoModel(self.sub_problem.model.name)

            self.master_problem.setup_model()

            if print_output:
                print(
                    "________________________________________________________ "
                    + "BENDERS "
                    + self.name
                    + " ITERATION "
                    + str(iteration)
                    + " ________________________________________________________"
                )

                print("Adding cuts to " + self.name + "...")

            # Add all benders cuts (in first iteration no benders cuts can be added)
            for cuts in self.benders_cuts:
                self.master_problem.model.add(cuts)

            if print_output:
                print(
                    "Master "
                    + self.name
                    + " ITERATION "
                    + str(iteration)
                    + " ________________________________________________________"
                )

            # Solve master problem
            self.master_problem.solve(print_output = print_output, timeout = min(900, max(10, timeout - (time.time() - start_time))))

            # Save results (uncomment if you want to store solutions of each iteration)
            # self.master_problem.write_solution('benders/benders-new/test/', 'iteration' + str(iteration))

            # Obtain optimal master problem objective value at each iteration.
            if self.master_problem.get_objective() is not None:
                self.master_problem_objective_value_list.append(self.master_problem.get_objective())
            else:
                self.master_problem_objective_value_list.append(0)

            # Check whether bound converged to optimum and terminate algorithm
            if self.__check_completion(iteration):
                end_iteration = iteration
                print("LBBD Complete")
                break

            # Extract trail values
            self.__extract_master_problem_results()

            # Setup sub problem with trial values
            if isinstance(self.sub_problem, OptimizationModel):
                self.sub_problem.setup_model(*self.master_problem_extract)

            # Run sub problem or else another decomposition
            if isinstance(self.sub_problem, OptimizationModel):
                if print_output:
                    print(
                        "Sub "
                        + self.name
                        + " ITERATION "
                        + str(iteration)
                        + " ________________________________________________________"
                    )

                self.sub_problem.solve(print_output, timeout = max(10, (timeout - (time.time() - start_time))))
            elif isinstance(self.sub_problem, LogicBasedBendersDecomposition):
                self.sub_problem.solve(print_output, timeout = max(10, (timeout - (time.time() - start_time))))

            # Proceed algorithm for either a feasibility sub problem or optimality sub problem
            if self.sub_problem.problem_type is ProblemType.FEASIBILITY_PROBLEM:
                if self.sub_problem.get_objective() == 'integer infeasible':
                    self.__apply_cuts()
            else:
                if (self.sub_problem_optimal_value is None or
                        self.sub_problem.get_objective() < self.sub_problem_optimal_value):

                    self.sub_problem_optimal_value = self.sub_problem.get_objective()
                    self.sub_problem_optimal_result = self.sub_problem.res

                if self.master_problem_objective_value_list[iteration] < self.sub_problem_optimal_value:
                    self.__apply_cuts()

            if timeout - (time.time() - start_time) < 1:
                abort = True

            # Check again for completion
            if self.__check_completion(iteration) or abort:
                end_iteration = iteration
                print("LBBD Complete")
                break

        # Store LBBD result
        try:
            if self.master_problem_objective_value_list[end_iteration] is not None:
                if self.sub_problem.problem_type is ProblemType.OPTIMIZATION_PROBLEM:
                    self.sub_problem.res = self.sub_problem_optimal_result
                    self.res =  self.sub_problem.res
                else:
                    self.res =  self.master_problem.res
        except:
            print("Did not find any solution")


    def __check_completion(self, i):
        """
        Check if LBBD procedure can finish
        """
        if isinstance(self.sub_problem, OptimizationModel):
            if self.sub_problem.problem_type is ProblemType.OPTIMIZATION_PROBLEM:
                return self.master_problem_objective_value_list[i] == self.sub_problem_optimal_value
            elif self.sub_problem.problem_type is ProblemType.FEASIBILITY_PROBLEM:
                print(self.sub_problem.get_objective())
                return self.sub_problem.get_objective() == 'integer optimal solution'


    @abstractmethod
    def __extract_master_problem_results(self):
        """
         Extracts the relevant trail values from the mater problem solution results
        """
        pass

    @abstractmethod
    def __apply_cuts(self):
        """
        Applies logic based benders cuts
        """
        pass

    def print_figure(self):
        """
        Prints figure of solution
        """
        visu = self.__build_figure()

        visu.show()

    def save_figure(self, destination, filename):
        """
        Saves figure of solution at /destination/filename.pdf
        """
        visu = self.__build_figure()

        with Show(destination + filename + ".pdf"):
            visu.show()

    @abstractmethod
    def __build_figure(self):
        """
        Creates a figure of solution
        """
        pass

    def get_objective(self):
        """
        Returns the objective value of master problem solution
        """
        return self.master_problem.get_objective()
        pass

    def write_solution(self, destination, filename):
        """
        Writes a textual solution to /destination/filename(_master/_sub).txt
        """
        self.master_problem.write_solution(destination, filename + '_master')
        self.sub_problem.write_solution(destination, filename + '_sub')
