from abc import ABC, abstractmethod
import sys

import cplex
from docplex.cp.model import *
from docplex.mp.model import *
import docplex.cp.parameters as params
from .model_data import *
from dataclasses import dataclass
import docplex.mp.conflict_refiner as cr
import matplotlib.pyplot as plt
import signal


@dataclass
class Transfer:
    endNode: int
    startingTime: int
    deadlineTime: int


class MIPDecisionVariables:
    def __init__(self):
        self.x = None
        self.c = None


class CPDecisionVariables:
    def __init__(self):
        self.operations = None
        self.machine_operation_options = None
        self.transfers = None
        self.vehicle_transfer_options = None
        self.load = None
        self.vehicle_sequence = None


class ProblemType(Enum):
    OPTIMIZATION_PROBLEM = 1
    FEASIBILITY_PROBLEM = 2


class OptimizationModel(ABC):
    def __init__(self, model_data, problem_type):
        self.problem_type = problem_type
        self.decision_vars = None
        self.model = None
        self.res = None

        if not isinstance(model_data, ModelData):
            raise TypeError('model_data must be an instance of ModelData')

        self.MODEL_DATA = model_data

    @abstractmethod
    def setup_model(self, *args):
        pass

    @abstractmethod
    def solve(self, print_output, timeout=900):
        pass

    def print_figure(self):
        visu = self.__build_figure()

        visu.show()

    def save_figure(self, destination, filename):
        visu = self.__build_figure()

        with Show(destination + filename + ".pdf"):
            visu.show()

    @abstractmethod
    def __build_figure(self):
        pass

    @abstractmethod
    def get_objective(self):
        pass

    @abstractmethod
    def write_solution(self, destination, filename):
        pass


class MIPOptimizationModel(OptimizationModel, ABC):
    def __init__(self, model_name, model_data, problem_type):
        super().__init__(model_data, problem_type)

        self.model = Model(model_name)
        self.decision_vars = MIPDecisionVariables
        self.cgroup = cr.ConstraintsGroup(1)

    def solve(self, print_output, timeout=900):
        self.model.set_time_limit(timeout)

        self.res = self.model.solve(log_output=print_output)
        if print_output:
            self.model.print_information()
            print(str(self.model.solve_details))

            print(self.res)

        self.model.set_time_limit(180)
        cref = cr.ConflictRefiner()
        print('Conflict refining')

        self.crefres = cref.refine_conflict(self.model, display=False)  # display flag is to show the conflicts
        self.model.set_time_limit(1000000000000000000000000000000000000000000000000000000)

        # if print_output:
        self.crefres.display()
        # print(crefres.refined_by)
        pass

    def get_objective(self):
        if hasattr(self.model, 'solve_details'):
            if self.model.solve_details is not None:
                return self.model.solve_details.status
            else:
                return ''
        else:
            return ''

    def getTransfers(self, vehicle_transfers):
        transfers = {}

        for v, T in enumerate(vehicle_transfers):
            for k, t in enumerate(T):
                transfers[(v, k)] = t

        return transfers

    def getTimeHorizon(self, vehicle_transfers):
        deadlines = []
        for T in vehicle_transfers:
            for t in T:
                if t != []:
                    deadlines.append(t.deadlineTime)

        return max(deadlines)

    def write_solution(self, destination, filename):
        f = open(destination + filename + ".txt", "w")
        f.write(str(self.res))
        f.close()




class CPOptimizationModel(OptimizationModel, ABC):
    def __init__(self, model_name, model_data, problem_type):
        super().__init__(model_data, problem_type)

        self.solver = None
        self.model = CpoModel(model_name)  # be aware maybe we need to build the model multiple times
        self.decision_vars = CPDecisionVariables()

        match model_data.MODEL_TYPE:
            case ModelType.TJSSP:
                decision_vars = ("operations",
                                 "transfers",
                                 "vehicle_transfer_options",
                                 "vehicle_sequence")
            case ModelType.TFJSSP:
                decision_vars = ("operations",
                                 "machine_operation_options",
                                 "transfers",
                                 # "load",
                                 "vehicle_transfer_options2",
                                 "vehicle_sequence")
            case ModelType.TFJSSP2:
                decision_vars = ("operations",
                                 "machine_operation_options",
                                 "transfers",
                                 # "load",
                                 "vehicle_transfer_options2",
                                 "vehicle_sequence")
            case ModelType.CFTFJSSP:
                decision_vars = ("operations",
                                 "machine_operation_options",
                                 "transfers",
                                 "load",
                                 "vehicle_transfer_options",
                                 "vehicle_sequence")
            case ModelType.CFTJSSP:
                decision_vars = ("operations",
                                 "transfers",
                                 "vehicle_transfer_options",
                                 "vehicle_sequence")
            case ModelType.FJSSP:
                decision_vars = ("operations",
                                 "machine_operation_options")

        for key in decision_vars:
            match key:
                case "operations":
                    self.decision_vars.operations = {
                        (j, o): interval_var(name='O_J{}_O{}'.format(j, o))
                        for j, J in enumerate(model_data.JOBS)
                        for o, O in enumerate(J)
                    }

                case "machine_operation_options":
                    if model_data.MODEL_TYPE == ModelType.TFJSSP2 or model_data.MODEL_TYPE == ModelType.TFJSSP:
                        self.decision_vars.machine_operation_options = {
                            (j, o, m): interval_var(name='MO_J{}_O{}_M{}'.format(j, o, m), optional=True, size=d)
                            for j, J in enumerate(model_data.JOBS)
                            for o, O in enumerate(J)
                            for k, (m, d) in enumerate(O)
                        }
                    else:
                        self.decision_vars.machine_operation_options = {
                            (j, o, k, m): interval_var(name='MO_J{}_O{}_C{}_M{}'.format(j, o, k, m), optional=True)
                            for j, J in enumerate(model_data.JOBS)
                            for o, O in enumerate(J)
                            for k, (m, d) in enumerate(O)
                        }
                case "transfers":
                    # if model_data.MODEL_TYPE == ModelType.TFJSSP:
                    #     self.decision_vars.transfers = {
                    #         (j, o): interval_var(name='T_J{}_O{}'.format(j, o))
                    #         for j in range(model_data.NR_JOBS)
                    #         for o in range(model_data.JOB_SIZES[j])
                    #     }
                    # else:
                    self.decision_vars.transfers = {
                            (j, o): interval_var(name='T_J{}_O{}'.format(j, o))
                            for j in range(model_data.NR_JOBS)
                            for o in range(model_data.JOB_SIZES[j] - 1)
                    }
                case "vehicle_transfer_options":
                    self.decision_vars.vehicle_transfer_options = {
                        (j, k, v): interval_var(name='J{}_K{}_V{}'.format(j, k, v), optional=True)
                        for j in range(model_data.NR_JOBS)
                        for k in range(model_data.JOB_SIZES[j] - 1)
                        for v in range(model_data.NR_VEHICLES)
                    }
                case "vehicle_transfer_options2":
                    # Deal with the last transfers
                    self.decision_vars.vehicle_transfer_options = {
                        (j, k, v, ms, me): interval_var(
                            name='TO_J{}_K{}_V{}_M{}-{}'.format(j, k, v, ms, me),
                            optional=True,
                        )
                        for j in range(model_data.NR_JOBS)
                        for k in range(model_data.JOB_SIZES[j] - 1)
                        for v in range(model_data.NR_VEHICLES)
                        for ms in model_data.MACHINE_JOBS[j][k]
                        for me in model_data.MACHINE_JOBS[j][k + 1]
                    }
                case "load":
                    self.decision_vars.load = {
                        (j, k): integer_var(name='L_J{}_K{}'.format(j, k))
                        for j in range(model_data.NR_JOBS)
                        for k in range(model_data.JOB_SIZES[j] - 1)
                    }
                case "vehicle_sequence":
                    if self.decision_vars.vehicle_transfer_options is None:
                        self.__init__(model_name, model_data, "vehicle_transfer_options")

                    if model_data.MODEL_TYPE == ModelType.TJSSP or model_data.MODEL_TYPE == ModelType.CFTJSSP:
                        self.decision_vars.vehicle_sequence = [
                            sequence_var([
                                self.decision_vars.vehicle_transfer_options[a]
                                for a in self.decision_vars.vehicle_transfer_options if a[2] == v
                            ], types=[
                                self.MODEL_DATA.JOBS[a[0]][a[1] + 1][0][0]
                                for a in self.decision_vars.vehicle_transfer_options if a[2] == v
                            ], name='S_V{}'.format(v))
                            for v in range(model_data.NR_VEHICLES)
                        ]
                    elif model_data.MODEL_TYPE == ModelType.TFJSSP2 or model_data.MODEL_TYPE == ModelType.TFJSSP:
                        self.decision_vars.vehicle_sequence = [
                            sequence_var([
                                self.decision_vars.vehicle_transfer_options[a]
                                for a in self.decision_vars.vehicle_transfer_options if a[2] == v
                            ], types=[a[3] * self.MODEL_DATA.NR_MACHINES + a[4]
                                      for a in self.decision_vars.vehicle_transfer_options if a[2] == v
                                      ], name='S_V{}'.format(v))
                            for v in range(model_data.NR_VEHICLES)
                        ]
                    else:
                        self.decision_vars.vehicle_sequence = [
                            sequence_var([
                                self.decision_vars.vehicle_transfer_options[a]
                                for a in self.decision_vars.vehicle_transfer_options if a[2] == v
                            ], name='S_V{}'.format(v))
                            for v in range(model_data.NR_VEHICLES)
                        ]
                case _:
                    raise TypeError('invalid decision variable: ' + key)

    def solve(self, print_output, timeout=900):
        # Solve model
        if print_output:
            self.solver = CpoSolver(self.model, LogVerbosity='Verbose', TimeLimit=timeout)#, Workers = 1)#, execfile='/Applications/CPLEX_Studio2211/cpoptimizer/bin/arm64_osx/cpoptimizer')# TimeLimit=1800)#, Workers=8)
        else:
            # params.set_TimeLimit(60)
            self.solver = CpoSolver(self.model, LogVerbosity='Quiet', TimeLimit=timeout)#, Workers = 1)#, execfile='/Applications/CPLEX_Studio2211/cpoptimizer/bin/arm64_osx/cpoptimizer')#, TimeLimit=1800)#, Workers=1)

        print('Solving model...')
        self.res = self.solver.solve()  # execfile='/opt/ibm/ILOG/CPLEX_Studio2211/cpoptimizer/bin/x86-64_linux/cpoptimizer')

        # solver.refine_conflict().print_conflict()

        if print_output:
            print('Solution:')
            self.res.print_solution()
        pass

    def get_objective(self):
        if self.res.solution.objective_values is None:
            return None
        else:
            return self.res.solution.objective_values[0]

    def write_solution(self, destination, filename):
        self.res.write(destination + filename + ".txt")


def pairwise(iterable):
    "s -> (s0, s1), (s2, s3), (s4, s5), ..."
    a = iter(iterable)
    return zip(a, a)


def conjunctify(list, negate):
    newList = []
    for constA, constB in pairwise(list):
        if constA.get_type() != Type_Constraint and constB.get_type() != Type_Constraint:
            if negate:
                newList.append(logical_and(logical_not(constA), logical_not(constB)))
            else:
                newList.append(logical_and(constA, constB))

        elif constA.get_type() == Type_Constraint and constB.get_type() == Type_Constraint:
            break
        elif constA.get_type() == Type_Constraint:
            if negate:
                newList.append(logical_not(constB))
            else:
                newList.append(constB)

        elif constB.get_type() == Type_Constraint:
            if negate:
                newList.append(logical_not(constA))
            else:
                newList.append(constA)

    if (len(list) % 2) != 0:
        newList.append(list[-1])

    return newList


def disjunctify(list, negate):
    newList = []
    for constA, constB in pairwise(list):
        if constA.get_type() != Type_Constraint and constB.get_type() != Type_Constraint:
            if negate:
                newList.append(logical_or(logical_not(constA), logical_not(constB)))
            else:
                newList.append(logical_or(constA, constB))

        elif constA.get_type() == Type_Constraint and constB.get_type() == Type_Constraint:
            break
        elif constA.get_type() == Type_Constraint:
            if negate:
                newList.append(logical_not(constB))
            else:
                newList.append(constB)

        elif constB.get_type() == Type_Constraint:
            if negate:
                newList.append(logical_not(constA))
            else:
                newList.append(constA)

    if (len(list) % 2) != 0:
        newList.append(list[-1])

    return newList

class Show:
    '''Simple context manager to temporarily reroute plt.show().

    This context manager temporarily reroutes the plt.show() function to
    plt.savefig() in order to save the figure to the file specified in the
    constructor rather than displaying it on the screen.'''
    def __init__(self, name):
        self._name = name
        self._orig = None
    def _save(self):
        plt.savefig(self._name)
        if False:
            # Here we could show the figure as well
            self._orig()
    def __enter__(self):
        self._orig = plt.show
        plt.show = lambda: self._save()
        return self
    def __exit__(self, type, value, traceback):
        if self._orig is not None:
            plt.show = self._orig
            self._orig = None