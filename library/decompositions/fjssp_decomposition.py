import docplex.cp.utils_visu as visu

from docplex.cp.model import *
from library.benders_decomposition import *
from library.optimization_problems.fjssp import FJSSP
from library.optimization_problems.machine_allocation import MachineAllocation


class DecompositionFJSSP(LogicBasedBendersDecomposition):
    def __init__(self, name, model_data):
        self.MODEL_DATA = model_data
        master_problem = MachineAllocation(name + "_master", model_data, ProblemType.OPTIMIZATION_PROBLEM)
        sub_problem = FJSSP(name + "_sub", model_data, ProblemType.OPTIMIZATION_PROBLEM)

        super().__init__(name, master_problem, sub_problem)

    def _LogicBasedBendersDecomposition__extract_master_problem_results(self):
        for a in self.master_problem.decision_vars.machine_operation_options:
            itv = self.master_problem.res.get_var_solution(
                self.master_problem.decision_vars.machine_operation_options[a]
            )
            if itv.is_present():
                self.sub_problem.model.add(
                    presence_of(self.sub_problem.decision_vars.machine_operation_options[a])
                )
        self.master_problem_extract = []

    def _LogicBasedBendersDecomposition__apply_cuts(self):
        # for m in range(self.MODEL_DATA.NR_MACHINES):
        machine_makespan = []
        machine_allocation = []
        for a in self.sub_problem.decision_vars.machine_operation_options:
            itv = self.sub_problem.res.get_var_solution(
                self.sub_problem.decision_vars.machine_operation_options[a]
            )
            if itv.is_present():
                machine_makespan.append(itv.end)
                machine_allocation.append(presence_of(
                    self.master_problem.decision_vars.machine_operation_options[a]
                ))

        while len(machine_allocation) != 1:
            machine_allocation = conjunctify(machine_allocation, False)

        machine_makespan = max(machine_makespan)

        self.benders_cuts.append(
            if_then(
                machine_allocation[0],
                max(end_of(
                    self.master_problem.decision_vars.operations[(j,o)]
                ) for j, o in self.master_problem.decision_vars.operations) >= machine_makespan
            )
        )
        print("test")

    def _LogicBasedBendersDecomposition__build_figure(self):
        if self.res and visu.is_visu_enabled():
            # Draw solution
            visu.timeline('Solution for flexible job-shop ' + self.MODEL_DATA.FILENAME)
            visu.panel('Machines')
            for m in range(self.MODEL_DATA.NR_MACHINES):
                visu.sequence(name='M' + str(m))
                for a in self.sub_problem.decision_vars.machine_operation_options:
                    if a[3] == m:
                        itv = self.res.get_var_solution(self.sub_problem.decision_vars.machine_operation_options[a])
                        if itv.is_present():
                            visu.interval(itv, a[0], 'J{}'.format(a[0]))
            return visu
