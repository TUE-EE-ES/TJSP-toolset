import docplex.cp.utils_visu as visu

from docplex.cp.model import *
from library.optimization_model import *


class FJSSP(CPOptimizationModel):
    """
    CP Optimizer Model of the FJSSP problem
    """

    def setup_model(self):
        """
        Sets up CP model for the FJSSP problem
        """

        # Operation size
        for j in range(self.MODEL_DATA.NR_JOBS):
            for k in range(self.MODEL_DATA.JOB_SIZES[j]):
                for i, (m, d) in enumerate(self.MODEL_DATA.JOBS[j][k]):
                    self.model.add(if_then(
                        presence_of(self.decision_vars.machine_operation_options[(j,k,i,m)]),
                        size_of(self.decision_vars.operations[(j, k)]) >= d)
                    )

        # Precedence constraints between operations of a job
        self.model.add(
            end_before_start(self.decision_vars.operations[(j, o)], self.decision_vars.operations[(j, o + 1)])
            for j, o in self.decision_vars.operations if o < self.MODEL_DATA.JOB_SIZES[j] - 1
        )

        # Alternative constraints
        self.model.add(
            alternative(
                self.decision_vars.operations[(j, o)], [
                    self.decision_vars.machine_operation_options[a]
                    for a in self.decision_vars.machine_operation_options if a[0:2] == (j, o)
                ])
            for j, o in self.decision_vars.operations
        )

        # Add no_overlap constraint between operations executed on the same machine
        self.model.add(
            no_overlap(
                self.decision_vars.machine_operation_options[a]
                for a in self.decision_vars.machine_operation_options if a[3] == m)
            for m in range(self.MODEL_DATA.NR_MACHINES)
        )

        # Minimize termination date
        self.model.add(
            minimize(max(end_of(self.decision_vars.operations[(j, o)]) for j, o in self.decision_vars.operations))
        )
        pass

    def _OptimizationModel__build_figure(self):
        """
        Creates a figure of the problem solution
        """
        if self.res and visu.is_visu_enabled():
            # Draw solution
            visu.timeline('Solution for flexible job-shop ' + self.MODEL_DATA.FILENAME)
            visu.panel('Machines')
            for m in range(self.MODEL_DATA.NR_MACHINES):
                visu.sequence(name='M' + str(m))
                for a in self.decision_vars.machine_operation_options:
                    if a[3] == m:
                        itv = self.res.get_var_solution(self.decision_vars.machine_operation_options[a])
                        if itv.is_present():
                            visu.interval(itv, a[0], 'J{}'.format(a[0]))
            return visu
