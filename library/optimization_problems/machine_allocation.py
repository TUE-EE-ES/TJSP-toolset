import docplex.cp.utils_visu as visu

from docplex.cp.model import *
from library.optimization_model import *


class MachineAllocation(CPOptimizationModel):
    """
    CP Optimizer Model of a machine allocation problem
    """

    def setup_model(self):
        """
        Sets up CP model for a machine allocation problem
        """
        # Alternative machines
        self.model.add(
            alternative(
                self.decision_vars.operations[j, o],
                [
                    self.decision_vars.machine_operation_options[a]
                    for a in self.decision_vars.machine_operation_options
                    if a[0:2] == (j, o)
                ]
            )
            for j, o in self.decision_vars.operations
        )

        # Operation size
        for j in range(self.MODEL_DATA.NR_JOBS):
            for k in range(self.MODEL_DATA.JOB_SIZES[j]):
                for i, (m, d) in enumerate(self.MODEL_DATA.JOBS[j][k]):
                    self.model.add(if_then(
                        presence_of(self.decision_vars.machine_operation_options[(j,k,i,m)]),
                        size_of(self.decision_vars.operations[(j, k)]) >= d)
                    )

        # Minimize termination date
        self.model.add(minimize(max(
            end_of(self.decision_vars.operations[j, o]) for j, o in self.decision_vars.operations))
        )

        pass

    def _OptimizationModel__build_figure(self):
        """
        Creates a figure of the problem solution
        """
        return visu