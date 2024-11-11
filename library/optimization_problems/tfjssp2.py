from docplex.cp.model import *
import docplex.cp.utils_visu as visu

# sys.path.append('/benders/benders-new/library')
from library.optimization_model import *


class TFJSSP2(CPOptimizationModel):
    """
    An alternative CP Optimizer Model of the TFJSSP as master problem in a LBBD-based CFTFJSSP
    """

    def __init__(self, model_name, model_data):
        model_data.MODEL_TYPE = ModelType.TFJSSP2
        super().__init__(model_name, model_data, ProblemType.OPTIMIZATION_PROBLEM)

    def setup_model(self):
        """
        Sets up CP model for the TFJSSP problem
        """

        # ----------------------------------------------------------------------------------------------------
        # Alternative Machines constraints
        # ----------------------------------------------------------------------------------------------------
        alternative_machines = (
            alternative(
                self.decision_vars.operations[j, o],
                [
                    self.decision_vars.machine_operation_options[a]
                    for a in self.decision_vars.machine_operation_options if a[0:2] == (j, o)
                ]
            ) for j, o in self.decision_vars.operations
        )

        # ----------------------------------------------------------------------------------------------------
        # Alternative Vehicles constraints
        # ----------------------------------------------------------------------------------------------------
        alternative_vehicles = (
            alternative(
                self.decision_vars.transfers[j, o],
                [
                    self.decision_vars.vehicle_transfer_options[a]
                    for a in self.decision_vars.vehicle_transfer_options if a[0:2] == (j, o)
                ]
            ) for j, o in self.decision_vars.transfers
        )

        # ----------------------------------------------------------------------------------------------------
        # No overlap machines constraints
        # ----------------------------------------------------------------------------------------------------
        no_overlap_machines = (
            no_overlap(
                self.decision_vars.machine_operation_options[a]
                for a in self.decision_vars.machine_operation_options if a[2] == m
            ) for m in range(self.MODEL_DATA.NR_MACHINES) if m != 0 and (m != self.MODEL_DATA.NR_MACHINES-1 or self.MODEL_DATA.SINGLE_UL)
        )

        TRANSFER_TIMES_PAIRED = []

        for i in range(self.MODEL_DATA.NR_MACHINES * self.MODEL_DATA.NR_MACHINES):
            TRANSFER_TIMES_PAIRED.append([])
            for j in range(self.MODEL_DATA.NR_MACHINES * self.MODEL_DATA.NR_MACHINES):
                TRANSFER_TIMES_PAIRED[i].append(0)

        for a1 in range(self.MODEL_DATA.NR_MACHINES):
            for a2 in range(self.MODEL_DATA.NR_MACHINES):
                for b1 in range(self.MODEL_DATA.NR_MACHINES):
                    for b2 in range(self.MODEL_DATA.NR_MACHINES):
                        TRANSFER_TIMES_PAIRED[a1 * self.MODEL_DATA.NR_MACHINES + b1][a2 * self.MODEL_DATA.NR_MACHINES + b2] = self.MODEL_DATA.TRANSFER_TIMES[b1][a2]

        no_overlap_vehicles = (
            no_overlap(
                self.decision_vars.vehicle_sequence[m], TRANSFER_TIMES_PAIRED #, is_direct=True
            ) for m in range(self.MODEL_DATA.NR_VEHICLES)
        )

        # ----------------------------------------------------------------------------------------------------
        # Precedence constraints between operations of a job
        # ----------------------------------------------------------------------------------------------------
        precedence_operations = (
            end_before_start(
                self.decision_vars.operations[j, o], self.decision_vars.operations[j, o + 1]
            ) for j, o in self.decision_vars.operations if o < self.MODEL_DATA.JOB_SIZES[j] - 1
        )

        # ----------------------------------------------------------------------------------------------------
        # Scheduling constraints constraints
        # ----------------------------------------------------------------------------------------------------
        for j in range(self.MODEL_DATA.NR_JOBS):
            for k in range(self.MODEL_DATA.JOB_SIZES[j] - 1):
                self.model.add(
                    end_before_start(self.decision_vars.operations[(j, k)], self.decision_vars.transfers[(j, k)])
                )

                self.model.add(
                    end_before_start(self.decision_vars.transfers[(j, k)], self.decision_vars.operations[(j, k + 1)])
                )


        for j in range(self.MODEL_DATA.NR_JOBS):
            for k in range(self.MODEL_DATA.JOB_SIZES[j] - 1):
                for v in range(self.MODEL_DATA.NR_VEHICLES):
                    for ms in self.MODEL_DATA.MACHINE_JOBS[j][k]:
                        for me in self.MODEL_DATA.MACHINE_JOBS[j][k + 1]:
                            self.model.add(
                                if_then(
                                    presence_of(self.decision_vars.vehicle_transfer_options[(j, k, v, ms, me)]),
                                    logical_and(
                                        presence_of(self.decision_vars.machine_operation_options[(j, k, ms)]),
                                        presence_of(self.decision_vars.machine_operation_options[(j, k+1, me)])
                                    )
                                )
                            )


                            self.model.add(
                                if_then(
                                    presence_of(self.decision_vars.vehicle_transfer_options[(j, k, v, ms, me)]),
                                    size_of(self.decision_vars.vehicle_transfer_options[(j, k, v, ms, me)]) >=
                                    self.MODEL_DATA.TRANSFER_TIMES[ms][me]
                                )
                            )


        # ----------------------------------------------------------------------------------------------------
        # Predefined cuts
        # ----------------------------------------------------------------------------------------------------
        for v1 in self.decision_vars.vehicle_transfer_options:
            for v2 in self.decision_vars.vehicle_transfer_options:
                if v1[0:1] != v2[0:1] and v1[2] != v2[2] and v1[4] == v2[4]:
                    if v1[4] != 0 and (v1[4] != self.MODEL_DATA.NR_MACHINES - 1 or self.MODEL_DATA.SINGLE_UL):
                        self.model.add(
                            if_then(
                                logical_and(
                                    presence_of(self.decision_vars.vehicle_transfer_options[v1]),
                                    presence_of(self.decision_vars.vehicle_transfer_options[v2]),
                                ),
                                end_of(self.decision_vars.vehicle_transfer_options[v1]) !=
                                end_of(self.decision_vars.vehicle_transfer_options[v2])
                            )
                        )

        for v1 in self.decision_vars.vehicle_transfer_options:
            for v2 in self.decision_vars.vehicle_transfer_options:
                if v1[0:1] != v2[0:1] and v1[2] != v2[2] and v1[3] == v2[4]:
                    if v1[4] != 0 and (v1[4] != self.MODEL_DATA.NR_MACHINES - 1 or self.MODEL_DATA.SINGLE_UL):
                        self.model.add(
                            if_then(
                                logical_and(
                                    presence_of(self.decision_vars.vehicle_transfer_options[v1]),
                                    presence_of(self.decision_vars.vehicle_transfer_options[v2]),
                                ),
                                start_of(self.decision_vars.vehicle_transfer_options[v1]) !=
                                end_of(self.decision_vars.vehicle_transfer_options[v2])
                            )
                        )

        starting_transfers = []
        for j in range(self.MODEL_DATA.NR_JOBS):
            starting_transfers.append(start_of(self.decision_vars.transfers[j, 0]))

        # If out-degree is 2 we can shortcut
        if len(self.MODEL_DATA.getExclusiveAdjacentNodes(self.MODEL_DATA.MACHINE_LOCATIONS[0])) == 2:
            self.model.add(sum([a == b for idx, a in enumerate(starting_transfers) for b in starting_transfers[idx + 1:]])
                           < len(self.MODEL_DATA.getExclusiveAdjacentNodes(self.MODEL_DATA.MACHINE_LOCATIONS[0])))
        else:
            for idx, a in enumerate(starting_transfers):
                self.model.add(sum([a == b for idy, b in enumerate(starting_transfers) if idx != idy])
                           < len(self.MODEL_DATA.getExclusiveAdjacentNodes(self.MODEL_DATA.MACHINE_LOCATIONS[0])))
        # ----------------------------------------------------------------------------------------------------
        # Model Constraints
        # ----------------------------------------------------------------------------------------------------
        # Alternative constraints
        self.model.add(alternative_machines)
        self.model.add(alternative_vehicles)

        # Precedence constraints
        self.model.add(precedence_operations)

        # No overlap constraints
        self.model.add(no_overlap_machines)
        self.model.add(no_overlap_vehicles)

        # ----------------------------------------------------------------------------------------------------
        # Objective
        # ----------------------------------------------------------------------------------------------------
        objective = max(end_of(self.decision_vars.operations[j, o]) for j, o in self.decision_vars.operations)
        minimize_objective = minimize(objective)

        self.model.add(minimize_objective)

    def _OptimizationModel__build_figure(self):
        """
        Creates a figure of the problem solution
        """
        if self.res and visu.is_visu_enabled():
            # Draw solution
            visu.timeline('Solution for transportation constrained flexible job-shop ' + self.MODEL_DATA.FILENAME)

            visu.panel('Machines')
            for m in range(self.MODEL_DATA.NR_MACHINES):
                visu.sequence(name='M' + str(m))
                for a in self.decision_vars.machine_operation_options:
                    if a[2] == m:
                        itv = self.res.get_var_solution(self.decision_vars.machine_operation_options[a])
                        if itv.is_present():
                            visu.interval(itv, a[0], 'J{}-O{}'.format(a[0], a[1]))

            visu.panel('Vehicles')
            for i in range(self.MODEL_DATA.NR_VEHICLES):
                visu.sequence(name='V' + str(i))
                for a in self.decision_vars.vehicle_transfer_options:
                    if a[2] == i:
                        itv = self.res.get_var_solution(self.decision_vars.vehicle_transfer_options[a])
                        if itv.is_present():
                            visu.interval(itv, a[0], 'J{}-O{}'.format(a[0], a[1]))

                            # if a[1] > 0:
                            # end = itv.get_end()
                            # visu.transition(end, end + self.MODEL_DATA.TRANSFER_TIMES[a[3]][a[4]])

            visu.panel('Vehicles')
            for i in range(self.MODEL_DATA.NR_VEHICLES):

                seq = self.res.get_var_solution(self.decision_vars.vehicle_sequence[i])
                visu.sequence(name='V' + str(i))

                ts = seq.get_value()

                for j in range(len(ts)):
                    t = ts[j]
                    nm1 = t.get_name()

                    visu.interval(t, int(nm1[4]), 'T' + nm1[13] + nm1[15])

                    if j < len(ts) - 1:
                        t2 = ts[j + 1]
                        nm2 = t2.get_name()

                        end2 = t.get_end()
                        end = t2.get_start()
                        visu.transition(end2, end)

            return visu




