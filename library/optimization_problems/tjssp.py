from docplex.cp.model import *
import docplex.cp.utils_visu as visu

# sys.path.append('/benders/benders-new/library')
from library.optimization_model import *


class TJSSP(CPOptimizationModel):
    """
    CP Optimizer Model of the TFJSSP problem
    """

    def setup_model(self):
        """
          Sets up CP model for the TFJSSP problem
          """

        # ----------------------------------------------------------------------------------------------------
        # Alternative Vehicles
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
        # No overlap
        # ----------------------------------------------------------------------------------------------------
        no_overlap_machines = (
            no_overlap(
                [
                    self.decision_vars.operations[(j,o)]
                    for j, J in enumerate(self.MODEL_DATA.JOBS)
                    for o, O in enumerate(J)
                    for k, (m, d) in enumerate(O) if m2 == m
                ]
            )
            for m2 in range(self.MODEL_DATA.NR_MACHINES)
        )

        no_overlap_vehicles = (
            no_overlap(
                self.decision_vars.vehicle_sequence[m]
            ) for m in range(self.MODEL_DATA.NR_VEHICLES)
        )

        # ----------------------------------------------------------------------------------------------------
        # Operation Durations
        # ----------------------------------------------------------------------------------------------------
        operation_duration = (
            size_of(self.decision_vars.operations[(j, k)])
            >=
            self.MODEL_DATA.JOBS[j][k][0][1]
            for j in range(self.MODEL_DATA.NR_JOBS)
            for k in range(self.MODEL_DATA.JOB_SIZES[j])
        )

        # ----------------------------------------------------------------------------------------------------
        # Scheduling constraints
        # ----------------------------------------------------------------------------------------------------
        for j in range(self.MODEL_DATA.NR_JOBS):
            for k in range(self.MODEL_DATA.JOB_SIZES[j]):
                for m in self.decision_vars.transfers:
                    if j == m[0] and k == m[1]:
                        # The start time of a transfer depend on the end time of an operation.
                        self.model.add(
                            end_of(self.decision_vars.operations[(j, k)]) <= start_of(self.decision_vars.transfers[m])
                        )

                        # An operation can only start when the previous move is finished.
                        self.model.add(
                            end_of(self.decision_vars.transfers[m]) <= start_of(self.decision_vars.operations[(j, k+1)])
                        )

                        # A transfer is bounded by the move time inbetween the machines related to the transfer
                        self.model.add(
                            size_of(self.decision_vars.transfers[m]) >=
                             self.MODEL_DATA.TRANSFER_TIMES[
                                self.MODEL_DATA.JOBS[j][k][0][0]
                            ][
                                self.MODEL_DATA.JOBS[j][k + 1][0][0]
                            ]
                        )

        # ----------------------------------------------------------------------------------------------------
        # Transfer scheduling
        # ----------------------------------------------------------------------------------------------------
        for v in range(self.MODEL_DATA.NR_VEHICLES):
            for m in self.decision_vars.vehicle_transfer_options:
                if m[2] == v:
                    for i in range(self.MODEL_DATA.NR_MACHINES):
                        self.model.add(
                            if_then(
                                presence_of(self.decision_vars.vehicle_transfer_options[m]),
                                if_then(
                                    type_of_prev(
                                        self.decision_vars.vehicle_sequence[v],
                                        self.decision_vars.vehicle_transfer_options[m]
                                    ) == i,

                                    start_of(self.decision_vars.vehicle_transfer_options[m])
                                    >= end_of_prev(
                                        self.decision_vars.vehicle_sequence[v],
                                        self.decision_vars.vehicle_transfer_options[m]
                                    ) + self.MODEL_DATA.TRANSFER_TIMES[i][self.MODEL_DATA.JOBS[m[0]][m[1]][0][0]])
                            )
                        )

        # ----------------------------------------------------------------------------------------------------
        # Model Constraints
        # ----------------------------------------------------------------------------------------------------
        # Alternative constraints
        self.model.add(alternative_vehicles)

        # Operation duration
        self.model.add(operation_duration)

        # No overlap constraints
        self.model.add(no_overlap_machines)
        self.model.add(no_overlap_vehicles)

        # ----------------------------------------------------------------------------------------------------
        # Objective
        # ----------------------------------------------------------------------------------------------------
        objective = max(end_of(self.decision_vars.operations[j, o]) for j, o in self.decision_vars.operations)
        minimize_objective = minimize(objective)

        self.model.add(minimize_objective)
        pass

    def _OptimizationModel__build_figure(self):
        """
        Creates a figure of the problem solution
        """
        if self.res and visu.is_visu_enabled():
            visu.timeline('Solution for permutation flow-shop ' + self.MODEL_DATA.FILENAME)
            visu.panel('Jobs')

            for i in range(self.MODEL_DATA.NR_JOBS):
                visu.sequence(name='J' + str(i),
                              intervals=[(self.res.get_var_solution(self.decision_vars.operations[i,j]), i,
                                          'M' + to_string(self.MODEL_DATA.JOBS[i][j][0][0]) + ' K' + str(j))
                                         for j in range(self.MODEL_DATA.JOB_SIZES[i])])

            visu.panel('Machines')
            for i in range(self.MODEL_DATA.NR_MACHINES):
                visu.sequence(name='M' + str(i),
                              intervals=[(self.res.get_var_solution(self.decision_vars.operations[j,k]), j,
                                          'J' + to_string(j) + ' K' + str(k))
                                         for j in range(self.MODEL_DATA.NR_JOBS)
                                         for k in range(self.MODEL_DATA.JOB_SIZES[j])
                                         if (self.MODEL_DATA.JOBS[j][k][0][0] == i)])

            visu.panel('Vehicles')
            for i in range(self.MODEL_DATA.NR_VEHICLES):

                seq = self.res.get_var_solution(self.decision_vars.vehicle_sequence[i])
                visu.sequence(name='V' + str(i))

                ts = seq.get_value()

                for j in range(len(ts)):
                    t = ts[j]
                    nm = t.get_name()

                    tp1 = self.MODEL_DATA.JOBS[int(nm[1])][int(nm[4])][0][0]
                    tp2 = self.MODEL_DATA.JOBS[int(nm[1])][int(nm[4]) + 1][0][0]

                    visu.interval(t, int(nm[1]), 'T' + nm[1] + nm[4])

                    if j < len(ts) - 1:
                        t2 = ts[j + 1]
                        nm = t2.get_name()

                        tp3 = self.MODEL_DATA.JOBS[int(nm[1])][int(nm[4])][0][0]

                        end = t2.get_start()
                        visu.transition(end - self.MODEL_DATA.TRANSFER_TIMES[tp2][tp3], end)

            return visu