import docplex.cp.utils_visu as visu
import re
import itertools
import gc

from docplex.cp.model import *
from library.benders_decomposition import *
from library.optimization_problems.tfjssp import TFJSSP
from library.optimization_problems.tfjssp2 import TFJSSP2
from library.optimization_problems.tfjssp2lcm import TFJSSP2lcm
from library.optimization_problems.tfjssp2tcm import TFJSSP2tcm

from library.optimization_problems.cfr import CFR


class DecompositionCFTFJSSPweak(LogicBasedBendersDecomposition):
    def _LogicBasedBendersDecomposition__build_figure(self):
        if self.res and visu.is_visu_enabled():

            # Draw solution
            visu.timeline('Solution for transportation constrained flexible job-shop ' + self.MODEL_DATA.FILENAME)

            visu.panel('Locations')
            for m in range(self.MODEL_DATA.NR_MACHINES):
                if self.MODEL_DATA.SINGLE_UL:
                    if m == 0:
                        visu.sequence(name='L/U')
                    elif m != self.MODEL_DATA.NR_MACHINES:
                        visu.sequence(name='M' + str(m))
                else:
                    if m == 0:
                        visu.sequence(name='L')
                    elif m == self.MODEL_DATA.NR_MACHINES -1:
                        visu.sequence(name='U')
                    else:
                        visu.sequence(name='M' + str(m))
                for a in self.master_problem.decision_vars.machine_operation_options:
                    if a[2] == m:
                        itv = self.res.get_var_solution(self.master_problem.decision_vars.machine_operation_options[a])
                        if itv.is_present():
                            visu.interval(itv, a[0], 'J{}-O{}'.format(a[0], a[1]))

            visu.panel('Vehicles')
            for i in range(self.MODEL_DATA.NR_VEHICLES):
                visu.sequence(name='V' + str(i))
                for a in self.master_problem.decision_vars.vehicle_transfer_options:
                    if a[2] == i:
                        itv = self.res.get_var_solution(self.master_problem.decision_vars.vehicle_transfer_options[a])
                        if itv.is_present():
                            visu.interval(itv, a[0], 'T{}{}'.format(a[0], a[1]))


            visu.panel('Vehicles')
            for i in range(self.MODEL_DATA.NR_VEHICLES):

                seq = self.res.get_var_solution(self.master_problem.decision_vars.vehicle_sequence[i])
                visu.sequence(name='V' + str(i))

                ts = seq.get_value()

                for j in range(len(ts)):
                    t = ts[j]
                    nm1 = t.get_name()

                    visu.interval(t, int(nm1[4]), 'LT')

                    if j < len(ts) - 1:
                        t2 = ts[j + 1]
                        nm2 = t2.get_name()

                        end = t2.get_start()
                        end2 = t.get_end()

                        visu.transition(end2, end)

            visu.panel('Node Presence')
            for i in range(self.MODEL_DATA.NR_VEHICLES):
                visu.sequence(name='V' + str(i))

                seq = self.res.get_var_solution(self.master_problem.decision_vars.vehicle_sequence[i])
                ts = seq.get_value()

                for t in range(self.MODEL_DATA.TIME_HORIZON - 1):
                    for nx in self.MODEL_DATA.NODES:
                        if self.sub_problem.decision_vars.x[t, i, nx].solution_value != 0:
                            # if nx != ny:
                            visu.interval(t, t + 1, i, '{}'.format(nx))

            return visu

    def   __init__(self, name, model_data):
        # Store the model data
        self.MODEL_DATA = model_data

        # Initialize master problem
        master_data = model_data
        master_data.MODEL_TYPE = ModelType.TFJSSP2
        master_problem = TFJSSP2(name + "_master", model_data)

        # Initialize master problem
        sub_data = model_data
        sub_data.MODEL_TYPE = ModelType.CFR
        sub_problem = CFR(name + "_sub", model_data, ProblemType.FEASIBILITY_PROBLEM)

        super().__init__(name, master_problem, sub_problem)

    def _LogicBasedBendersDecomposition__extract_master_problem_results(self):
        """
        Extracts the relevant trail values from the TFJSSP mater problem solution
            - Starting times / completion times of transfers
            - Transfer to vehicle allocations

        """

        # List to hold sequence of transfers for each vehicle
        transferSequence = []

        # For all vehicles
        for i in range(self.MODEL_DATA.NR_VEHICLES):
            # Append new list for each vehicle
            transferSequence.append([])

            # Obtain vehicle sequence and transfers
            seq = self.master_problem.res.get_var_solution(self.master_problem.decision_vars.vehicle_sequence[i])
            ts = seq.get_value()

            # For all transfers
            for j in range(len(ts)):
                t = ts[j]
                nm = t.get_name()

                # Add first transfer
                if j == 0:
                    transferSequence[i].append(
                        Transfer(self.MODEL_DATA.MACHINE_LOCATIONS[int(nm[15])], 0, t.get_end())
                    )
                else:
                    tlast = ts[j - 1]

                    # Add empty transfers
                    transferSequence[i].append(
                        Transfer(self.MODEL_DATA.MACHINE_LOCATIONS[int(nm[13])], tlast.get_end(), t.get_start())
                    )

                    # Add subsequent transfers
                    transferSequence[i].append(
                        Transfer(self.MODEL_DATA.MACHINE_LOCATIONS[int(nm[15])], t.get_start(), t.get_end())
                    )

        self.master_problem_extract = transferSequence

    def _LogicBasedBendersDecomposition__apply_cuts(self):
        """
        Apply benders cuts
        """

        # Initialise new lists
        conflicting_cp_transfer_list = [] # List of the conflicting CP master problem transfers
        conflicting_mp_transfer_list = [] # List of the conflicting trial values for the MP sub problem


        for v in range(self.MODEL_DATA.NR_VEHICLES):
            conflicting_cp_transfer_list.append([])

        try:
            for cf in self.sub_problem.crefres.iter_conflicts():
                if cf[1].sense.value == 2 and cf[1].left_expr.size == 1:

                    # Extract variable that causes the conflict
                    conflicting_variable = cf[1].left_expr.lp_name
                    conflicting_variable = [int(s) for s in re.findall(r'\d+', conflicting_variable)]

                    # Find the transfer that corresponds to the conflicting variable
                    for transfer in self.master_problem_extract[int(conflicting_variable[1])]:
                        if int(conflicting_variable[0]) <= transfer.deadlineTime:
                            conflicting_cp_transfer_list[conflicting_variable[1]].append(
                                self._get_cp_transfer_from_mp(transfer, conflicting_variable[1])
                            )
                            conflicting_mp_transfer_list.append(transfer)
                            break


            conflict_correctness_verification = CFR(
                "CFR",
                self.MODEL_DATA,
                ProblemType.FEASIBILITY_PROBLEM
            )

            # Construct new specification of trial values for the CFR sub problem
            vehicle_transfer_list = []
            for v in range(self.MODEL_DATA.NR_VEHICLES):
                vehicle_transfer_list.append([])
                for transfer in self.master_problem_extract[v]:
                    if transfer in conflicting_mp_transfer_list:
                        vehicle_transfer_list[v].append(transfer)

            conflict_correctness_verification.setup_model(*vehicle_transfer_list)

            if self.print_output:
                print("Check correctness of conflicting transfers")

            # Perform correctness check of conflicting transfers
            conflict_correctness_verification.solve(print_output=self.print_output)

            if conflict_correctness_verification.get_objective() != 'integer infeasible':
                if self.print_output:
                    print("Correct set of conflicting transfers!")

                raise Exception("Conflict refinement may exclude optimal solutions")

            # Generate cut
            self._CUT_exclude_single_conflicting_transfer(conflicting_cp_transfer_list)
        except:
            # If set of conflicting transfers is incorrect apply simple feasibilty cut
            self.benders_cuts.append(
                logical_or(
                    self._CUT_get_schedule(True)[0],
                    logical_and(
                        logical_and(
                            self._CUT_fixate_vehicle_sequence()[0],
                            self._CUT_fixate_machine_sequence()[0]
                        ),
                        self._CUT_increase_any_transfer_duration()[0]
                    )
                )
            )

    def _CUT_fixate_vehicle_sequence(self):
        cons = []

        # For all vehicle sequences
        for v in range(self.MODEL_DATA.NR_VEHICLES):
            vehicleSequence = []

            for vo in self.master_problem.decision_vars.vehicle_transfer_options:
                if vo[2] == v:
                    # Get all transfers from vehicle sequence
                    vehicleAllocation = self.master_problem.res.get_var_solution(
                        self.master_problem.decision_vars.vehicle_transfer_options[vo]
                    )
                    # Guarantee existence of all transfers in vehicle sequence
                    if vehicleAllocation.is_present():
                        vehicleSequence.append(vehicleAllocation)
                        cons.append(presence_of(vehicleAllocation.get_expr()))

            # Guarantee the same ordering of transfers
            for opA, opB in itertools.combinations(vehicleSequence, 2):
                if opA.get_start() > opB.get_start():
                    cons.append(start_of(opA.get_expr()) > start_of(opB.get_expr()))
                else:
                    cons.append(start_of(opA.get_expr()) < start_of(opB.get_expr()))

        # Create a conjunction of all expressions
        while len(cons) != 1:
            cons = conjunctify(cons, False)

        return cons

    def _CUT_fixate_machine_sequence(self):
        cons = []

        # For all machine sequences
        for m in range(self.MODEL_DATA.NR_MACHINES):
            machineSequence = []

            for op in self.master_problem.decision_vars.machine_operation_options:
                if op[2] == m:
                    # Get all operations from machine sequence
                    machineAllocation = self.master_problem.res.get_var_solution(
                        self.master_problem.decision_vars.machine_operation_options[op]
                    )
                    # Guarantee existence of all operations in machine sequence
                    if machineAllocation.is_present():
                        machineSequence.append(machineAllocation)
                        cons.append(presence_of(machineAllocation.get_expr()))

            # Guarantee the same ordering of operations
            for opA, opB in itertools.combinations(machineSequence, 2):
                if opA.get_start() > opB.get_start():
                    cons.append(start_of(opA.get_expr()) > start_of(opB.get_expr()))
                else:
                    cons.append(start_of(opA.get_expr()) < start_of(opB.get_expr()))

        # Create a conjunction of all expressions
        while len(cons) != 1:
            cons = conjunctify(cons, False)

        return cons

    def _CUT_increase_any_transfer_duration(self):
        cons = []

        # For all transfers
        for vo in self.master_problem.decision_vars.vehicle_transfer_options:
            transfer = self.master_problem.res.get_var_solution(
                self.master_problem.decision_vars.vehicle_transfer_options[vo]
            )
            if transfer.is_present():
                interval = transfer.get_expr()

                # Expression that size of transfer should be bigger than before
                cons.append((size_of(interval) > transfer.get_size()))

        # Create a disjunction of all expressions
        while len(cons) != 1:
            cons = disjunctify(cons, False)

        return cons

    def _CUT_get_schedule(self, negated):
        # Obtain solution of last master problem iteration
        sol = self.master_problem.res.get_solution()

        # Get all constraints
        cons = sol.get_as_constraints()

        # Find all chosen alternative decision variables
        alternative_vars = []
        for tr in self.master_problem.decision_vars.transfers:
            alternative_vars.append('T_J' + str(tr[0]) +'_O' + str(tr[1]))
        for o in self.master_problem.decision_vars.operations:
            alternative_vars.append('O_J' + str(o[0]) +'_O' + str(o[1]))

        # Create a list of not chosen alternatives
        for i, con in enumerate(cons):
            k = con.children

            if k[0].name in alternative_vars:
                cons.pop(i)

        # Create a conjunction of all expressions
        while len(cons) != 1:
            cons = conjunctify(cons, False)

        # If we want to replicate same schedule we need to negate
        if negated:
            cons[0] = logical_not(cons[0])

        return cons

    def _CUT_exclude_single_conflicting_transfer(self, interval_list):
        expression_list = []

        sol = self.master_problem.res.get_solution()
        cons = sol.get_as_constraints()

        for v in range(self.MODEL_DATA.NR_VEHICLES):
            for i, interval in enumerate(interval_list[v]):
                if interval[0].is_present():
                    itv = interval[0].get_expr()

                    if interval[1] != 1:
                        expression_list.append(start_of(itv) == interval[0].get_start())
                        expression_list.append(end_of(itv) == interval[0].get_end())
                    else:
                        expression_list.append(start_of(itv) == interval[0].get_start())
                        expression_list.append(end_of(itv) == interval[0].get_end())

                        if interval[2].is_present():
                            itv = interval[2].get_expr()

                            expression_list.append(start_of(itv) == interval[2].get_start())
                            expression_list.append(end_of(itv) == interval[2].get_end())

        while len(expression_list) != 1:
            expression_list = conjunctify(expression_list, False)

        expression_list[0] = logical_not(expression_list[0])

        self.benders_cuts.append(expression_list)

    def _CUT_exclude_conflicting_transfers(self, interval_list):

        global_expression_list = [] # Variable to fill all constraints as part of the cut

        #
        for v in range(self.MODEL_DATA.NR_VEHICLES):
            global_expression_list.append([])

            for v2 in range(self.MODEL_DATA.NR_VEHICLES):
                global_expression_list[v].append([])

            # Start building a new expression list for each conflicting transfer that occurred for vehicle v
            #       - Each first index covers a different conflicting transfer
            #       - Each second index covers every similar transfer for that conflicting transfer
            #       - Each third index covers a constraint for every vehicle regarding the similar transfer
            expression_list = []

            # Go over all conflicting transfers
            for i, interval in enumerate(interval_list[v]):

                alternative_counter = -1

                # Obtain loaded transfer expression and conflicting start and end times
                itve = interval[0].get_expr()
                itve_start = interval[0].get_start()
                itve_end = interval[0].get_end()

                # Extract the job, operation, vehicle, and target destination
                digits = re.findall(r'\d+', itve.name)
                itv_e_specs = [int(digit) for digit in digits]

                # If empty transfer we need to obtain both loaded transfers causing the empty transfer
                if interval[1] == 1:
                    itvs = interval[2].get_expr()
                    itvs_start = interval[2].get_start()
                    itvs_end = interval[2].get_end()

                    digits = re.findall(r'\d+', itvs.name)
                    itv_s_specs = [int(digit) for digit in digits]


                expression_list.append([])

                for j1 in range(self.MODEL_DATA.NR_JOBS):
                    for o1 in range(self.MODEL_DATA.JOB_SIZES[j1]):
                        # If empty transfer is conflicting we need to constrain surrounding loaded transfers
                        if interval[1] == 1:
                            for j2 in range(self.MODEL_DATA.NR_JOBS):
                                for o2 in range(self.MODEL_DATA.JOB_SIZES[j2]):
                                    if not (j1 == j2 and o1 <= o2):
                                        # Find all similar transfers
                                        if ((j2, o2, v, itv_s_specs[3], itv_s_specs[4]) in self.master_problem.decision_vars.vehicle_transfer_options and
                                            (j1, o1, v, itv_e_specs[3], itv_e_specs[4]) in self.master_problem.decision_vars.vehicle_transfer_options):

                                            # For each similar transfer we increment alternative counter
                                            alternative_counter += 1
                                            # For each similar transfer set we find append new list
                                            expression_list[i].append([])

                                            # Create a constraint for the similar transfer for each vehicle
                                            for vit in range(self.MODEL_DATA.NR_VEHICLES):
                                                expression_list[i][alternative_counter].append([])
                                                expression_list[i][alternative_counter][vit].append(
                                                    logical_and(
                                                        end_of(self.master_problem.decision_vars.vehicle_transfer_options[
                                                                    j2, o2, vit, itv_s_specs[3], itv_s_specs[4]
                                                                ])
                                                        == itvs_end,
                                                        start_of(
                                                            self.master_problem.decision_vars.vehicle_transfer_options[
                                                                j1, o1, vit, itv_e_specs[3], itv_e_specs[4]
                                                            ])
                                                        == itve_start,
                                                    ),
                                            )
                        else:
                            # Find similar transfers
                            if (j1, o1, v, itv_e_specs[3], itv_e_specs[4]) in self.master_problem.decision_vars.vehicle_transfer_options:

                                # For each similar transfer we increment alternative counter
                                alternative_counter += 1
                                # For each similar transfer set we find append new list
                                expression_list[i].append([])

                                # Create a constraint for the similar transfer for each vehicle
                                for vit in range(self.MODEL_DATA.NR_VEHICLES):
                                    expression_list[i][alternative_counter].append([])
                                    expression_list[i][alternative_counter][vit].append(
                                        logical_and(
                                            start_of(self.master_problem.decision_vars.vehicle_transfer_options[
                                                         j1, o1, vit, itv_e_specs[3], itv_e_specs[4]
                                                     ])
                                            == itve_start,
                                            end_of(self.master_problem.decision_vars.vehicle_transfer_options[
                                                       j1, o1, vit, itv_e_specs[3], itv_e_specs[4]
                                                   ])
                                            == itve_end
                                        )
                                    )

            # Do some garbage collection to increase speed
            gc.collect()

            # Add all combinations of constraints to global list
            for k in itertools.product(*expression_list):
                g = list(k)

                for vit in range(self.MODEL_DATA.NR_VEHICLES):
                    f = []

                    # Create a temporary list of combination
                    z = len(g)
                    for h in range(len(g)):
                        f.append(g[h][vit][0])

                    # Create a conjunction of all items in the list
                    while len(f) > 1:
                        f = conjunctify(f, False)

                    # Add the expression to the global expression list
                    if f != []:
                        global_expression_list[v][vit].append(f)
                gc.collect()

        # Do some more garbage collection to increase speed
        gc.collect()

        # Remove empty items from global expression ist
        global_expression_list = [x for x in global_expression_list if x != [[] for y in range(self.MODEL_DATA.NR_VEHICLES)]]

        vehicle_choices = [[y for y in range(self.MODEL_DATA.NR_VEHICLES)] for x in range(len(global_expression_list))]

        # Gives the cartesian product of two lists with unique items only
        # Copyright: Tim Peters
        def _uprod(*seqs):
            def inner(i):
                if i == n:
                    yield tuple(result)
                    return
                for elt in sets[i] - seen:
                    seen.add(elt)
                    result[i] = elt
                    for t in inner(i + 1):
                        yield t
                    seen.remove(elt)

            sets = [set(seq) for seq in seqs]
            n = len(sets)
            seen = set()
            result = [None] * n
            for t in inner(0):
                yield t


        # Find all combinations of vehicles executing similar transfers
        for l in _uprod(*vehicle_choices):
            # Find a new constraint based on two alternative expressions
            for k in itertools.product(*[global_expression_list[x][l[x]] for x in range(len(global_expression_list))]):
                g = list(k)

                f = []

                z = len(g)
                for h in range(len(g)):
                    f.append(g[h][0])

                while len(f) > 1:
                    f = conjunctify(f, False)

                # Append new cuts to the list of benders cuts
                self.benders_cuts.append(logical_not(f[0]))

    def _get_cp_transfer_from_mp(self, transfer, vehicle):
        """
        Obtains the CP master problem transfer from a trial value
        """

        # Obtain vehicle sequence and transfers
        seq = self.master_problem.res.get_var_solution(self.master_problem.decision_vars.vehicle_sequence[vehicle])
        ts = seq.get_value()

        # For all transfers
        for j in range(len(ts)):
            t = ts[j]
            nm = t.get_name()


            if j == 0:
                # Returns if first transfer
                if Transfer(self.MODEL_DATA.MACHINE_LOCATIONS[int(nm[15])], 0, t.get_end()) == transfer:
                    return (t, 0)


            else:
                tlast = ts[j - 1]

                # Returns if empty transfer
                if (Transfer(self.MODEL_DATA.MACHINE_LOCATIONS[int(nm[13])], tlast.get_end(), t.get_start())
                        == transfer):
                    return (t, 1, tlast)

                # Returns if subsequent transfer
                if (Transfer(self.MODEL_DATA.MACHINE_LOCATIONS[int(nm[15])], t.get_start(), t.get_end())
                        == transfer):
                    return (t, 2)

        return None

