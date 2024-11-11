from library.optimization_model import *


class CFR(MIPOptimizationModel):
    """
    CPLEX ILP Model of the CFR problem
    """

    def setup_model(self, *args):
        """
        Sets up

        :param *args: List of transfers that construct the CFR problem
        """

        # Parse list of transfers
        vehicle_transfers = []
        for keys in args:
            vehicle_transfers.append(keys)


        # Setup variable data
        self.MODEL_DATA.TRANSFERS = self.getTransfers(vehicle_transfers)
        self.MODEL_DATA.TIME_HORIZON = self.getTimeHorizon(vehicle_transfers) + 1

        x_index = [ (t, v, n) 
            for t in range(self.MODEL_DATA.TIME_HORIZON) 
            for v in range(self.MODEL_DATA.NR_VEHICLES) 
            for n in self.MODEL_DATA.NODES
        ]

        # Decision variable setup
        self.decision_vars.c = None
        self.decision_vars.c = self.model.integer_var_dict(
            self.MODEL_DATA.TRANSFERS, ub=self.MODEL_DATA.TIME_HORIZON, name='c'
        )
        self.decision_vars.x = None
        self.decision_vars.x = self.model.binary_var_dict(
            x_index, name='x'
        )

        # ----------------------------------------------------------------------------------------------------
        # Continuity constraints
        # ----------------------------------------------------------------------------------------------------

        # Every timestamp a presence is required
        self.model.add_constraints(
            self.model.sum(self.decision_vars.x[t, v, n] for n in self.MODEL_DATA.NODES) == 1 
            for t in range(self.MODEL_DATA.TIME_HORIZON) 
            for v in self.MODEL_DATA.VEHICLES
        )

        # Time continuity // move to adjacent nodes
        self.model.add_constraints(
            self.decision_vars.x[t + self.MODEL_DATA.DELTA, v, n] <= self.model.sum(
                self.decision_vars.x[t, v, nz]
                for nz in self.MODEL_DATA.NODES if nz in self.MODEL_DATA.getAdjacentNodes(n)
            )
            for t in range(self.MODEL_DATA.TIME_HORIZON - 1)
            for v in self.MODEL_DATA.VEHICLES
            for n in self.MODEL_DATA.NODES
        )

        # ----------------------------------------------------------------------------------------------------
        # Start constraints
        # ----------------------------------------------------------------------------------------------------

        # Start location
        self.model.add_constraints(
            self.decision_vars.x[0, v, self.MODEL_DATA.VEHICLE_START_LOCATIONS] == 1
            for v in self.MODEL_DATA.VEHICLES
        )

        # ----------------------------------------------------------------------------------------------------
        # Completion constraints
        # ----------------------------------------------------------------------------------------------------

        # Visit all end points in time window
        self.model.add_constraints(
            self.decision_vars.x[
                vehicle_transfers[v][k].deadlineTime, v, vehicle_transfers[v][k].endNode
            ] == 1
            for v in self.MODEL_DATA.VEHICLES
            for k in range(len(vehicle_transfers[v]))
            if vehicle_transfers[v][k] != []
        )

        # Deadline constraints
        for v in self.MODEL_DATA.VEHICLES:
            self.model.add(
                self.decision_vars.c[v, k] <= vehicle_transfers[v][k].deadlineTime
                for k in range(len(vehicle_transfers[v]))
                if vehicle_transfers[v][k] != []
            )

        # Completion time
        for v in self.MODEL_DATA.VEHICLES:
            for k in range(len(vehicle_transfers[v])):
                if vehicle_transfers[v][k] != []:
                    self.model.add_indicator_constraints(
                        self.model.indicator_constraint(
                            self.decision_vars.x[t, v, vehicle_transfers[v][k].endNode], self.decision_vars.c[v, k] == t
                        )
                        for t in range(vehicle_transfers[v][k].startingTime, vehicle_transfers[v][k].deadlineTime)
                    )

        # ----------------------------------------------------------------------------------------------------
        # Completion constraints
        # ----------------------------------------------------------------------------------------------------

        # Conflict free nodes
        self.model.add_constraints(
            sum(self.decision_vars.x[t, v, n] for v in self.MODEL_DATA.VEHICLES) <= 1
            for t in range(self.MODEL_DATA.TIME_HORIZON)
            for n in self.MODEL_DATA.NODES
            if n != self.MODEL_DATA.VEHICLE_START_LOCATIONS and n != self.MODEL_DATA.MACHINE_LOCATIONS[self.MODEL_DATA.NR_MACHINES - 1]
        )

        # Conflict free edges
        self.model.add_constraints(
            self.decision_vars.x[t + self.MODEL_DATA.DELTA, vx, nx] +
            self.decision_vars.x[t + self.MODEL_DATA.DELTA, vy, ny] <=
            3 - (self.decision_vars.x[t, vx, ny] + self.decision_vars.x[t, vy, nx])

            for t in range(self.MODEL_DATA.TIME_HORIZON - self.MODEL_DATA.DELTA)
            for nx in self.MODEL_DATA.NODES
            for ny in self.MODEL_DATA.getExclusiveAdjacentNodes(nx)
            for vx in self.MODEL_DATA.VEHICLES
            for vy in self.MODEL_DATA.VEHICLES
        )

    def _OptimizationModel__build_figure(self):
        """
        Creates a figure of the problem solution
        """

        if self.res != None:

            fig, ax = plt.subplots()
            colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown', 'tab:pink',
                      'tab:gray', 'tab:olive', 'tab:cyan']

            for v in self.MODEL_DATA.VEHICLES:
                for t in range(self.MODEL_DATA.TIME_HORIZON):
                    for nx in self.MODEL_DATA.NODES:
                        if self.decision_vars.x[t, v, nx].solution_value != 0:
                            ax.broken_barh([(t, self.MODEL_DATA.DELTA)], (v, 0.8), facecolors=colors[v % len(colors)],
                                           edgecolor='black')
                            ax.text(t + self.MODEL_DATA.DELTA / 2, v + 0.4, f'{nx}', ha='center', va='center')

            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Vehicles')
            ax.set_yticks(self.MODEL_DATA.VEHICLES)
            ax.grid(True)
            ax.invert_yaxis()

            return plt
