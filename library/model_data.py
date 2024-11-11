
import os
from tkinter.constants import SINGLE

import numpy as np
from enum import Enum
from library.dijkstra import *


class ModelType(Enum):
    """
    Supported types of TJSP's
    """
    TJSSP = 1
    TFJSSP = 2
    CFTFJSSP = 3
    FJSSP = 4
    CFR = 5
    CFTJSSP = 6
    TFJSSP2 = 8


class ModelData:
    """
    The model data class is used to setup and store all the relevant data for each of the supported TJSP's
    """

    def __init__(self, filepath, filename, model_type):
        """
        Specify a TJSP data file for parsing and setting up model data

        :param filepath: Path to the TJSP data file
        :param filename: Name of the TJSP data file
        :param model_type: Specify the type of TJSP
        """
        self.FILENAME = filename

        # Type checking
        if not isinstance(model_type, ModelType):
            raise TypeError('TJSP type must be an instance of ModelType Enum')

        self.MODEL_TYPE = model_type

        # Run parsing script based on ModelType
        match model_type:
            case ModelType.TJSSP:
                self.__read_TJSSP(filepath + filename)
            case ModelType.TFJSSP:
                self.__read_TFJSSP(filepath + filename)
            case ModelType.TFJSSP2:
                self.__read_TFJSSP(filepath + filename)
            case ModelType.CFTFJSSP:
                self.__read_CFTFJSSP(filepath + filename)
            case ModelType.CFTJSSP:
                self.__read_CFTJSSP(filepath + filename)
            case ModelType.FJSSP:
                self.__read_FJSSP(filepath + filename)
            case ModelType.CFR:
                self.__read_CFR(filepath + filename)

    def __read_CFR(self, filename):
        """
        Parses and sets up a CFR data file

        :param filename: Name of the TJSP data file
        """

        # Open data file
        with(open(filename, 'r') as file):
            # Read job and routing network data
            self.NR_JOBS, self.NR_MACHINES, self.NR_VEHICLES = [
                int(v.replace("(", "").replace(")", "")) for v in file.readline().split()
            ]
            self.NR_ROWS, self.NR_COLS = [
                v.replace("(", "").replace(")", "") for v in file.readline().split(sep='x')
            ]
            self.NR_ROWS = int(self.NR_ROWS)

            # Check if diagonal routing is allowed
            if 'd' in self.NR_COLS:
                self.NR_COLS = int(self.NR_COLS.replace("d\n", ""))
                self.IS_DIAGONAL = True
            else:
                self.NR_COLS = int(self.NR_COLS)
                self.IS_DIAGONAL = False

            self.NR_NODES = self.NR_ROWS * self.NR_COLS
            self.NR_MACHINES += 1 # A loading station is added to the NR of machines
            self.MACHINE_LOCATIONS = np.array([int(v) for v in file.readline().split()])
            self.REMOVE_EDGES = [
                int(v.replace("(", "").replace(")", "")) for v in file.readline().split()
            ] # Specified obstacles to remove later
            self.VEHICLE_START_LOCATIONS = self.MACHINE_LOCATIONS[0] # Vehicles start at loading station
            self.NODES = [ i +1 for i in range(self.NR_NODES)]
            self.EDGES = self.__construct_edges(self.NR_ROWS, self.NR_COLS)

            # Remove all defined obstacles from the routing network
            for i in range(0, len(self.REMOVE_EDGES), 2):
                self.remove_edges(self.REMOVE_EDGES[i], self.REMOVE_EDGES[i + 1])

            # Run dijkstra to find shortest distances between all nodes
            dijkstra_graph = Graph(self.NR_NODES)
            for e in self.EDGES:
                dijkstra_graph.addEdge(e[0] - 1, e[1] - 1, 1)

            self.VEHICLES = [i for i in range(self.NR_VEHICLES)]
            self.DELTA = 1 # Duration of a time unit

    def __read_CFTFJSSP(self, filename):
        """
        Parses and sets up a CFTFJSSP data file

        :param filename: Name of the TJSP data file
        """

        # Open data file
        with (open(filename, 'r') as file):

            # Read job and routing network data
            self.NR_JOBS, self.NR_MACHINES, self.NR_VEHICLES = [
                int(v.replace("(", "").replace(")", "")) for v in file.readline().split()
            ]
            JOB_LIST = [
                [
                    int(v.replace("(", "").replace(")", "")) for v in file.readline().split()
                ] for i in range(self.NR_JOBS)]
            self.NR_ROWS, self.NR_COLS = [
                v.replace("(", "").replace(")", "") for v in file.readline().split(sep='x')
            ]
            self.NR_ROWS = int(self.NR_ROWS)

            # Check if diagonal routing is allowed
            if 'd' in self.NR_COLS:
                self.NR_COLS = int(self.NR_COLS.replace("d\n", ""))
                self.IS_DIAGONAL = True
            else:
                self.NR_COLS = int(self.NR_COLS)
                self.IS_DIAGONAL = False

            self.NR_NODES = self.NR_ROWS * self.NR_COLS
            self.MACHINE_LOCATIONS = np.array([int(v) for v in file.readline().split()])
            self.NR_MACHINES = len(self.MACHINE_LOCATIONS)
            self.REMOVE_EDGES = [
                int(v.replace("(", "").replace(")", "")) for v in file.readline().split()
            ] # Specified obstacles to remove later
            self.VEHICLE_START_LOCATIONS = self.MACHINE_LOCATIONS[0] # Vehicles start at loading station
            self.NODES = [ i +1 for i in range(self.NR_NODES)]
            self.EDGES = self.__construct_edges(self.NR_ROWS, self.NR_COLS)

            # Remove all defined obstacles from the routing network
            for i in range(0, len(self.REMOVE_EDGES), 2):
                self.remove_edges(self.REMOVE_EDGES[i], self.REMOVE_EDGES[ i +1])

            # Run dijkstra to find shortest distances between all nodes
            dijkstra_graph = Graph(self.NR_NODES)
            for e in self.EDGES:
                dijkstra_graph.addEdge(e[0 ] -1, e[1 ] -1, 1)

            self.VEHICLES = [i for i in range(self.NR_VEHICLES)]
            self.DELTA = 1 # Duration of a time unit

            self.TRANSFER_TIMES = []

            for i, m1 in enumerate(self.MACHINE_LOCATIONS):
                self.TRANSFER_TIMES.append([])
                distances = dijkstra_graph.shortestPath(m1 -1)

                for j, m2 in enumerate(self.MACHINE_LOCATIONS):
                    self.TRANSFER_TIMES[i].append(distances[m2 -1])

        # Build jobs based on JOB_LIST
        self.__construct_jobs_with_load_and_unload(JOB_LIST)

    def __read_TFJSSP(self, filename):
        with (open(filename, 'r') as file):
            self.NR_JOBS, self.NR_MACHINES, self.NR_VEHICLES = [
                int(v.replace("(", "").replace(")", "")) for v in file.readline().split()
            ]
            JOB_LIST = [
                [
                    int(v.replace("(", "").replace(")", "")) for v in file.readline().split()
                ] for i in range(self.NR_JOBS)]
            self.NR_MACHINES += 1
            self.TRANSFER_TIMES = np.array([[int(v) for v in file.readline().split()] for i in range(self.NR_MACHINES)])

        self.__construct_jobs_with_load_and_unload(JOB_LIST)

        if self.MODEL_TYPE == ModelType.TFJSSP:
            for i in range(len(self.MACHINE_JOBS)):
                self.MACHINE_JOBS[i].append([0])


    def __read_TJSSP(self, filename):
        """
        Parses and sets up a TJSSP data file

        :param filename: Name of the TJSP data file
        """

        # Open data file
        with (open(filename, 'r') as file):

            # Read number of jobs, machines and vehicles
            self.NR_JOBS, self.NR_MACHINES, self.NR_VEHICLES = [
                int(v.replace("(", "").replace(")", "")) for v in file.readline().split()
            ]
            JOB_LIST = [
                [
                    int(v.replace("(", "").replace(")", "")) for v in file.readline().split()
                ] for i in range(self.NR_JOBS)]

            # Make sure JOBS are of the same format as the TFJSSP
            for i in range(self.NR_JOBS):
                operations = int(len(JOB_LIST[i]) / 2)

                JOB_LIST[i].insert(0, operations)
                for k, j in enumerate(range(0, len(JOB_LIST[i] ) -1, 2)):

                    JOB_LIST[i].insert(1 + j + k, 1)

            # Setup transfer time matrix
            self.TRANSFER_TIMES = np.array([[int(v) for v in file.readline().split()] for i in range(self.NR_MACHINES)])
            self.__construct_jobs_with_load_and_unload(JOB_LIST)

            print("test")

    def __read_CFTJSSP(self, filename):
        with (open(filename, 'r') as file):
            self.NR_JOBS, self.NR_MACHINES, self.NR_VEHICLES = [
                int(v.replace("(", "").replace(")", "")) for v in file.readline().split()
            ]
            self.NR_ROWS, self.NR_COLS = [
                int(v.replace("(", "").replace(")", "")) for v in file.readline().split(sep='x')
            ]
            self.NR_NODES = self.NR_ROWS * self.NR_COLS
            JOB_LIST = [
                [
                    int(v.replace("(", "").replace(")", "")) for v in file.readline().split()
                ] for i in range(self.NR_JOBS)]

            # Make sure JOBS are of the same format as the TFJSSP
            for i in range(self.NR_JOBS):
                operations = int(len(JOB_LIST[i]) / 2)

                JOB_LIST[i].insert(0, operations)
                for k, j in enumerate(range(0, len(JOB_LIST[i]) - 1, 2)):
                    JOB_LIST[i].insert(1 + j + k, 1)

            self.MACHINE_LOCATIONS = np.array([int(v) for v in file.readline().split()])

            self.NODES = [i for i in range(self.NR_NODES)]
            self.EDGES = self.__construct_edges(self.NR_ROWS, self.NR_COLS)
            self.VEHICLES = [i for i in range(self.NR_VEHICLES)]
            self.DELTA = 1

            self.TRANSFER_TIMES = []

            for m1 in range(self.NR_MACHINES):
                self.TRANSFER_TIMES.append([])
                for m2 in range(self.NR_MACHINES):
                    if m1 == m2:
                        self.TRANSFER_TIMES[m1].append(0)
                    else:
                        self.TRANSFER_TIMES[m1].append(self.manhattan_distance(
                            self.MACHINE_LOCATIONS[m1],
                            self.MACHINE_LOCATIONS[m2],
                        ))

        self.__construct_jobs_with_load_and_unload(JOB_LIST)

    def __read_FJSSP(self, filename):
        with (open(filename, 'r') as file):
            self.NR_JOBS, self.NR_MACHINES = [
                int(v.replace("(", "").replace(")", "")) for v in file.readline().split()
            ]
            JOB_LIST = [
                [
                    int(v.replace("(", "").replace(")", "")) for v in file.readline().split()
                ] for i in range(self.NR_JOBS)]

        self.__construct_jobs(JOB_LIST)



    def __construct_jobs(self, job_list):
        """
        Constructs a set of jobs and alternative options based on data file

        :param job_list: List of job data extracted from the TJSP data file
        """

        if not hasattr(self, 'MACHINE_LOCATIONS'):
            print("CRAASHH")


        self.JOBS = []
        self.JOB_SIZES = []
        self.MACHINE_JOBS = []

        # For all jobs in data file
        for job_line in job_list:
            nr_steps = job_line.pop(0) # Extract number of operations
            self.JOB_SIZES.append(nr_steps) # Job size als includes travel from and to loading/unloading station

            # Start creating job
            job = []

            # Store machines used by job for later use
            machine_job = []

            # For each operation
            for stp in range(nr_steps):
                nbc = job_line.pop(0)

                choices = []
                machine_choices = []

                # For all options in each operation
                for c in range(nbc):
                    m = job_line.pop(0)
                    d = job_line.pop(0)

                    # Extract choice
                    choices.append((m, d))
                    machine_choices.append(m)

                # Store choices in job list
                job.append(choices)
                machine_job.append(machine_choices)

            # Store all information
            self.JOBS.append(job)
            self.MACHINE_JOBS.append(machine_job)


    def __construct_jobs_with_load_and_unload(self, job_list):
        """
        Constructs a set of jobs and alternative options based on data file

        :param job_list: List of job data extracted from the TJSP data file
        """

        if not hasattr(self, 'MACHINE_LOCATIONS'):
            print("CRAASHH")


        self.JOBS = []
        self.JOB_SIZES = []
        self.MACHINE_JOBS = []

        # For all jobs in data file
        for job_line in job_list:
            nr_steps = job_line.pop(0) # Extract number of operations
            self.JOB_SIZES.append(nr_steps + 2) # Job size als includes travel from and to loading/unloading station

            # Start creating job
            job = []
            job.append([(0, 0)]) # Dummy operation of duration 0 in loading station

            # Store machines used by job for later use
            machine_job = []
            machine_job.append([0])

            # For each operation
            for stp in range(nr_steps):
                nbc = job_line.pop(0)

                choices = []
                machine_choices = []

                # For all options in each operation
                for c in range(nbc):
                    m = job_line.pop(0)
                    d = job_line.pop(0)

                    # Extract choice
                    choices.append((m, d))
                    machine_choices.append(m)

                # Store choices in job list
                job.append(choices)
                machine_job.append(machine_choices)

                # # End with dummy operation of duration 0 in unloading station
            try:
                if self.MACHINE_LOCATIONS[0] == self.MACHINE_LOCATIONS[self.NR_MACHINES - 1]:
                    # If single unloading / loading (UL) station is specified
                    self.SINGLE_UL = True
                    job.append([(0, 0)])
                    machine_job.append([0])
                else:
                    self.SINGLE_UL = False

                    # If separate loading / unloading station is specified
                    job.append([(self.NR_MACHINES - 1, 0)])
                    machine_job.append([self.NR_MACHINES - 1])

                if self.SINGLE_UL: self.NR_MACHINES -= 1  # Load / Unload is the same station so remove redundant machine count

            except:
                job.append([(0, 0)])
                machine_job.append([0])


            # Store all information
            self.JOBS.append(job)
            self.MACHINE_JOBS.append(machine_job)


    def __construct_edges(self, rows, columns):
        """
        Constructs a set of edges based on data file

        :param rows: Number of rows in the grid
        :param columns: Number of columns in the grid
        """

        # Specify row/column end-indexes
        row_end_index = rows
        column_end_index = columns

        # ----------------------------------------------------------------
        # Corners first
        # ----------------------------------------------------------------
        if self.IS_DIAGONAL:
            edges = [
                # Bot Left
                (1, 2),
                (1, column_end_index + 1),
                (1, column_end_index + 2),

                # Bot Right
                (self.to_node(1, column_end_index), self.to_node(1, column_end_index - 1)),
                (self.to_node(1, column_end_index), self.to_node(2, column_end_index)),
                (self.to_node(1, column_end_index), self.to_node(2, column_end_index -1)),

                # Top Left
                (self.to_node(row_end_index, 1), self.to_node(row_end_index - 1, 1)),
                (self.to_node(row_end_index, 1), self.to_node(row_end_index, 2)),
                (self.to_node(row_end_index, 1), self.to_node(row_end_index - 1, 2)),

                # Top Right
                (
                    self.to_node(row_end_index, column_end_index),
                    self.to_node(row_end_index - 1, column_end_index)
                ),
                (
                    self.to_node(row_end_index, column_end_index),
                    self.to_node(row_end_index, column_end_index - 1)
                ),
                (
                    self.to_node(row_end_index, column_end_index),
                    self.to_node(row_end_index - 1, column_end_index - 1)
                )
            ]
        else:
            edges = [
                # Bot Left
                (1, 2),
                (1, column_end_index + 1),

                # Bot Right
                (self.to_node(1, column_end_index), self.to_node(1, column_end_index - 1)),
                (self.to_node(1, column_end_index), self.to_node(2, column_end_index)),

                # Top Left
                (self.to_node(row_end_index, 1), self.to_node(row_end_index - 1, 1)),
                (self.to_node(row_end_index, 1), self.to_node(row_end_index, 2)),

                # Top Right
                (
                    self.to_node(row_end_index, column_end_index),
                    self.to_node(row_end_index - 1, column_end_index)
                ),
                (
                    self.to_node(row_end_index, column_end_index),
                    self.to_node(row_end_index, column_end_index - 1)
                )
            ]

        # ----------------------------------------------------------------
        # Sides seconds
        # ----------------------------------------------------------------
        for r in range(rows):
            r += 1
            for c in range(columns):
                c += 1
                if r == 1 or r == row_end_index:
                    if c != 1 and c != column_end_index:
                        if r == 1:
                            edges.append((self.to_node(r, c), self.to_node(r, c + 1)))
                            edges.append((self.to_node(r, c), self.to_node(r, c - 1)))
                            edges.append((self.to_node(r + 1, c), self.to_node(r, c)))
                            if self.IS_DIAGONAL:
                                edges.append((self.to_node(r, c), self.to_node(r + 1, c - 1)))
                                edges.append((self.to_node(r, c), self.to_node(r + 1, c + 1)))
                        else:
                            edges.append((self.to_node(r, c), self.to_node(r, c + 1)))
                            edges.append((self.to_node(r, c), self.to_node(r, c - 1)))
                            edges.append((self.to_node(r - 1, c), self.to_node(r, c)))
                            if self.IS_DIAGONAL:
                                edges.append((self.to_node(r, c), self.to_node(r - 1, c - 1)))
                                edges.append((self.to_node(r, c), self.to_node(r - 1, c + 1)))

                if r != 1 and r != row_end_index:
                    if c == 1 or c == column_end_index:
                        if c == 1:
                            edges.append((self.to_node(r, c), self.to_node(r + 1, c)))
                            edges.append((self.to_node(r, c), self.to_node(r - 1, c)))
                            edges.append((self.to_node(r, c), self.to_node(r, c + 1)))
                            if self.IS_DIAGONAL:
                                edges.append((self.to_node(r, c), self.to_node(r - 1, c + 1)))
                                edges.append((self.to_node(r, c), self.to_node(r + 1, c + 1)))
                        else:
                            edges.append((self.to_node(r, c), self.to_node(r + 1, c)))
                            edges.append((self.to_node(r, c), self.to_node(r - 1, c)))
                            edges.append((self.to_node(r, c), self.to_node(r, c - 1)))
                            if self.IS_DIAGONAL:
                                edges.append((self.to_node(r, c), self.to_node(r - 1, c - 1)))
                                edges.append((self.to_node(r, c), self.to_node(r + 1, c - 1)))
        # ----------------------------------------------------------------
        # Middles last
        # ----------------------------------------------------------------

        for r in range(2, row_end_index):
            for c in range(2, column_end_index):
                edges.append((self.to_node(r, c), self.to_node(r + 1, c)))
                edges.append((self.to_node(r, c), self.to_node(r - 1, c)))
                edges.append((self.to_node(r, c), self.to_node(r, c + 1)))
                edges.append((self.to_node(r, c), self.to_node(r, c - 1)))
                if self.IS_DIAGONAL:
                    edges.append((self.to_node(r, c), self.to_node(r - 1, c - 1)))
                    edges.append((self.to_node(r, c), self.to_node(r + 1, c - 1)))
                    edges.append((self.to_node(r, c), self.to_node(r - 1, c + 1)))
                    edges.append((self.to_node(r, c), self.to_node(r + 1, c + 1)))

        return edges

    def remove_edges(self, node1, node2):
        """
        Remove edges between two nodes
        :param node1:
        :param node2:
        """

        # Check removal of edges in both directions
        try:
            self.EDGES.remove((node1, node2))
        except:
            print("No edges found")

        try:
            self.EDGES.remove((node2, node1))
        except:
            print("No edges found")

    def to_node(self, r, c):
        """
        Converts a row and column to a node

        :return: Node number
        """
        return (r - 1) * self.NR_ROWS + c

    def to_coordinates(self, n):
        """
        Converts a node to a row and column coordinate

        :return: List of [Row Column] coordinates
        """
        row = int((n - 1) / self.NR_ROWS) + 1
        col = int((n - 1) % self.NR_COLS) + 1
        return [row, col]

    def manhattan_distance(self, n1, n2):
        """
        Obtains the manhattan distance between two nodes

        :return: Manhattan distance
        """
        c1 = self.to_coordinates(n1)
        c2 = self.to_coordinates(n2)

        return abs(c1[0] - c2[0]) + abs(c1[1] - c2[1])

    def getAdjacentNodes(self, n):
        """
        Obtains the adjacent nodes of a node including the node itself

        :return: List of all adjacent nodes
        """
        nodes = []

        for x, y in self.EDGES:
            if x == n or y == n:
                nodes.append(x)
                nodes.append(y)

        nodes_set = set(nodes)
        return list(nodes_set)

    def getExclusiveAdjacentNodes(self, n):
        """
        Obtains the adjacent nodes of a node without the node itself

        :return: List of all adjacent nodes
        """
        nodes = []

        for x, y in self.EDGES:
            if x == n or y == n:
                nodes.append(x)
                nodes.append(y)

        nodes_set = set(nodes)
        nodes_set.remove(n)
        return list(nodes_set)

