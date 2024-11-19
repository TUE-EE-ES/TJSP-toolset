# TJSP Toolset
The TJSP toolset is a collection of Python scripts related to solving Transportation-Constrained Job Scheduling (TJSP) problems. The toolset can be used to replicate the results from _“Optimizing Conflict-Free-Transportation-Constrained Flexible Manufacturing Systems”_ [^fn], and used as basis for creation of new TJSP solutions. More information on the toolset can be found in [^fn]. For any questions feel free to contact: Roel van Os (r.w.m.v.os@tue.nl).

[^fn]: [_“Optimizing Conflict-Free-Transportation-Constrained Flexible Manufacturing Systems”_
van Os, R. W. M. 29 Aug 2024](https://research.tue.nl/en/studentTheses/optimizing-conflict-free-transportation-constrained-flexible-manu)


The remainder of this document provides instructions to set up the TJSP toolset correctly and briefly describes how to use it.

---
## Setup
### 1. Get IBM ILOG CPLEX Optimization Studio
The IBM ILOG CPLEX Optimization Studio is needed for obtaining the solvers that solve the CP and MP problems. More information about the tool can be found here: https://www.ibm.com/products/ilog-cplex-optimization-studio

Academics can download and install the software directly by using their institution-issued email, and following the instructions via: https://academic.ibm.com/a2mt/downloads/data_science#/.

### 2. Setting up Python Environment 
We first need to set up a Python environment to develop and run the models.
1. **Install Python:** https://www.python.org/downloads/release/python-3100/
    > **NOTE:** Python version cannot exceed 3.10 as some packages do not support newer Python versions. Additionally, make sure to install `tcl/tk/IDLE` with your Python installation.
    
2. **Configure Python Interpreter:**
    To successfully solve the optimization problems we need to set up the Python environment with the correct modules. We need a package manager like _pip_ to successfully install all packages. We can install _pip_ accordingly using the following command line commands:
   - **Windows:** `py -m ensurepip --upgrade`
   - **Linux (Ubuntu):** `sudo apt install python3-pip`

    The following packages are needed to run all the scripts in the toolset:
    1. **Install the CPLEX/DocPLEX package:** provides a Python API for creating the MP models. 

        >**NOTE:** When installing CPLEX via pip, we obtain the community edition of the package. This edition only allows us to solve small problem instances. For most of the CFTFJSSP benchmark instances, we need to full version of the CPLEX package. The setup files for this package is provided after installing the IBM ILOG CPLEX Optimizaiton Studio.

        - In your terminal application, browse to the CPLEX package install directory, by default this is: `C:\Program Files\IBM\ILOG\CPLEX_StudioXXXX\python`
        - Run the `setup.py` script:
           - **Windows:** `py setup.py install`
           - **Linux (Ubuntu):** `python3 setup.py install`
        
        Instructions to get the full CPLEX package, after we have installed IBM ILOG CPLEX Optimization Studio, can also be found here: https://www.ibm.com/docs/en/icos/20.1.0?topic=cplex-setting-up-python-api

    4. **Install Matplotlib/Numpy:**
       provides a Python API for creating the figures and handling complex problem data. You can do this using the _pip_ package manager:
        - **Windows:** `py -m pip install matplotlib`
        - **Linux (Ubuntu):** `pip install matplotlib`
---
## Provided content
This software toolset is able to solve 5 types of problems. All problem data for each problem are located in the `.data` files in the `\data` folder. An example is provided for each of the problems:
- CFRP: Solves an instance of the CFR problem.  `py`/`python3 cfrp_solve.py` 
- TJSSP: Solves an instance of the TJSSP problem.  Run Windows/Linux (Ubuntu): `py`/`python3 tjssp_solve.py`
- TFJSSP: Solves an instance of the TFJSSP problem.  Run Windows/Linux (Ubuntu): `py`/`python3 tfjssp_solve.py`
- CFTFJSSP (using LBBD): Solves an instance of the CFTFJSSP problem.  Run Windows/Linux (Ubuntu): `py`/`python3 cftfjssp_solve.py`
- FJSSP (using LBBD): Solves an instance of the FJSSP problem. Run Windows/Linux (Ubuntu):`py`/`python3 fjssp_solve.py`

The example code specifies which `.data` file is used, solves the problem instance, and produces an output figure. Feel free to change this structure or adjust the problem specification in the `.data` file.

---
## Adding new content
It is possible to create your own optimization problems using the library files located in `/library`:
1. Create a new `ModelType` to `/library/model_data.py` and edit the `/library/model_data.py` to include all data for your optimization problem. 
2. Create a new models:
   - Create new optimization problem models by creating a new class and inherit `OptimizationModel` located in `/library/optimization_model`. 
   - Create new Logic-Based Benders Decomposition (LBBD) models by creating a new class and inherit `LogicBasedBendersDecomposition` located in `/library/benders_decomposition`.
     > **Note:** The master problem and sub problem need to be of type `OptimizationModel`

For any questions or help feel free to contact: Roel van Os (r.w.m.v.os@tue.nl).

---