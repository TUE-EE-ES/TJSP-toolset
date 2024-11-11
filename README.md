# READ ME

## Setting up
### 1. Get IBM ILOG CPLEX Optimization Studio
The IBM ILOG CPLEX Optimization studio is needed for obtainining the solvers that solve the CP and MP problems. More information can be found here: https://www.ibm.com/products/ilog-cplex-optimization-studio

### 2. Setting up Python Environment
We first need to setup a Python environment to develop and run the models.
1. Install Python: https://www.python.org/downloads/
> **NOTE:** Python version cannot exceed 3.10 as some packages do not support newer Python versions. Additionally, make sure to install `tcl/tk/IDLE` with your python installation. 

To successfully solve the optimization problems we need to setup the Python environment with the correct modules. We need a package manger like _pip_ to succesfully install all packages.

2. **Install the CPLEX package:** provides a Python API for creating the MP models. 

>**NOTE:** When installing CPLEX via pip, we obtain the community edition of the package. This edition only allows us to solve small problem instances. For most of the CFTFJSSP benchmark instances we need to full version of the CPLEX package.
Instructions to get the full CPLEX package, after we have installed IBM ILOG CPLEX Optimization Studio, can be found here: https://www.ibm.com/docs/en/icos/20.1.0?topic=cplex-setting-up-python-api
3. **Install the DoCPLEX python package:** provides a Python API for creating the CP models. You can do this using the _pip_ package manager:
`python -m pip install docplex`
4. **Install Matplotlib/Numpy:** provides a Python API for creating the figures and handle complex problem data. You can do this using the _pip_ package manager:
`python -m pip install matplotlib`

## Provided content
This software toolset is able to solve 5 types of problems. All problem data for each problem are located in the `.data` files in the `\data` folder. An example is provided for each of the problems:
- CFRP `python cfrp_solve.py`: Solves an instance of the CFR problem. 
- TJSSP `python tjssp_solve.py`: Solves an instance of the TJSSP problem. 
- TFJSSP `python tfjssp_solve.py`: Solves an instance of the TFJSSP problem. 
- CFTFJSSP (using LBBD) `python cftfjssp_solve.py`: Solves an instance of the CFTFJSSP problem. 
- FJSSP (using LBBD) `python fjssp_solve.py`: Solves an instance of the FJSSP problem. 

The example code specifies which `.data` file is used, solves the problem instance and produces an output figure. Feel free to change this structure or adjust the problem specification in the `.data` file.


More instructions coming soon....