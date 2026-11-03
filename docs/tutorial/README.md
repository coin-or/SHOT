# SHOTpy Tutorial: Using SHOT from Python

**The Supporting Hyperplane Optimization Toolkit (SHOT)** is an open-source solver for mixed-integer nonlinear
programming (MINLP) problems that uses a combination of polyhedral outer approximation, extended supporting
hyperplanes, and other techniques. This tutorial is a guide to using **SHOTpy**, the Python interface of SHOT.

SHOTpy exposes SHOT's internal callbacks to Python, so that you can customize the behavior of the solver and
experiment with new heuristics or polyhedral approximation strategies. Beyond solving, SHOT can thus serve as a
**flexible platform for optimization research and experimentation**.

The Python interface also gives access to SHOT's internal functionality: loading models, configuring the solver,
inspecting detailed solution information, retrieving all the solutions found, evaluating the functions of a model,
and more. **The interface thus enables new ways of using SHOT in larger workflows and applications.**

SHOTpy is installed with `pip install SHOTpy` (Python 3.11 or later), which also installs SHOT and the open-source
subsolvers it uses (HiGHS, Cbc and Ipopt). Each notebook starts with a `%pip install` cell for what it needs.

The tutorial consists of **four notebooks**:

1. [Getting started](01_getting_started.ipynb)
    - Installation
    - Creating variables
    - Solving LP/MILP/MI(QC)QP/MINLP problems (linear, quadratic, exponential, logarithmic, trigonometric, square
      root, power function, monomial and signomial terms, and mixed expressions)
    - Building large problems efficiently
    - Retrieving solution details

2. [Solver features](02_solver_features.ipynb)
    - Reading problems from files and printing the problem structure
    - Configuring solver settings
    - Solver callbacks (`NewPrimalSolution`, `PrimalCandidateCheck`, `TerminationCheck`, `DualBoundUpdate`,
      `PrimalCandidateSearch`, `HyperplaneSelection`, `InteriorPointSearch`, `BoundsUpdated` and `BoundTightening`)

3. [Python workflows](03_python_workflows.ipynb)
    - SHOT in the Python ecosystem: loading data with pandas, and plotting solutions and the convergence of the
      bounds with matplotlib, for a 3D bin packing problem

4. [Advanced modeling](04_advanced_modeling.ipynb)
    - Evaluating functions, gradients and Hessians
    - Interval evaluation, a numerical convexity check and αBB underestimators

The notebooks gradually evolve from basic model construction towards building larger optimization workflows in
Python around SHOT. The progression moves from the question

> *"How do I solve a model?"*

to the broader and more practical perspective

> *"How can I use SHOT together with the Python ecosystem to build larger optimization workflows?"*

Notebooks 2 and 3 read the data files `shot_ex_jogo.osil` and `binpacking_items.csv`, so run the notebooks from
this folder.

The tutorial is written by Joakim Blomqvist (Åbo Akademi University), based on an earlier SHOTpy tutorial notebook.
