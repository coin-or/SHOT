[![Build Status](https://travis-ci.org/coin-or/SHOT.svg?branch=master)](https://travis-ci.org/coin-or/SHOT)

# The Supporting Hyperplane Optimization Toolkit (SHOT)

SHOT is a software for solving mathematical optimization problems of the mixed-integer nonlinear programming (MINLP) class. 

Originally SHOT was intended for convex MINLP problems only, but as of version 1.0 it also has functionality to solve nonconvex MINLP problems,  as a heuristic method without providing any guarantees of global optimality. SHOT can solve certain nonconvex problem types to global optimality as well, and the bounds for the objective function value are guaranteed for nonconvex problems as well.

The documentation is provided at the project website at https://www.shotsolver.dev.

## Dual bound through polyhedral (outer) approximation

SHOT is based on iteratively creating a tighter polyhedral approximation of the nonlinear feasible set by generating supporting hyperplanes or cutting planes. These linearized problems are then solved with an mixed-integer linear programming (MILP) solver such as CPLEX, Gurobi or Cbc. If CPLEX or Gurobi is used, the subproblems can also include quadratic and bilinear nonlinearities directly; then MIQP or MIQCQP subproblems are solved. 

## Primal bound using heuristics

The solution to the outer approximation problem provides a lower (dual) bound (when solving a minimization problem) to the original problem if the problem is convex. If the problem is nonconvex, convergence to the global optimal solution cannot be guaranteed (but might be achieved for certain classes of problems, cf. [this paper](http://www.optimization-online.org/DB_HTML/2020/03/7691.html). 

To get an upper (primal) bound (when solving a minimization problem) on the optimal solution SHOT utilizes the following heuristics:
- Solving nonlinear programming (NLP) problems where the integer variables have been fixed to valid values. This is done by calling an external NLP solver (e.g. Ipopt).
- By checking solutions from the MIP solver's solution pool for points that fulfill also the nonlinearities in the original MINLP problem.
- By performing root searches. 

## Termination

When the relative or absolute difference (objective gap) between the primal and dual bounds is less than a user-specified value, SHOT terminates with the current primal solution. If the original problem is convex, this is a global solution to the problem. If it is nonconvex, there is normally no guarantee that such a solution can be found, however SHOT will always in addition to the primal solution give a valid lower bound on the solution. 

## Compilation instructions

Instructions for compiling SHOT is available at the project website at https://www.shotsolver.dev. 

## Publications

**SHOT is best described in the paper:**

Lundell, A. Kronqvist, J. and Westerlund, T. The Supporting Hyperplane Optimization Toolkit: A Polyhedral Outer Approximation Based 
Convex MINLP Solver Utilizing a Single Branching Tree Approach (2018). http://www.optimization-online.org/DB_FILE/2018/06/6680.pdf

**The features for solving nonconvex MINLP problems are described in the papers:**

Lundell, A. and Kronqvist, J., Polyhedral Approximation Strategies in Nonconvex Mixed-Integer Nonlinear Programming. Optimization Online (2020). http://www.optimization-online.org/DB_HTML/2020/03/7691.html

Lundell, A. and Kronqvist, J. On Solving Nonconvex MINLP Problems with SHOT (2019). In: Le Thi H., Le H., Pham Dinh T. (editors) Optimization of Complex Systems: Theory, Models, Algorithms and Applications. WCGO 2019. Advances in Intelligent Systems and Computing, vol 991. Springer, Cham.
