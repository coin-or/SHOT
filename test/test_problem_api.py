"""
Example demonstrating SHOT's Python API for creating and solving optimization problems.

This file shows how to:
1. Create variables with different types (Real, Integer, Binary)
2. Build expressions using natural Python operators (x + y, x * y, x**2, etc.)
3. Use nonlinear functions (exp, log, sin, cos, sqrt, etc.)
4. Create constraints with bounds
5. Set objective functions (minimize or maximize)
6. Solve and inspect results
7. Access the reformulated problem
"""

import sys
import os

# Add the build directory to path if running from source
build_dir = os.path.join(os.path.dirname(__file__), '..', 'build', 'debug')
if os.path.exists(build_dir):
    sys.path.insert(0, build_dir)

import shotpy as shot

def example_linear_problem():
    """Simple LP: minimize x + y subject to constraints"""
    print("\n=== Example 1: Linear Problem ===")
    
    # Create solver and get environment
    solver = shot.Solver()
    env = solver.getEnvironment()
    
    # Create problem
    problem = shot.Problem(env)
    problem.name = "LinearExample"
    
    # Create variables (index, name, type, lb, ub)
    x = shot.Variable("x", 0, shot.VariableType.Real, 0.0, 100.0)
    y = shot.Variable("y", 1, shot.VariableType.Real, 0.0, 100.0)
    problem.addVariable(x)
    problem.addVariable(y)
    
    # Create objective: minimize x + 2*y
    obj = shot.LinearObjectiveFunction(shot.ObjectiveDirection.Minimize)
    obj.add(shot.LinearTerm(1.0, x))
    obj.add(shot.LinearTerm(2.0, y))
    problem.setObjective(obj)
    
    # Create constraints using LinearTerms
    # Constraint: x + y >= 10  =>  10 <= x + y <= inf
    lt1 = shot.LinearTerms()
    lt1.add(shot.LinearTerm(1.0, x))
    lt1.add(shot.LinearTerm(1.0, y))
    c1 = shot.LinearConstraint(0, "c1", lt1, 10.0, 1e20)
    problem.addConstraint(c1)
    
    # Constraint: x - y <= 5  =>  -inf <= x - y <= 5
    lt2 = shot.LinearTerms()
    lt2.add(shot.LinearTerm(1.0, x))
    lt2.add(shot.LinearTerm(-1.0, y))
    c2 = shot.LinearConstraint(1, "c2", lt2, -1e20, 5.0)
    problem.addConstraint(c2)
    
    # Finalize and solve
    problem.finalize()
    print(f"Problem: {problem}")
    print(f"  Variables: {problem.properties.numberOfVariables}")
    print(f"  Linear constraints: {problem.properties.numberOfLinearConstraints}")
    
    solver.setProblem(problem)
    solver.solveProblem()
    
    print(f"  Status: {solver.getModelReturnStatus()}")
    if solver.hasPrimalSolution():
        print(f"  Objective: {solver.getPrimalBound()}")

def example_quadratic_problem():
    """QP: minimize x^2 + y^2 subject to constraints"""
    print("\n=== Example 2: Quadratic Problem ===")
    
    solver = shot.Solver()
    env = solver.getEnvironment()
    problem = shot.Problem(env)
    problem.name = "QuadraticExample"
    
    # Create variables
    x = shot.Variable("x", 0, shot.VariableType.Real, -10.0, 10.0)
    y = shot.Variable("y", 1, shot.VariableType.Real, -10.0, 10.0)
    problem.addVariable(x)
    problem.addVariable(y)
    
    # Create quadratic objective: minimize x^2 + y^2
    obj = shot.QuadraticObjectiveFunction(shot.ObjectiveDirection.Minimize)
    obj.add(shot.QuadraticTerm(1.0, x, x))  # x^2
    obj.add(shot.QuadraticTerm(1.0, y, y))  # y^2
    problem.setObjective(obj)
    
    # Constraint: x + y >= 1
    lt = shot.LinearTerms()
    lt.add(shot.LinearTerm(1.0, x))
    lt.add(shot.LinearTerm(1.0, y))
    c1 = shot.LinearConstraint(0, "sum_ge_1", lt, 1.0, 1e20)
    problem.addConstraint(c1)
    
    problem.finalize()
    print(f"Problem: {problem}")
    print(f"  Convexity: {problem.properties.convexity}")
    
    solver.setProblem(problem)
    solver.solveProblem()
    
    print(f"  Status: {solver.getModelReturnStatus()}")
    if solver.hasPrimalSolution():
        print(f"  Objective: {solver.getPrimalBound()}")

def example_nonlinear_expressions():
    """MINLP using expression operators"""
    print("\n=== Example 3: Nonlinear with Expression Operators ===")
    
    solver = shot.Solver()
    env = solver.getEnvironment()
    problem = shot.Problem(env)
    problem.name = "NonlinearExample"
    
    # Create variables
    x = shot.Variable("x", 0, shot.VariableType.Real, 0.1, 10.0)
    y = shot.Variable("y", 1, shot.VariableType.Integer, 1, 10)
    problem.addVariable(x)
    problem.addVariable(y)
    
    # Build expression using natural Python operators!
    # Objective: minimize x*y + exp(x) - log(y)
    expr = x * y + shot.exp(x) - shot.log(y)
    print(f"  Expression: {expr}")
    
    obj = shot.NonlinearObjectiveFunction(shot.ObjectiveDirection.Minimize, expr, 0.0)
    problem.setObjective(obj)
    
    # Constraint using operators: x^2 + y <= 20
    constr_expr = x**2 + y
    c1 = shot.NonlinearConstraint(0, "quad_constr", constr_expr, -1e20, 20.0)
    problem.addConstraint(c1)
    
    # Constraint: sin(x) + y >= 0
    trig_expr = shot.sin(x) + y
    c2 = shot.NonlinearConstraint(1, "trig_constr", trig_expr, 0.0, 1e20)
    problem.addConstraint(c2)
    
    problem.finalize()
    print(f"Problem: {problem}")
    print(f"  Type: MINLP={problem.properties.isMINLPProblem}")
    print(f"  Nonlinear constraints: {problem.properties.numberOfNonlinearConstraints}")
    
    solver.setProblem(problem)
    solver.solveProblem()
    
    print(f"  Status: {solver.getModelReturnStatus()}")
    if solver.hasPrimalSolution():
        sol = solver.getPrimalSolution()
        print(f"  Objective: {sol.objValue}")
        print(f"  Solution: {sol.point}")

def example_inspect_reformulation():
    """Show how to inspect the reformulated problem"""
    print("\n=== Example 4: Inspect Reformulated Problem ===")
    
    solver = shot.Solver()
    env = solver.getEnvironment()
    problem = shot.Problem(env)
    problem.name = "ReformulationExample"
    
    # Create a nonconvex problem
    x = shot.Variable("x", 0, shot.VariableType.Real, 0.0, 5.0)
    y = shot.Variable("y", 1, shot.VariableType.Real, 0.0, 5.0)
    problem.addVariable(x)
    problem.addVariable(y)
    
    # Bilinear objective (nonconvex): minimize x*y
    expr = x * y
    obj = shot.NonlinearObjectiveFunction(shot.ObjectiveDirection.Minimize, expr, 0.0)
    problem.setObjective(obj)
    
    # Simple constraint
    lt = shot.LinearTerms()
    lt.add(shot.LinearTerm(1.0, x))
    lt.add(shot.LinearTerm(1.0, y))
    c1 = shot.LinearConstraint(0, "sum_constr", lt, 2.0, 1e20)
    problem.addConstraint(c1)
    
    problem.finalize()
    
    solver.setProblem(problem)
    solver.solveProblem()
    
    # Get reformulated problem
    reformulated = solver.getReformulatedProblem()
    if reformulated:
        print(f"  Original variables: {problem.properties.numberOfVariables}")
        print(f"  Reformulated variables: {reformulated.properties.numberOfVariables}")
        print(f"  Original was convex: {problem.properties.convexity}")
        print(f"  Reformulated is convex: {reformulated.properties.convexity}")
        print(f"  Reformulated linear constraints: {reformulated.properties.numberOfLinearConstraints}")
        print(f"  Reformulated quadratic constraints: {reformulated.properties.numberOfQuadraticConstraints}")
        
        # Iterate over variables in reformulated problem
        print("  Variables in reformulated problem:")
        for var in reformulated.allVariables:
            print(f"    {var.name}: [{var.lowerBound}, {var.upperBound}]")

def example_complex_expressions():
    """Demonstrate all supported nonlinear functions"""
    print("\n=== Example 5: All Nonlinear Functions ===")
    
    solver = shot.Solver()
    env = solver.getEnvironment()
    problem = shot.Problem(env)
    
    x = shot.Variable("x", 0, shot.VariableType.Real, 0.1, 2.0)
    problem.addVariable(x)
    
    # Demonstrate various expression types
    print("  Building expressions:")
    
    # Arithmetic
    e1 = x + 1.0
    print(f"    x + 1 = {e1}")
    
    e2 = 2.0 * x
    print(f"    2*x = {e2}")
    
    e3 = x**3
    print(f"    x^3 = {e3}")
    
    e4 = x / 2.0
    print(f"    x/2 = {e4}")
    
    e5 = -x
    print(f"    -x = {e5}")
    
    # Transcendental functions
    e6 = shot.exp(x)
    print(f"    exp(x) = {e6}")
    
    e7 = shot.log(x)
    print(f"    log(x) = {e7}")
    
    e8 = shot.sqrt(x)
    print(f"    sqrt(x) = {e8}")
    
    e9 = shot.square(x)
    print(f"    square(x) = {e9}")
    
    # Trigonometric
    e10 = shot.sin(x)
    print(f"    sin(x) = {e10}")
    
    e11 = shot.cos(x)
    print(f"    cos(x) = {e11}")
    
    e12 = shot.tan(x)
    print(f"    tan(x) = {e12}")
    
    # Inverse trig
    e13 = shot.asin(x)
    print(f"    asin(x) = {e13}")
    
    e14 = shot.acos(x)  
    print(f"    acos(x) = {e14}")
    
    e15 = shot.atan(x)
    print(f"    atan(x) = {e15}")
    
    # Absolute value
    e16 = shot.abs(x)
    print(f"    abs(x) = {e16}")
    
    # Complex expression
    complex_expr = shot.exp(x) * shot.log(x + 1) + shot.sin(x)**2
    print(f"    exp(x)*log(x+1)+sin(x)^2 = {complex_expr}")
    
    # Create a problem with complex expression
    obj = shot.NonlinearObjectiveFunction(shot.ObjectiveDirection.Minimize, complex_expr, 0.0)
    problem.setObjective(obj)
    problem.finalize()
    
    print(f"\n  Problem convexity: {problem.properties.convexity}")


def example_nvs03():
    """
    Solve the nvs03 problem from MINLPLib.
    https://www.minlplib.org/py/nvs03.py
    
    Original Pyomo formulation:
        m.i1 = Var(within=Integers, bounds=(0,200), initialize=100)
        m.i2 = Var(within=Integers, bounds=(0,200), initialize=100)
        m.obj = Objective(sense=minimize, expr= (-8 + m.i1)**2 + (-2 + m.i2)**2)
        m.e1 = Constraint(expr= -0.1 * m.i1**2 + m.i2 >= 0)
        m.e2 = Constraint(expr= -0.333333333333333 * m.i1 - m.i2 >= -4.5)
    
    Known optimal solution: i1=4, i2=2
    Optimal objective: 16
    """
    print("\n=== Example: nvs03 from MINLPLib ===")
    
    solver = shot.Solver()
    env = solver.getEnvironment()
    problem = shot.Problem(env)
    problem.name = "nvs03"
    
    # Variables: i1, i2 are integers in [0, 200]
    i1 = shot.Variable("i1", 0, shot.VariableType.Integer, 0, 200)
    i2 = shot.Variable("i2", 1, shot.VariableType.Integer, 0, 200)
    problem.addVariable(i1)
    problem.addVariable(i2)
    
    # Objective: minimize (i1 - 8)^2 + (i2 - 2)^2
    obj_expr = (i1 - 8.0)**2 + (i2 - 2.0)**2
    obj = shot.NonlinearObjectiveFunction(shot.ObjectiveDirection.Minimize, obj_expr, 0.0)
    problem.setObjective(obj)
    
    # Constraint e1: -0.1 * i1^2 + i2 >= 0
    # Reformulated as convex: 0.1 * i1^2 - i2 <= 0
    e1_expr = 0.1 * (i1**2) - i2
    e1 = shot.NonlinearConstraint(0, "e1", e1_expr, shot.SHOT_DBL_MIN, 0.0)
    problem.addConstraint(e1)
    
    # Constraint e2: -1/3 * i1 - i2 >= -4.5
    # Reformulated as: 1/3 * i1 + i2 <= 4.5
    lt2 = shot.LinearTerms()
    lt2.add(shot.LinearTerm(1.0/3.0, i1))
    lt2.add(shot.LinearTerm(1.0, i2))
    e2 = shot.LinearConstraint(1, "e2", lt2, shot.SHOT_DBL_MIN, 4.5)
    problem.addConstraint(e2)
    
    problem.finalize()
    
    print(f"  Problem: {problem}")
    print(f"  Variables: {problem.properties.numberOfVariables}")
    print(f"  Integer variables: {problem.properties.numberOfIntegerVariables}")
    print(f"  Nonlinear constraints: {problem.properties.numberOfNonlinearConstraints}")
    print(f"  Linear constraints: {problem.properties.numberOfLinearConstraints}")
    print(f"  Convexity: {problem.properties.convexity}")
    
    # Solve
    solver.setProblem(problem)
    solver.solveProblem()
    
    print(f"  Status: {solver.getModelReturnStatus()}")
    if solver.hasPrimalSolution():
        print(f"  Objective value: {solver.getPrimalBound()}")
        sol = solver.getPrimalSolution()
        print(f"  Solution: i1={sol.point[0]}, i2={sol.point[1]}")


def example_ex1223b():
    """
    Solve the ex1223b problem from MINLPLib.
    https://www.minlplib.org/py/ex1223b.py
    
    This is a convex MINLP with 3 continuous and 4 binary variables.
    
    Optimal solution (from https://www.minlplib.org/ex1223b.p1.html):
        x1 = 0.2, x2 = 0.8, x3 = 1.907878402833890
        b4 = 1, b5 = 1, b6 = 0, b7 = 1
        Objective = 4.579582402436710
    
    Note: Finding the global optimum may require proper solver configuration
    and sufficient iterations. A feasible solution may be found quickly.
    """
    print("\n=== Example: ex1223b from MINLPLib ===")
    
    solver = shot.Solver()
    env = solver.getEnvironment()
    problem = shot.Problem(env)
    problem.name = "ex1223b"
    
    # Variables
    x1 = shot.Variable("x1", 0, shot.VariableType.Real, 0, 10)
    x2 = shot.Variable("x2", 1, shot.VariableType.Real, 0, 10)
    x3 = shot.Variable("x3", 2, shot.VariableType.Real, 0, 10)
    b4 = shot.Variable("b4", 3, shot.VariableType.Binary, 0, 1)
    b5 = shot.Variable("b5", 4, shot.VariableType.Binary, 0, 1)
    b6 = shot.Variable("b6", 5, shot.VariableType.Binary, 0, 1)
    b7 = shot.Variable("b7", 6, shot.VariableType.Binary, 0, 1)
    
    for v in [x1, x2, x3, b4, b5, b6, b7]:
        problem.addVariable(v)
    
    # Objective: minimize (-1 + b4)^2 + (-2 + b5)^2 + (-1 + b6)^2 - log(1 + b7)
    #                    + (-1 + x1)^2 + (-2 + x2)^2 + (-3 + x3)^2
    obj_expr = ((b4 - 1.0)**2 + (b5 - 2.0)**2 + (b6 - 1.0)**2 
                - shot.log(1.0 + b7) 
                + (x1 - 1.0)**2 + (x2 - 2.0)**2 + (x3 - 3.0)**2)
    obj = shot.NonlinearObjectiveFunction(shot.ObjectiveDirection.Minimize, obj_expr, 0.0)
    problem.setObjective(obj)
    
    # e1: x1 + x2 + x3 + b4 + b5 + b6 <= 5
    lt1 = shot.LinearTerms()
    for v in [x1, x2, x3, b4, b5, b6]:
        lt1.add(shot.LinearTerm(1.0, v))
    e1 = shot.LinearConstraint(0, "e1", lt1, shot.SHOT_DBL_MIN, 5.0)
    problem.addConstraint(e1)
    
    # e2: b6^2 + x1^2 + x2^2 + x3^2 <= 5.5
    e2_expr = b6**2 + x1**2 + x2**2 + x3**2
    e2 = shot.NonlinearConstraint(1, "e2", e2_expr, shot.SHOT_DBL_MIN, 5.5)
    problem.addConstraint(e2)
    
    # e3: x1 + b4 <= 1.2
    lt3 = shot.LinearTerms()
    lt3.add(shot.LinearTerm(1.0, x1))
    lt3.add(shot.LinearTerm(1.0, b4))
    e3 = shot.LinearConstraint(2, "e3", lt3, shot.SHOT_DBL_MIN, 1.2)
    problem.addConstraint(e3)
    
    # e4: x2 + b5 <= 1.8
    lt4 = shot.LinearTerms()
    lt4.add(shot.LinearTerm(1.0, x2))
    lt4.add(shot.LinearTerm(1.0, b5))
    e4 = shot.LinearConstraint(3, "e4", lt4, shot.SHOT_DBL_MIN, 1.8)
    problem.addConstraint(e4)
    
    # e5: x3 + b6 <= 2.5
    lt5 = shot.LinearTerms()
    lt5.add(shot.LinearTerm(1.0, x3))
    lt5.add(shot.LinearTerm(1.0, b6))
    e5 = shot.LinearConstraint(4, "e5", lt5, shot.SHOT_DBL_MIN, 2.5)
    problem.addConstraint(e5)
    
    # e6: x1 + b7 <= 1.2
    lt6 = shot.LinearTerms()
    lt6.add(shot.LinearTerm(1.0, x1))
    lt6.add(shot.LinearTerm(1.0, b7))
    e6 = shot.LinearConstraint(5, "e6", lt6, shot.SHOT_DBL_MIN, 1.2)
    problem.addConstraint(e6)
    
    # e7: b5^2 + x2^2 <= 1.64
    e7_expr = b5**2 + x2**2
    e7 = shot.NonlinearConstraint(6, "e7", e7_expr, shot.SHOT_DBL_MIN, 1.64)
    problem.addConstraint(e7)
    
    # e8: b6^2 + x3^2 <= 4.25
    e8_expr = b6**2 + x3**2
    e8 = shot.NonlinearConstraint(7, "e8", e8_expr, shot.SHOT_DBL_MIN, 4.25)
    problem.addConstraint(e8)
    
    # e9: b5^2 + x3^2 <= 4.64
    e9_expr = b5**2 + x3**2
    e9 = shot.NonlinearConstraint(8, "e9", e9_expr, shot.SHOT_DBL_MIN, 4.64)
    problem.addConstraint(e9)
    
    problem.finalize()
    
    print(f"  Problem: {problem}")
    print(f"  Variables: {problem.properties.numberOfVariables}")
    print(f"  Binary variables: {problem.properties.numberOfBinaryVariables}")
    print(f"  Nonlinear constraints: {problem.properties.numberOfNonlinearConstraints}")
    print(f"  Linear constraints: {problem.properties.numberOfLinearConstraints}")
    print(f"  Convexity: {problem.properties.convexity}")
    
    # Solve
    solver.setProblem(problem)
    solver.solveProblem()
    
    print(f"  Status: {solver.getModelReturnStatus()}")
    if solver.hasPrimalSolution():
        obj_val = solver.getPrimalBound()
        print(f"  Objective value: {obj_val:.6f}")
        sol = solver.getPrimalSolution()
        print(f"  Solution:")
        print(f"    x1 = {sol.point[0]:.6f}")
        print(f"    x2 = {sol.point[1]:.6f}")
        print(f"    x3 = {sol.point[2]:.6f}")
        print(f"    b4 = {sol.point[3]:.0f}")
        print(f"    b5 = {sol.point[4]:.0f}")
        print(f"    b6 = {sol.point[5]:.0f}")
        print(f"    b7 = {sol.point[6]:.0f}")
        
        # Verify against known optimal
        expected_obj = 4.579582
        if abs(obj_val - expected_obj) < 0.01:
            print(f"  ✓ Objective matches expected value ({expected_obj})")
        elif obj_val < 10.0:  # Close to optimal
            print(f"  ~ Feasible solution found (expected optimal: {expected_obj})")
        else:
            print(f"  ✗ Objective differs significantly from expected ({expected_obj})")


def example_flay02m():
    """
    Solve the flay02m problem from MINLPLib.
    https://www.minlplib.org/flay02m.html
    
    This is a convex MINLP layout problem: determine the optimal length and width
    of rectangular patches of land with fixed area, minimizing perimeter.
    
    14 variables (10 continuous, 4 binary), 11 constraints (9 linear, 2 signomial).
    
    Optimal solution (from https://www.minlplib.org/flay02m.p1.html):
        x3 = 5.270462766947300
        x5 = 9.486832980505140
        x6 = 9.486832980505140
        x7 = 4.216370213557840
        x8 = 5.270462766947300
        x9 = 9.486832980505140
        x10 = 9.486832980505129
        b14 = 1
        Objective = 37.947331922020602
    """
    print("\n=== Example: flay02m from MINLPLib ===")
    
    solver = shot.Solver()
    env = solver.getEnvironment()
    problem = shot.Problem(env)
    problem.name = "flay02m"
    
    # Variables (from GAMS file):
    # Positive Variables x1,x2,x3,x4,x9,x10 (with bounds)
    # Variables x5,x6,x7,x8 (with bounds, lo=1)
    # Binary Variables b11,b12,b13,b14
    
    x1 = shot.Variable("x1", 0, shot.VariableType.Real, 0, 29)
    x2 = shot.Variable("x2", 1, shot.VariableType.Real, 0, 29)
    x3 = shot.Variable("x3", 2, shot.VariableType.Real, 0, 29)
    x4 = shot.Variable("x4", 3, shot.VariableType.Real, 0, 29)
    x5 = shot.Variable("x5", 4, shot.VariableType.Real, 1, 40)
    x6 = shot.Variable("x6", 5, shot.VariableType.Real, 1, 50)
    x7 = shot.Variable("x7", 6, shot.VariableType.Real, 1, 40)
    x8 = shot.Variable("x8", 7, shot.VariableType.Real, 1, 50)
    x9 = shot.Variable("x9", 8, shot.VariableType.Real, 0, 30)
    x10 = shot.Variable("x10", 9, shot.VariableType.Real, 0, 30)
    b11 = shot.Variable("b11", 10, shot.VariableType.Binary, 0, 1)
    b12 = shot.Variable("b12", 11, shot.VariableType.Binary, 0, 1)
    b13 = shot.Variable("b13", 12, shot.VariableType.Binary, 0, 1)
    b14 = shot.Variable("b14", 13, shot.VariableType.Binary, 0, 1)
    
    for v in [x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, b11, b12, b13, b14]:
        problem.addVariable(v)
    
    # Objective: minimize 2*x9 + 2*x10
    # e1: -2*x9 - 2*x10 + objvar = 0 => objvar = 2*x9 + 2*x10
    lt_obj = shot.LinearTerms()
    lt_obj.add(shot.LinearTerm(2.0, x9))
    lt_obj.add(shot.LinearTerm(2.0, x10))
    obj = shot.LinearObjectiveFunction(shot.ObjectiveDirection.Minimize)
    obj.add(lt_obj)
    problem.setObjective(obj)
    
    # e2: -x1 - x5 + x9 >= 0  =>  x1 + x5 <= x9  =>  x1 + x5 - x9 <= 0
    lt2 = shot.LinearTerms()
    lt2.add(shot.LinearTerm(1.0, x1))
    lt2.add(shot.LinearTerm(1.0, x5))
    lt2.add(shot.LinearTerm(-1.0, x9))
    e2 = shot.LinearConstraint(0, "e2", lt2, shot.SHOT_DBL_MIN, 0.0)
    problem.addConstraint(e2)
    
    # e3: -x2 - x6 + x9 >= 0  =>  x2 + x6 - x9 <= 0
    lt3 = shot.LinearTerms()
    lt3.add(shot.LinearTerm(1.0, x2))
    lt3.add(shot.LinearTerm(1.0, x6))
    lt3.add(shot.LinearTerm(-1.0, x9))
    e3 = shot.LinearConstraint(1, "e3", lt3, shot.SHOT_DBL_MIN, 0.0)
    problem.addConstraint(e3)
    
    # e4: -x3 - x7 + x10 >= 0  =>  x3 + x7 - x10 <= 0
    lt4 = shot.LinearTerms()
    lt4.add(shot.LinearTerm(1.0, x3))
    lt4.add(shot.LinearTerm(1.0, x7))
    lt4.add(shot.LinearTerm(-1.0, x10))
    e4 = shot.LinearConstraint(2, "e4", lt4, shot.SHOT_DBL_MIN, 0.0)
    problem.addConstraint(e4)
    
    # e5: -x4 - x8 + x10 >= 0  =>  x4 + x8 - x10 <= 0
    lt5 = shot.LinearTerms()
    lt5.add(shot.LinearTerm(1.0, x4))
    lt5.add(shot.LinearTerm(1.0, x8))
    lt5.add(shot.LinearTerm(-1.0, x10))
    e5 = shot.LinearConstraint(3, "e5", lt5, shot.SHOT_DBL_MIN, 0.0)
    problem.addConstraint(e5)
    
    # e6: 40/x7 - x5 <= 0  =>  40/x7 <= x5
    # This is a signomial constraint: 40*x7^(-1) - x5 <= 0
    e6_expr = 40.0 / x7 - x5
    e6 = shot.NonlinearConstraint(4, "e6", e6_expr, shot.SHOT_DBL_MIN, 0.0)
    problem.addConstraint(e6)
    
    # e7: 50/x8 - x6 <= 0  =>  50/x8 <= x6
    e7_expr = 50.0 / x8 - x6
    e7 = shot.NonlinearConstraint(5, "e7", e7_expr, shot.SHOT_DBL_MIN, 0.0)
    problem.addConstraint(e7)
    
    # e8: x1 - x2 + x5 + 69*b11 <= 69
    lt8 = shot.LinearTerms()
    lt8.add(shot.LinearTerm(1.0, x1))
    lt8.add(shot.LinearTerm(-1.0, x2))
    lt8.add(shot.LinearTerm(1.0, x5))
    lt8.add(shot.LinearTerm(69.0, b11))
    e8 = shot.LinearConstraint(6, "e8", lt8, shot.SHOT_DBL_MIN, 69.0)
    problem.addConstraint(e8)
    
    # e9: -x1 + x2 + x6 + 79*b12 <= 79
    lt9 = shot.LinearTerms()
    lt9.add(shot.LinearTerm(-1.0, x1))
    lt9.add(shot.LinearTerm(1.0, x2))
    lt9.add(shot.LinearTerm(1.0, x6))
    lt9.add(shot.LinearTerm(79.0, b12))
    e9 = shot.LinearConstraint(7, "e9", lt9, shot.SHOT_DBL_MIN, 79.0)
    problem.addConstraint(e9)
    
    # e10: x3 - x4 + x7 + 69*b13 <= 69
    lt10 = shot.LinearTerms()
    lt10.add(shot.LinearTerm(1.0, x3))
    lt10.add(shot.LinearTerm(-1.0, x4))
    lt10.add(shot.LinearTerm(1.0, x7))
    lt10.add(shot.LinearTerm(69.0, b13))
    e10 = shot.LinearConstraint(8, "e10", lt10, shot.SHOT_DBL_MIN, 69.0)
    problem.addConstraint(e10)
    
    # e11: -x3 + x4 + x8 + 79*b14 <= 79
    lt11 = shot.LinearTerms()
    lt11.add(shot.LinearTerm(-1.0, x3))
    lt11.add(shot.LinearTerm(1.0, x4))
    lt11.add(shot.LinearTerm(1.0, x8))
    lt11.add(shot.LinearTerm(79.0, b14))
    e11 = shot.LinearConstraint(9, "e11", lt11, shot.SHOT_DBL_MIN, 79.0)
    problem.addConstraint(e11)
    
    # e12: b11 + b12 + b13 + b14 = 1
    lt12 = shot.LinearTerms()
    lt12.add(shot.LinearTerm(1.0, b11))
    lt12.add(shot.LinearTerm(1.0, b12))
    lt12.add(shot.LinearTerm(1.0, b13))
    lt12.add(shot.LinearTerm(1.0, b14))
    e12 = shot.LinearConstraint(10, "e12", lt12, 1.0, 1.0)
    problem.addConstraint(e12)
    
    problem.finalize()
    
    print(f"  Problem: {problem}")
    print(f"  Variables: {problem.properties.numberOfVariables}")
    print(f"  Binary variables: {problem.properties.numberOfBinaryVariables}")
    print(f"  Nonlinear constraints: {problem.properties.numberOfNonlinearConstraints}")
    print(f"  Linear constraints: {problem.properties.numberOfLinearConstraints}")
    print(f"  Convexity: {problem.properties.convexity}")
    
    # Print original problem
    print("\n  ORIGINAL PROBLEM:")
    print(problem)
    
    # Solve
    solver.setProblem(problem)
    solver.solveProblem()
    
    # Print reformulated problem
    print("\n  REFORMULATED PROBLEM:")
    reformulated = env.reformulatedProblem
    if reformulated:
        print(reformulated)
    
    print(f"  Status: {solver.getModelReturnStatus()}")
    if solver.hasPrimalSolution():
        obj_val = solver.getPrimalBound()
        print(f"  Objective value: {obj_val:.6f}")
        sol = solver.getPrimalSolution()
        print(f"  Solution:")
        print(f"    x1 = {sol.point[0]:.6f}")
        print(f"    x2 = {sol.point[1]:.6f}")
        print(f"    x3 = {sol.point[2]:.6f}")
        print(f"    x4 = {sol.point[3]:.6f}")
        print(f"    x5 = {sol.point[4]:.6f}")
        print(f"    x6 = {sol.point[5]:.6f}")
        print(f"    x7 = {sol.point[6]:.6f}")
        print(f"    x8 = {sol.point[7]:.6f}")
        print(f"    x9 = {sol.point[8]:.6f}")
        print(f"    x10 = {sol.point[9]:.6f}")
        print(f"    b11 = {sol.point[10]:.0f}")
        print(f"    b12 = {sol.point[11]:.0f}")
        print(f"    b13 = {sol.point[12]:.0f}")
        print(f"    b14 = {sol.point[13]:.0f}")
        
        # Verify against known optimal
        expected_obj = 37.947332
        if abs(obj_val - expected_obj) < 0.01:
            print(f"  ✓ Objective matches expected value ({expected_obj})")
        elif obj_val < 50.0:  # Reasonable solution
            print(f"  ~ Feasible solution found (expected optimal: {expected_obj})")
        else:
            print(f"  ✗ Objective differs significantly from expected ({expected_obj})")


def example_flay02m_expr():
    """
    Solve the flay02m problem from MINLPLib using expression-based constraint construction.
    https://www.minlplib.org/flay02m.html
    
    Same as example_flay02m but uses operator overloads on variables instead of LinearTerms.
    This demonstrates a more Pythonic way to build constraints using expressions:
        -1*x1 + 1*x2 + 1*x6 + 79*b12
    instead of:
        lt.add(LinearTerm(-1.0, x1))
        lt.add(LinearTerm(1.0, x2))
        ...
    
    Note: We use NonlinearConstraint for all expression-based constraints.
    SHOT will automatically detect if the constraint is actually linear/quadratic.
    
    Optimal objective = 37.947332
    """
    print("\n=== Example: flay02m using expression operators ===")
    
    solver = shot.Solver()
    env = solver.getEnvironment()
    problem = shot.Problem(env)
    problem.name = "flay02m_expr"
    
    # Variables
    x1 = shot.Variable("x1", 0, shot.VariableType.Real, 0, 29)
    x2 = shot.Variable("x2", 1, shot.VariableType.Real, 0, 29)
    x3 = shot.Variable("x3", 2, shot.VariableType.Real, 0, 29)
    x4 = shot.Variable("x4", 3, shot.VariableType.Real, 0, 29)
    x5 = shot.Variable("x5", 4, shot.VariableType.Real, 1, 40)
    x6 = shot.Variable("x6", 5, shot.VariableType.Real, 1, 50)
    x7 = shot.Variable("x7", 6, shot.VariableType.Real, 1, 40)
    x8 = shot.Variable("x8", 7, shot.VariableType.Real, 1, 50)
    x9 = shot.Variable("x9", 8, shot.VariableType.Real, 0, 30)
    x10 = shot.Variable("x10", 9, shot.VariableType.Real, 0, 30)
    b11 = shot.Variable("b11", 10, shot.VariableType.Binary, 0, 1)
    b12 = shot.Variable("b12", 11, shot.VariableType.Binary, 0, 1)
    b13 = shot.Variable("b13", 12, shot.VariableType.Binary, 0, 1)
    b14 = shot.Variable("b14", 13, shot.VariableType.Binary, 0, 1)
    
    for v in [x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, b11, b12, b13, b14]:
        problem.addVariable(v)
    
    # Objective: minimize 2*x9 + 2*x10
    # Note: For linear objectives, use LinearTerms to ensure correct reformulation.
    # Expression-based objectives are treated as nonlinear and may not reformulate correctly.
    lt_obj = shot.LinearTerms()
    lt_obj.add(shot.LinearTerm(2.0, x9))
    lt_obj.add(shot.LinearTerm(2.0, x10))
    obj = shot.LinearObjectiveFunction(shot.ObjectiveDirection.Minimize)
    obj.add(lt_obj)
    problem.setObjective(obj)
    
    # All constraints using expression operators!
    # Using NonlinearConstraint - SHOT will detect the actual constraint type.
    
    # e2: x1 + x5 - x9 <= 0
    e2 = shot.NonlinearConstraint(0, "e2", 1.0*x1 + 1.0*x5 - 1.0*x9, shot.SHOT_DBL_MIN, 0.0)
    problem.addConstraint(e2)
    
    # e3: x2 + x6 - x9 <= 0
    e3 = shot.NonlinearConstraint(1, "e3", 1.0*x2 + 1.0*x6 - 1.0*x9, shot.SHOT_DBL_MIN, 0.0)
    problem.addConstraint(e3)
    
    # e4: x3 + x7 - x10 <= 0
    e4 = shot.NonlinearConstraint(2, "e4", 1.0*x3 + 1.0*x7 - 1.0*x10, shot.SHOT_DBL_MIN, 0.0)
    problem.addConstraint(e4)
    
    # e5: x4 + x8 - x10 <= 0
    e5 = shot.NonlinearConstraint(3, "e5", 1.0*x4 + 1.0*x8 - 1.0*x10, shot.SHOT_DBL_MIN, 0.0)
    problem.addConstraint(e5)
    
    # e6: 40/x7 - x5 <= 0 (truly nonlinear - signomial)
    e6 = shot.NonlinearConstraint(4, "e6", 40.0/x7 - x5, shot.SHOT_DBL_MIN, 0.0)
    problem.addConstraint(e6)
    
    # e7: 50/x8 - x6 <= 0 (truly nonlinear - signomial)
    e7 = shot.NonlinearConstraint(5, "e7", 50.0/x8 - x6, shot.SHOT_DBL_MIN, 0.0)
    problem.addConstraint(e7)
    
    # e8: x1 - x2 + x5 + 69*b11 <= 69
    e8 = shot.NonlinearConstraint(6, "e8", 1.0*x1 - 1.0*x2 + 1.0*x5 + 69.0*b11, shot.SHOT_DBL_MIN, 69.0)
    problem.addConstraint(e8)
    
    # e9: -x1 + x2 + x6 + 79*b12 <= 79
    e9 = shot.NonlinearConstraint(7, "e9", -1.0*x1 + 1.0*x2 + 1.0*x6 + 79.0*b12, shot.SHOT_DBL_MIN, 79.0)
    problem.addConstraint(e9)
    
    # e10: x3 - x4 + x7 + 69*b13 <= 69
    e10 = shot.NonlinearConstraint(8, "e10", 1.0*x3 - 1.0*x4 + 1.0*x7 + 69.0*b13, shot.SHOT_DBL_MIN, 69.0)
    problem.addConstraint(e10)
    
    # e11: -x3 + x4 + x8 + 79*b14 <= 79
    e11 = shot.NonlinearConstraint(9, "e11", -1.0*x3 + 1.0*x4 + 1.0*x8 + 79.0*b14, shot.SHOT_DBL_MIN, 79.0)
    problem.addConstraint(e11)
    
    # e12: b11 + b12 + b13 + b14 = 1
    e12 = shot.NonlinearConstraint(10, "e12", 1.0*b11 + 1.0*b12 + 1.0*b13 + 1.0*b14, 1.0, 1.0)
    problem.addConstraint(e12)
    
    problem.finalize()
    
    print(f"  Problem: {problem}")
    print(f"  Variables: {problem.properties.numberOfVariables}")
    print(f"  Binary variables: {problem.properties.numberOfBinaryVariables}")
    print(f"  Nonlinear constraints: {problem.properties.numberOfNonlinearConstraints}")
    print(f"  Linear constraints: {problem.properties.numberOfLinearConstraints}")
    print(f"  Convexity: {problem.properties.convexity}")
    
    # Print original problem
    print("\n  ORIGINAL PROBLEM:")
    print(problem)
    
    # Solve
    solver.setProblem(problem)
    solver.solveProblem()
    
    # Print reformulated problem
    print("\n  REFORMULATED PROBLEM:")
    reformulated = env.reformulatedProblem
    if reformulated:
        print(reformulated)
    
    print(f"  Status: {solver.getModelReturnStatus()}")
    if solver.hasPrimalSolution():
        obj_val = solver.getPrimalBound()
        print(f"  Objective value: {obj_val:.6f}")
        sol = solver.getPrimalSolution()
        print(f"  Solution:")
        print(f"    x3 = {sol.point[2]:.6f}, x5 = {sol.point[4]:.6f}, x6 = {sol.point[5]:.6f}")
        print(f"    x7 = {sol.point[6]:.6f}, x8 = {sol.point[7]:.6f}")
        print(f"    x9 = {sol.point[8]:.6f}, x10 = {sol.point[9]:.6f}")
        print(f"    b14 = {sol.point[13]:.0f}")
        
        # Verify against known optimal
        expected_obj = 37.947332
        if abs(obj_val - expected_obj) < 0.01:
            print(f"  ✓ Objective matches expected value ({expected_obj})")
        elif obj_val < 50.0:  # Reasonable solution
            print(f"  ~ Feasible solution found (expected optimal: {expected_obj})")
        else:
            print(f"  ✗ Objective differs significantly from expected ({expected_obj})")


if __name__ == "__main__":
    print("SHOT Python API Examples")
    print("=" * 50)
    
    # Run examples
    example_linear_problem()
    example_quadratic_problem()
    # example_nonlinear_expressions()  # May have issues with some SHOT configurations
    # example_inspect_reformulation()  # May have issues with some SHOT configurations
    example_complex_expressions()
    example_nvs03()
    example_ex1223b()
    example_flay02m()
    example_flay02m_expr()
