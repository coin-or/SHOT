#!/usr/bin/env python3
"""
Test the ex1223b problem from MINLPLib using the Python API.
This is a convex MINLP with 3 continuous and 4 binary variables.
Optimal solution: x1=0.2, x2=0.8, x3=1.907878, b4=1, b5=1, b6=0, b7=1
Optimal objective: 4.579582402436710
"""

import SHOTpy

print("\n=== Testing ex1223b problem creation and solving via Python API ===\n")

# Create solver and environment
solver = SHOTpy.Solver()
env = solver.getEnvironment()

# Create problem
problem = SHOTpy.Problem(env)
problem.name = "ex1223b"

# Create variables
x1 = SHOTpy.Variable("x1", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
x2 = SHOTpy.Variable("x2", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
x3 = SHOTpy.Variable("x3", 2, SHOTpy.VariableType.Real, 0.0, 10.0)
b4 = SHOTpy.Variable("b4", 3, SHOTpy.VariableType.Binary, 0.0, 1.0)
b5 = SHOTpy.Variable("b5", 4, SHOTpy.VariableType.Binary, 0.0, 1.0)
b6 = SHOTpy.Variable("b6", 5, SHOTpy.VariableType.Binary, 0.0, 1.0)
b7 = SHOTpy.Variable("b7", 6, SHOTpy.VariableType.Binary, 0.0, 1.0)

# Add variables to problem
problem.addVariable(x1)
problem.addVariable(x2)
problem.addVariable(x3)
problem.addVariable(b4)
problem.addVariable(b5)
problem.addVariable(b6)
problem.addVariable(b7)

# Create nonlinear objective function
# minimize (b4-1)^2 + (b5-2)^2 + (b6-1)^2 - log(1+b7) + (x1-1)^2 + (x2-2)^2 + (x3-3)^2
objective = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)

# Build objective expression using operator overloading
# (b4 - 1)^2 + (b5 - 2)^2 + (b6 - 1)^2 - log(1 + b7) + (x1 - 1)^2 + (x2 - 2)^2 + (x3 - 3)^2
obj_expr = (b4 - 1)**2 + (b5 - 2)**2 + (b6 - 1)**2 - SHOTpy.log(1 + b7) + (x1 - 1)**2 + (x2 - 2)**2 + (x3 - 3)**2

# Debug: print the expression type 
print("DEBUG: obj_expr type:", type(obj_expr))

objective.add(obj_expr)

problem.setObjective(objective)

# e1: x1 + x2 + x3 + b4 + b5 + b6 <= 5
e1 = SHOTpy.LinearConstraint(0, "e1", -SHOTpy.SHOT_DBL_MAX, 5.0)
e1.add(SHOTpy.LinearTerm(1.0, x1))
e1.add(SHOTpy.LinearTerm(1.0, x2))
e1.add(SHOTpy.LinearTerm(1.0, x3))
e1.add(SHOTpy.LinearTerm(1.0, b4))
e1.add(SHOTpy.LinearTerm(1.0, b5))
e1.add(SHOTpy.LinearTerm(1.0, b6))
problem.addConstraint(e1)

# e2: b6^2 + x1^2 + x2^2 + x3^2 <= 5.5
e2 = SHOTpy.QuadraticConstraint(1, "e2", -SHOTpy.SHOT_DBL_MAX, 5.5)
e2.add(SHOTpy.QuadraticTerm(1.0, b6, b6))
e2.add(SHOTpy.QuadraticTerm(1.0, x1, x1))
e2.add(SHOTpy.QuadraticTerm(1.0, x2, x2))
e2.add(SHOTpy.QuadraticTerm(1.0, x3, x3))
problem.addConstraint(e2)

# e3: x1 + b4 <= 1.2
e3 = SHOTpy.LinearConstraint(2, "e3", -SHOTpy.SHOT_DBL_MAX, 1.2)
e3.add(SHOTpy.LinearTerm(1.0, x1))
e3.add(SHOTpy.LinearTerm(1.0, b4))
problem.addConstraint(e3)

# e4: x2 + b5 <= 1.8
e4 = SHOTpy.LinearConstraint(3, "e4", -SHOTpy.SHOT_DBL_MAX, 1.8)
e4.add(SHOTpy.LinearTerm(1.0, x2))
e4.add(SHOTpy.LinearTerm(1.0, b5))
problem.addConstraint(e4)

# e5: x3 + b6 <= 2.5
e5 = SHOTpy.LinearConstraint(4, "e5", -SHOTpy.SHOT_DBL_MAX, 2.5)
e5.add(SHOTpy.LinearTerm(1.0, x3))
e5.add(SHOTpy.LinearTerm(1.0, b6))
problem.addConstraint(e5)

# e6: x1 + b7 <= 1.2
e6 = SHOTpy.LinearConstraint(5, "e6", -SHOTpy.SHOT_DBL_MAX, 1.2)
e6.add(SHOTpy.LinearTerm(1.0, x1))
e6.add(SHOTpy.LinearTerm(1.0, b7))
problem.addConstraint(e6)

# e7: b5^2 + x2^2 <= 1.64
e7 = SHOTpy.QuadraticConstraint(6, "e7", -SHOTpy.SHOT_DBL_MAX, 1.64)
e7.add(SHOTpy.QuadraticTerm(1.0, b5, b5))
e7.add(SHOTpy.QuadraticTerm(1.0, x2, x2))
problem.addConstraint(e7)

# e8: b6^2 + x3^2 <= 4.25
e8 = SHOTpy.QuadraticConstraint(7, "e8", -SHOTpy.SHOT_DBL_MAX, 4.25)
e8.add(SHOTpy.QuadraticTerm(1.0, b6, b6))
e8.add(SHOTpy.QuadraticTerm(1.0, x3, x3))
problem.addConstraint(e8)

# e9: b5^2 + x3^2 <= 4.64
e9 = SHOTpy.QuadraticConstraint(8, "e9", -SHOTpy.SHOT_DBL_MAX, 4.64)
e9.add(SHOTpy.QuadraticTerm(1.0, b5, b5))
e9.add(SHOTpy.QuadraticTerm(1.0, x3, x3))
problem.addConstraint(e9)

# Finalize (this now calls simplifyNonlinearExpressions like OSiL does)
problem.finalize()

print("Problem created:\n")
print(problem.toString())

# Set problem and solve
solver.setProblem(problem)

print("\nSolving...\n")

if not solver.solveProblem():
    print("Failed to solve problem!")
    exit(1)
else:
    objValue = solver.getPrimalBound()
    expectedObj = 4.579582
    
    print(f"\nSolution found:")
    print(f"  Objective value: {objValue}")
    print(f"  Expected value:  {expectedObj}")
    
    if abs(objValue - expectedObj) < 0.01:
        print("\n*** TEST PASSED: Objective matches expected value! ***")
    else:
        print("\n*** TEST FAILED: Objective differs from expected! ***")
        print(f"  Difference: {abs(objValue - expectedObj)}")
        exit(1)
