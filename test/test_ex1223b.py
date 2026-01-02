#!/usr/bin/env python3
"""
Test the ex1223b problem from MINLPLib using the Python API.
This is a convex MINLP with 3 continuous and 4 binary variables.
Optimal solution: x1=0.2, x2=0.8, x3=1.907878, b4=1, b5=1, b6=0, b7=1
Optimal objective: 4.579582402436710
"""

import shotpy

print("\n=== Testing ex1223b problem creation and solving via Python API ===\n")

# Create solver and environment
solver = shotpy.Solver()
env = solver.getEnvironment()

# Create problem
problem = shotpy.Problem(env)
problem.name = "ex1223b"

# Create variables
x1 = shotpy.Variable("x1", 0, shotpy.VariableType.Real, 0.0, 10.0)
x2 = shotpy.Variable("x2", 1, shotpy.VariableType.Real, 0.0, 10.0)
x3 = shotpy.Variable("x3", 2, shotpy.VariableType.Real, 0.0, 10.0)
b4 = shotpy.Variable("b4", 3, shotpy.VariableType.Binary, 0.0, 1.0)
b5 = shotpy.Variable("b5", 4, shotpy.VariableType.Binary, 0.0, 1.0)
b6 = shotpy.Variable("b6", 5, shotpy.VariableType.Binary, 0.0, 1.0)
b7 = shotpy.Variable("b7", 6, shotpy.VariableType.Binary, 0.0, 1.0)

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
objective = shotpy.NonlinearObjectiveFunction(shotpy.ObjectiveDirection.Minimize)

# Build objective expression using operator overloading
# (b4 - 1)^2 + (b5 - 2)^2 + (b6 - 1)^2 - log(1 + b7) + (x1 - 1)^2 + (x2 - 2)^2 + (x3 - 3)^2
obj_expr = (b4 - 1)**2 + (b5 - 2)**2 + (b6 - 1)**2 - shotpy.log(1 + b7) + (x1 - 1)**2 + (x2 - 2)**2 + (x3 - 3)**2

# Debug: print the expression type 
print("DEBUG: obj_expr type:", type(obj_expr))

objective.add(obj_expr)

problem.setObjective(objective)

# e1: x1 + x2 + x3 + b4 + b5 + b6 <= 5
e1 = shotpy.LinearConstraint(0, "e1", -shotpy.SHOT_DBL_MAX, 5.0)
e1.add(shotpy.LinearTerm(1.0, x1))
e1.add(shotpy.LinearTerm(1.0, x2))
e1.add(shotpy.LinearTerm(1.0, x3))
e1.add(shotpy.LinearTerm(1.0, b4))
e1.add(shotpy.LinearTerm(1.0, b5))
e1.add(shotpy.LinearTerm(1.0, b6))
problem.addConstraint(e1)

# e2: b6^2 + x1^2 + x2^2 + x3^2 <= 5.5
e2 = shotpy.QuadraticConstraint(1, "e2", -shotpy.SHOT_DBL_MAX, 5.5)
e2.add(shotpy.QuadraticTerm(1.0, b6, b6))
e2.add(shotpy.QuadraticTerm(1.0, x1, x1))
e2.add(shotpy.QuadraticTerm(1.0, x2, x2))
e2.add(shotpy.QuadraticTerm(1.0, x3, x3))
problem.addConstraint(e2)

# e3: x1 + b4 <= 1.2
e3 = shotpy.LinearConstraint(2, "e3", -shotpy.SHOT_DBL_MAX, 1.2)
e3.add(shotpy.LinearTerm(1.0, x1))
e3.add(shotpy.LinearTerm(1.0, b4))
problem.addConstraint(e3)

# e4: x2 + b5 <= 1.8
e4 = shotpy.LinearConstraint(3, "e4", -shotpy.SHOT_DBL_MAX, 1.8)
e4.add(shotpy.LinearTerm(1.0, x2))
e4.add(shotpy.LinearTerm(1.0, b5))
problem.addConstraint(e4)

# e5: x3 + b6 <= 2.5
e5 = shotpy.LinearConstraint(4, "e5", -shotpy.SHOT_DBL_MAX, 2.5)
e5.add(shotpy.LinearTerm(1.0, x3))
e5.add(shotpy.LinearTerm(1.0, b6))
problem.addConstraint(e5)

# e6: x1 + b7 <= 1.2
e6 = shotpy.LinearConstraint(5, "e6", -shotpy.SHOT_DBL_MAX, 1.2)
e6.add(shotpy.LinearTerm(1.0, x1))
e6.add(shotpy.LinearTerm(1.0, b7))
problem.addConstraint(e6)

# e7: b5^2 + x2^2 <= 1.64
e7 = shotpy.QuadraticConstraint(6, "e7", -shotpy.SHOT_DBL_MAX, 1.64)
e7.add(shotpy.QuadraticTerm(1.0, b5, b5))
e7.add(shotpy.QuadraticTerm(1.0, x2, x2))
problem.addConstraint(e7)

# e8: b6^2 + x3^2 <= 4.25
e8 = shotpy.QuadraticConstraint(7, "e8", -shotpy.SHOT_DBL_MAX, 4.25)
e8.add(shotpy.QuadraticTerm(1.0, b6, b6))
e8.add(shotpy.QuadraticTerm(1.0, x3, x3))
problem.addConstraint(e8)

# e9: b5^2 + x3^2 <= 4.64
e9 = shotpy.QuadraticConstraint(8, "e9", -shotpy.SHOT_DBL_MAX, 4.64)
e9.add(shotpy.QuadraticTerm(1.0, b5, b5))
e9.add(shotpy.QuadraticTerm(1.0, x3, x3))
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
