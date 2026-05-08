import os, sys
from pathlib import Path

# Establish parent folders and add build folder to sys.path
parent = str(Path(__file__).absolute().parent.parent)
build_folder = os.path.join(parent, 'build')
sys.path.append(build_folder)

# Import SHOTpy, which in this case is SHOTpy lib file from build folder
import SHOTpy


def solveProblem(problemFile, correctObjectiveValue):
        
    solver = SHOTpy.Solver()
    loglevel = 6
    solver.updateSetting('Console.LogLevel', 'Output', loglevel)
    
   
    if (solver.getIntSetting('Console.LogLevel', 'Output') != loglevel):
        print("Failed to set log level")
        return 
    
    if (not solver.setProblem(problemFile)):
        print("Failed to read problem file from " + problemFile)
        return
    else:
        print("Problem file read from " + problemFile)
    

    solver.outputProblemInstanceReport()
    
    if (not solver.solveProblem()):
        print("Failed to solve problem in " + problemFile)
        return
    
    osrl = solver.getResultsOSrL()
    
    solutions = solver.getPrimalSolutions()
    
    if (len(solutions) == 0):
        print("No solution found")
        return  
    else :
        print("Number of solution found", len(solutions))
        
    
    for solution in solutions:
        print("Solution " + solutions.index(solution).__str__())
        print("\tObjective value " + str(solution.objValue))
        print("\tSource " + str(solution.sourceType))
        print("\tSource description: " + str(solution.sourceDescription))
        print("\tPoint " + str(solution.point))
        print("\tMax deviation " + str(solution.maxDevatingConstraintNonlinear.value))
                
    objValue = solutions[0].objValue
    
    if (abs(objValue - correctObjectiveValue) > 1e-4):
        print("Objective value is incorrect")
        return
    else: 
        print("Objective value is correct")
        
    stats = solver.getSolutionStatistics()
    print("Number of iterations " + str(stats.numberOfIterations))
    
    # print(solver.getResultsOSrL())
    
    terminationreason = solver.getTerminationReason() 
    print("Termination reason is:", terminationreason)
        
if __name__ == '__main__':

    solveProblem('data/alan.osil', 2.925)  
    solveProblem('data/meanvarxsc.osil', 14.36923211)
    solveProblem('data/synthes1.osil', 6.00975909)
