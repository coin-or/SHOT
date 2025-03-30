import shotpy

def solveProblem(problemFile, correctObjectiveValue):
        
    solver = shotpy.Solver()
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
        #print("\tSource description: " + str(solution.sourceDescription)) # not working
        print("\tPoint " + str(solution.point))
        #print("\tMax deviation " + str(solution.maxDevatingConstraintNonlinear.value)) # not working
                
    objValue = solutions[0].objValue
    
    if (abs(objValue - correctObjectiveValue) > 1e-4):
        print("Objective value is incorrect")
        return
    else: 
        print("Objective value is correct")
        
    stats = solver.getSetSolutionStatistics()
    print("Number of iterations " + str(stats.numberOfIterations))
    
    #print(solver.getResultsOSrL())
    
    #if (solver.getTerminationReason() != E_TerminationReason.AbsoluteGap): #not working
    #    print("Termination reason is AbsoluteGap")
        
if __name__ == '__main__':

    solveProblem('test/data/alan.osil', 2.925)  
    solveProblem('test/data/meanvarxsc.osil', 14.36923211)
    solveProblem('test/data/fo7_2.osil', 17.74934573)
