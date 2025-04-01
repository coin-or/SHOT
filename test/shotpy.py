
from build import shotpy

if __name__ == '__main__':
    solver = shotpy.Solver()

    # solver.setOptionsFromFile('filename')
    
    print('getSettingsAsMarkup', solver.getSettingsAsMarkup())

    solver.setLogFile('testlogfile.txt')

    solver.updateLogLevels()

    solver.setProblem('test/data/alan.osil')

    solver.solveProblem()

    solver.outputSolverHeader()
    solver.outputOptionsReport()
    solver.outputProblemInstanceReport()
    solver.outputSolutionReport()

    print('getOptionsOSol()', solver.getOptionsOSoL())

    print('getOptions()', solver.getOptions())

    print('getResultsOSrL', solver.getResultsOSrL())
    print('getResultsTrace', solver.getResultsTrace())
    print('getResultsSol', solver.getResultsSol())


    print('getCurrentDualBound', solver.getCurrentDualBound())
    print('getPrimalBound', solver.getPrimalBound())
    print('getAbsoluteObjectiveGap', solver.getAbsoluteObjectiveGap())
    print('getRelativeObjectiveGap', solver.getRelativeObjectiveGap())


    print('hasPrimalSolution', solver.hasPrimalSolution())
    primalsolution = solver.getPrimalSolution()
    print('primalsolution.point', primalsolution.point)
    print('primalsolution.sourceType', primalsolution.sourceType)
    print('primalsolution.objValue', primalsolution.objValue)
    print('primalsolution.displayed', primalsolution.displayed)
    
    primalsolutions = solver.getPrimalSolutions()
    for primalsolution in primalsolutions:
        print('primalsolution.point', primalsolution.point)
        print('primalsolution.sourceType', primalsolution.sourceType)
        print('primalsolution.objValue', primalsolution.objValue)
        print('primalsolution.displayed', primalsolution.displayed)
    
    print('getModelReturnStatus', solver.getModelReturnStatus())
    print('getTerminationReason', solver.getTerminationReason())

    getsolutionstatistics = solver.getSolutionStatistics() 
    print('numberofiterations', getsolutionstatistics.numberOfIterations)

    solver.updateSetting('name', 'category', False)
