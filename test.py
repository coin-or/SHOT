
from build import shotpy

if __name__ == '__main__':
    solver = shotpy.Solver()

    solver.setProblem('test/data/alan.osil')

    solver.solveProblem()


    result = solver.getResultsOSrL()
    
    modelreturnstatus = solver.getModelReturnStatus()
    print('modelreturnstatus', modelreturnstatus)

    eterminationreason = solver.getTerminationReason()
    print('eterminationreason', eterminationreason)

    primalsolutions = solver.getPrimalSolutions()
    print('primalsolutions', primalsolutions)

    hasprimalsolution = solver.hasPrimalSolution()
    print('hasprimalsolution', hasprimalsolution)

    currentdualbound = solver.getCurrentDualBound()

    primal = solver.getPrimalBound()
    absolute = solver.getAbsoluteObjectiveGap()
    relative = solver.getRelativeObjectiveGap()

    primalsolution = solver.getPrimalSolution()


    getsetsolutionstatistics = solver.getSetSolutionStatistics() 
    print('numberofiterations', getsetsolutionstatistics.numberOfIterations)

    print(primalsolution.point)
    print(primalsolution.sourceType)
    print(primalsolution.objValue)
    print(primalsolution.displayed)

    solver.getIntSetting('test', 'test')
    solver.getBoolSetting('test', 'test')
    solver.updateSetting('test', 'test', 'test')

    # displayed
    # print(dir(test))

    # print(currentdualbound, primal, absolute, relative)


    # environment = shotpy.Environment()

    # report = shotpy.Report(environment)

    # report.outputProblemInstanceReport()





    # clas.setOptionsFromFile('test.txt')

    # clas.doc
    # clas.setOptionsFromString('teststu')




    # print(environment)
    # print(environment.report)
    # env = clas.getEnvironment()
    # print(env)
    # print(dir(clas))


    # clas.solveProblem()
    # result = clas.getResultsOSrL()
    # print(result, type(result))
    # clas.finalizeSolution()
    # help(py_solver)
    # clas.setLogFile('test.txt')

    

    # clas.setOptionsFromFile('test.txt')

    # clas.doc
    # clas.setOptionsFromString('teststu')