/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "SHOTSolver.h"

#include <iostream>

using namespace SHOT;

bool GurobiTest1(std::string filename);

int GurobiTest(int argc, char* argv[])
{

    int defaultchoice = 1;

    int choice = defaultchoice;

    if(argc > 1)
    {
        if(sscanf(argv[1], "%d", &choice) != 1)
        {
            printf("Couldn't parse that input as a number\n");
            return -1;
        }
    }

    bool passed = true;

    switch(choice)
    {
    case 1:
        std::cout << "Starting test to solve a MINLP problem with Gurobi." << std::endl;
        passed = GurobiTest1("data/tls2.osil");
        std::cout << "Finished test to solve a MINLP problem with Gurobi." << std::endl;
        break;
    default:
        passed = false;
        std::cout << "Test #" << choice << " does not exist!\n";
    }

    if(passed)
        return 0;
    else
        return -1;
}

bool GurobiTest1(std::string filename)
{
    bool passed = true;

    EnvironmentPtr env(new Environment);
    env->output = OutputPtr(new Output());
    env->results = ResultsPtr(new Results(env));
    env->settings = SettingsPtr(new Settings(env->output));
    env->tasks = TaskHandlerPtr(new TaskHandler(env));
    env->report = ReportPtr(new Report(env));
    std::unique_ptr<SHOTSolver> solver(new SHOTSolver(env));

    solver->updateSetting("MIP.Solver", "Dual", static_cast<int>(ES_MIPSolver::Gurobi));

    try
    {
        if(solver->setProblem(filename))
        {
            passed = true;
        }
        else
        {
            passed = false;
        }
    }
    catch(ErrorClass& e)
    {
        std::cout << "Error: " << e.errormsg << std::endl;
        return false;
    }

    solver->solveProblem();
    std::string osrl = solver->getOSrL();
    std::string trace = solver->getTraceResult();
    if(!UtilityFunctions::writeStringToFile("result.osrl", osrl))
    {
        std::cout << "Could not write results to OSrL file." << std::endl;
        passed = false;
    }

    if(!UtilityFunctions::writeStringToFile("trace.trc", trace))
    {
        std::cout << "Could not write results to trace file." << std::endl;
        passed = false;
    }

    if(solver->getNumberOfPrimalSolutions() > 0)
    {
        std::cout << std::endl << "Objective value: " << solver->getPrimalSolution().objValue << std::endl;
    }
    else
    {
        passed = false;
    }

    return passed;
}