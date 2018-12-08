/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "SHOTSolver.h"

using namespace SHOT;

bool ReadGAMSProblem(std::string filename);
bool SolveGAMSProblem(std::string filename);

int GAMSTest(int argc, char *argv[])
{
    osoutput->AddChannel("shotlogfile");

    int defaultchoice = 1;

    int choice = defaultchoice;

    if (argc > 1)
    {
        if (sscanf(argv[1], "%d", &choice) != 1)
        {
            printf("Couldn't parse that input as a number\n");
            return -1;
        }
    }

    bool passed = true;

    switch (choice)
    {
    case 1:
        std::cout << "Starting test to read GMS files:" << std::endl;
        passed = ReadGAMSProblem("data/fo7.gms");
        std::cout << "Finished test to read GMS files." << std::endl;
    case 2:
        std::cout << "Starting test to solve a MINLP problem in GAMS syntax:" << std::endl;
        passed = SolveGAMSProblem("data/enpro48pb.gms");
        std::cout << "Finished test to solve a MINLP problem in GAMS syntax." << std::endl;
        break;
    default:
        passed = false;
        std::cout << "Test #" << choice << " does not exist!\n";
    }

    if (passed)
        return 0;
    else
        return -1;
}

// Test the reading a problem
bool ReadGAMSProblem(std::string filename)
{
    bool passed = true;

    EnvironmentPtr env(new Environment);
    env->output = OutputPtr(new Output());
    env->process = ProcessPtr(new ProcessInfo(env));
    env->settings = SettingsPtr(new Settings(env->output));
    env->tasks = TaskHandlerPtr(new TaskHandler(env));
    env->report = ReportPtr(new Report(env));
    env->model = ModelPtr(new Model(env));
    std::unique_ptr<SHOTSolver> solver(new SHOTSolver(env));

    try
    {
        if (solver->setProblem(filename))
        {
            passed = true;
        }
        else
        {
            passed = false;
        }
    }
    catch (ErrorClass &e)
    {
        std::cout << "Error: " << e.errormsg << std::endl;
        return false;
    }

    return passed;
}

bool SolveGAMSProblem(std::string filename)
{
    bool passed = true;

    EnvironmentPtr env(new Environment);
    env->output = OutputPtr(new Output());
    env->process = ProcessPtr(new ProcessInfo(env));
    env->settings = SettingsPtr(new Settings(env->output));
    env->tasks = TaskHandlerPtr(new TaskHandler(env));
    env->report = ReportPtr(new Report(env));
    env->model = ModelPtr(new Model(env));
    std::unique_ptr<SHOTSolver> solver(new SHOTSolver(env));

    //solver->updateSetting("MIP.Solver", "Dual", static_cast<int>(ES_MIPSolver::Cbc));

    try
    {
        if (solver->setProblem(filename))
        {
            passed = true;
        }
        else
        {
            passed = false;
        }
    }
    catch (ErrorClass &e)
    {
        std::cout << "Error: " << e.errormsg << std::endl;
        return false;
    }

    solver->solveProblem();
    std::string osrl = solver->getOSrL();
    std::string trace = solver->getTraceResult();
    if (!UtilityFunctions::writeStringToFile("result.osrl", osrl))
    {
        std::cout << "Could not write results to OSrL file." << std::endl;
        passed = false;
    }

    if (!UtilityFunctions::writeStringToFile("trace.trc", trace))
    {
        std::cout << "Could not write results to trace file." << std::endl;
        passed = false;
    }

    if (solver->getNumberOfPrimalSolutions() > 0)
    {
        std::cout << std::endl
                  << "Objective value: " << solver->getPrimalSolution().objValue << std::endl;
    }
    else
    {
        passed = false;
    }

    return passed;
}