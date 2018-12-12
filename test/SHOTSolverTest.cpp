/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "SHOTSolver.h"
#include "UtilityFunctions.h"

using namespace SHOT;

bool SHOTSolverReadProblem(std::string filename);
bool SHOTSolverTestOptions(bool useOSiL);

int SHOTSolverTest(int argc, char *argv[])
{
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
        std::cout << "Starting test to read OSiL files:" << std::endl;
        passed = SHOTSolverReadProblem("data/tls2.osil");
        std::cout << "Finished test to read OSiL files." << std::endl;
        break;
    case 2:
        std::cout << "Starting test to read Ampl nl files:" << std::endl;
        passed = SHOTSolverReadProblem("data/tls2.nl");
        std::cout << "Finished test to read Ampl nl files." << std::endl;
        break;
    case 3:
        std::cout << "Starting test to read and write OSoL files:" << std::endl;
        passed = SHOTSolverTestOptions(true);
        std::cout << "Finished test to read and write OSoL files." << std::endl;
        break;
    case 4:
        std::cout << "Starting test to read and write opt files:" << std::endl;
        passed = SHOTSolverTestOptions(false);
        std::cout << "Finished test to read and write opt files." << std::endl;
        break;
    default:
        passed = false;
        std::cout << "Test #" << choice << " does not exist!\n";
        break;
    }

    if (passed)
        return 0;
    else
        return -1;
}

// Test the reading a problem
bool SHOTSolverReadProblem(std::string filename)
{
    bool passed = true;

    EnvironmentPtr env(new Environment);
    env->output = OutputPtr(new Output());
    env->process = ProcessPtr(new ProcessInfo(env));
    env->settings = SettingsPtr(new Settings(env->output));
    env->tasks = TaskHandlerPtr(new TaskHandler(env));
    env->report = ReportPtr(new Report(env));
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

// Test the writing and reading of options files
bool SHOTSolverTestOptions(bool useOSiL)
{
    bool passed = true;
    EnvironmentPtr env(new Environment);
    env->output = OutputPtr(new Output());
    env->process = ProcessPtr(new ProcessInfo(env));
    env->settings = SettingsPtr(new Settings(env->output));
    env->tasks = TaskHandlerPtr(new TaskHandler(env));
    env->report = ReportPtr(new Report(env));
    std::unique_ptr<SHOTSolver> solver(new SHOTSolver(env));

    std::string filename;

    if (useOSiL)
    {
        filename = "options.xml";
    }
    else
    {
        filename = "options.opt";
    }

    if (boost::filesystem::exists(filename))
        std::remove(filename.c_str());

    try
    {
        if (useOSiL)
        {
            if (!UtilityFunctions::writeStringToFile(filename, solver->getOSoL()))
            {
                passed = false;
            }
        }
        else
        {
            if (!UtilityFunctions::writeStringToFile(filename, solver->getGAMSOptFile()))
            {
                passed = false;
            }
        }

        if (passed && !solver->setOptions(filename))
        {
            std::cout << "Could not read OSoL file." << std::endl;
            passed = false;
        }
    }
    catch (ErrorClass &e)
    {
        std::cout << "Error: " << e.errormsg << std::endl;
        passed = false;
    }

    return passed;
}
