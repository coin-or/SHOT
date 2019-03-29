/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "../Solver.h"
#include "../Structs.h"

#include "ModelingSystemOS.h"

using namespace SHOT;

bool ReadProblemOS(std::string filename)
{
    bool passed = true;

    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();

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
    catch(Error& e)
    {
        std::cout << "Error: " << e.message << std::endl;
        return false;
    }

    return passed;
}

bool SolveProblemOS(std::string filename)
{
    bool passed = true;

    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();

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
    catch(Error& e)
    {
        std::cout << "Error: " << e.message << std::endl;
        return false;
    }

    solver->solveProblem();
    std::string osrl = solver->getResultsOSrL();
    std::string trace = solver->getResultsTrace();
    if(!SHOT::UtilityFunctions::writeStringToFile("result.osrl", osrl))
    {
        std::cout << "Could not write results to OSrL file." << std::endl;
        passed = false;
    }

    if(!SHOT::UtilityFunctions::writeStringToFile("trace.trc", trace))
    {
        std::cout << "Could not write results to trace file." << std::endl;
        passed = false;
    }

    if(solver->getPrimalSolutions().size() > 0)
    {
        std::cout << std::endl << "Objective value: " << solver->getPrimalSolution().objValue << std::endl;
    }
    else
    {
        passed = false;
    }

    return passed;
}

bool TestRootsearchOS(const std::string& problemFile)
{
    bool passed = true;

    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(ENUM_OUTPUT_LEVEL_debug));

    env->modelingSystem = std::make_shared<SHOT::ModelingSystemOS>(env);
    SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

    std::cout << "Reading problem:  " << problemFile << '\n';

    if(std::dynamic_pointer_cast<ModelingSystemOS>(env->modelingSystem)
            ->createProblem(problem, problemFile, E_OSInputFileFormat::OSiL)
        != E_ProblemCreationStatus::NormalCompletion)
    {
        std::cout << "Error while reading problem";
        passed = false;
    }
    else
    {
        env->problem = problem;
        env->reformulatedProblem = env->problem;
        std::cout << "Problem read successfully:\n\n";
        std::cout << env->problem << "\n\n";
    }

    VectorDouble interiorPoint;
    interiorPoint.push_back(7.44902);
    interiorPoint.push_back(8.53506);

    VectorDouble exteriorPoint;
    exteriorPoint.push_back(20.0);
    exteriorPoint.push_back(20.0);

    std::cout << "Interior point:\n";
    UtilityFunctions::displayVector(interiorPoint);

    std::cout << "Exterior point:\n";
    UtilityFunctions::displayVector(exteriorPoint);

    auto rootsearch = std::make_unique<LinesearchMethodBoost>(env);

    auto root = rootsearch->findZero(
        interiorPoint, exteriorPoint, 100, 10e-13, 10e-3, env->problem->nonlinearConstraints, false);

    std::cout << "Root found:\n";
    UtilityFunctions::displayVector(root.first, root.second);

    exteriorPoint.clear();
    exteriorPoint.push_back(8.47199);
    exteriorPoint.push_back(20.0);

    std::cout << "Interior point:\n";
    UtilityFunctions::displayVector(interiorPoint);

    std::cout << "Exterior point:\n";
    UtilityFunctions::displayVector(exteriorPoint);

    root = rootsearch->findZero(
        interiorPoint, exteriorPoint, 100, 10e-13, 10e-3, env->problem->nonlinearConstraints, false);

    std::cout << "Root found:\n";
    UtilityFunctions::displayVector(root.first, root.second);

    exteriorPoint.clear();
    exteriorPoint.push_back(1.0);
    exteriorPoint.push_back(10.0);

    std::cout << "Interior point:\n";
    UtilityFunctions::displayVector(interiorPoint);

    std::cout << "Exterior point:\n";
    UtilityFunctions::displayVector(exteriorPoint);

    root = rootsearch->findZero(
        interiorPoint, exteriorPoint, 100, 10e-13, 10e-3, env->problem->nonlinearConstraints, false);

    std::cout << "Root found:\n";
    UtilityFunctions::displayVector(root.first, root.second);

    return passed;
}

bool TestGradientOS(const std::string& problemFile)
{
    bool passed = true;

    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(ENUM_OUTPUT_LEVEL_debug));

    env->modelingSystem = std::make_shared<SHOT::ModelingSystemOS>(env);
    SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

    std::cout << "Reading problem: " << problemFile << '\n';

    if(std::dynamic_pointer_cast<ModelingSystemOS>(env->modelingSystem)
            ->createProblem(problem, problemFile, E_OSInputFileFormat::OSiL)
        != E_ProblemCreationStatus::NormalCompletion)
    {
        std::cout << "Error while reading problem";
        passed = false;
    }
    else
    {
        std::cout << "Problem read successfully:\n\n";
        std::cout << problem << "\n\n";
        std::cout << problem->factorableFunctionsDAG << '\n';
    }

    VectorDouble point;

    for(auto& V : problem->allVariables)
    {
        point.push_back((V->upperBound - V->lowerBound) / 2.0);
    }

    std::cout << "Point to evaluate gradients in:\n";
    UtilityFunctions::displayVector(point);

    for(auto& C : problem->numericConstraints)
    {

        std::cout << "\nCalculating gradient for constraint:\t" << C << ":\n";

        auto gradient = C->calculateGradient(point, true);

        for(auto const& G : gradient)
        {
            std::cout << G.first->name << ":  " << G.second << '\n';
        }

        std::cout << '\n';
    }

    return passed;
}

bool TestReformulateProblemOS(const std::string& problemFile)
{
    bool passed = true;

    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(ENUM_OUTPUT_LEVEL_debug));

    env->modelingSystem = std::make_shared<SHOT::ModelingSystemOS>(env);
    SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

    std::cout << "Reading problem: " << problemFile << '\n';

    if(std::dynamic_pointer_cast<ModelingSystemOS>(env->modelingSystem)
            ->createProblem(problem, problemFile, E_OSInputFileFormat::OSiL)
        != E_ProblemCreationStatus::NormalCompletion)
    {
        std::cout << "Error while reading problem";
        passed = false;
    }
    else
    {
        std::cout << "Problem read successfully:\n\n";
        std::cout << problem << "\n\n";
    }

    env->problem = problem;
    auto taskReformulate = std::make_unique<TaskReformulateProblem>(env);

    taskReformulate->run();

    std::cout << env->reformulatedProblem << std::endl;

    return passed;
}

int OSTest(int argc, char* argv[])
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
        std::cout << "Starting test to read OSiL files:" << std::endl;
        passed = ReadProblemOS("data/tls2.osil");
        std::cout << "Finished test to read OSiL files." << std::endl;
    case 2:
        std::cout << "Starting test to read NL files:" << std::endl;
        passed = ReadProblemOS("data/tls2.nl");
        std::cout << "Finished test to read NL files." << std::endl;
    case 3:
        std::cout << "Starting test to solve a MINLP problem in OSiL syntax:" << std::endl;
        passed = SolveProblemOS("data/tls2.osil");
        std::cout << "Finished test to solve a MINLP problem in OSiL syntax." << std::endl;
        break;
    case 4:
        passed = TestGradientOS("data/flay02h.osil");
        break;
    case 5:
        passed = TestReformulateProblemOS("data/synthes1.osil");
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