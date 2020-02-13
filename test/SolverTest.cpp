/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "../src/Solver.h"
#include "../src/Environment.h"
#include "../src/Structs.h"
#include "../src/Utilities.h"

#include "../src/Model/Variables.h"
#include "../src/Model/Terms.h"
#include "../src/Model/Constraints.h"
#include "../src/Model/NonlinearExpressions.h"
#include "../src/Model/Problem.h"

#include "../src/ModelingSystem/ModelingSystemOSiL.h"
#include "../src/ModelingSystem/ModelingSystemAMPL.h"

#include "../src/RootsearchMethod/RootsearchMethodBoost.h"

#include "../src/Tasks/TaskReformulateProblem.h"

using namespace SHOT;

bool ReadProblem(std::string filename)
{
    bool passed = true;

    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();

    if(solver->setProblem(filename))
    {
        passed = true;
    }
    else
    {
        passed = false;
    }

    return passed;
}

bool SolveProblem(std::string filename)
{
    bool passed = true;

    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();

    if(solver->setProblem(filename))
    {
        passed = true;
    }
    else
    {
        passed = false;
    }

    if(passed == false)
        return passed;

    solver->solveProblem();
    std::string osrl = solver->getResultsOSrL();
    std::string trace = solver->getResultsTrace();
    if(!SHOT::Utilities::writeStringToFile("result.osrl", osrl))
    {
        std::cout << "Could not write results to OSrL file." << std::endl;
        passed = false;
    }

    if(!SHOT::Utilities::writeStringToFile("trace.trc", trace))
    {
        std::cout << "Could not write results to trace file." << std::endl;
        passed = false;
    }

    if(solver->getPrimalSolutions().size() > 0)
    {
        std::cout << std::endl << "Objective value: " << solver->getPrimalSolution().objValue << std::endl;
        passed = true;
    }
    else
    {
        passed = false;
    }

    return passed;
}

bool TestRootsearch(const std::string& problemFile)
{
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(E_LogLevel::Error));

    std::cout << "Reading problem:  " << problemFile << '\n';

    if(!solver->setProblem(problemFile))
    {
        std::cout << "Error while reading problem";
        passed = false;
    }

    std::cout << env->problem << "\n\n";

    VectorDouble interiorPoint;
    interiorPoint.push_back(7.44902);
    interiorPoint.push_back(8.53506);

    VectorDouble exteriorPoint;
    exteriorPoint.push_back(20.0);
    exteriorPoint.push_back(20.0);

    std::cout << "Interior point:\n";
    Utilities::displayVector(interiorPoint);

    std::cout << "Exterior point:\n";
    Utilities::displayVector(exteriorPoint);

    auto rootsearch = std::make_unique<RootsearchMethodBoost>(env);

    auto root = rootsearch->findZero(
        interiorPoint, exteriorPoint, 100, 10e-13, 10e-3, env->problem->nonlinearConstraints, false);

    std::cout << "Root found:\n";
    Utilities::displayVector(root.first, root.second);

    exteriorPoint.clear();
    exteriorPoint.push_back(8.47199);
    exteriorPoint.push_back(20.0);

    std::cout << "Interior point:\n";
    Utilities::displayVector(interiorPoint);

    std::cout << "Exterior point:\n";
    Utilities::displayVector(exteriorPoint);

    root = rootsearch->findZero(
        interiorPoint, exteriorPoint, 100, 10e-13, 10e-3, env->problem->nonlinearConstraints, false);

    std::cout << "Root found:\n";
    Utilities::displayVector(root.first, root.second);

    exteriorPoint.clear();
    exteriorPoint.push_back(1.0);
    exteriorPoint.push_back(10.0);

    std::cout << "Interior point:\n";
    Utilities::displayVector(interiorPoint);

    std::cout << "Exterior point:\n";
    Utilities::displayVector(exteriorPoint);

    root = rootsearch->findZero(
        interiorPoint, exteriorPoint, 100, 10e-13, 10e-3, env->problem->nonlinearConstraints, false);

    std::cout << "Root found:\n";
    Utilities::displayVector(root.first, root.second);

    return passed;
}

bool TestGradient(const std::string& problemFile)
{
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(E_LogLevel::Error));

    std::cout << "Reading problem:  " << problemFile << '\n';

    if(!solver->setProblem(problemFile))
    {
        std::cout << "Error while reading problem";
        passed = false;
    }

    VectorDouble point;

    for(auto& V : env->problem->allVariables)
    {
        point.push_back((V->upperBound - V->lowerBound) / 2.0);
    }

    std::cout << "Point to evaluate gradients in:\n";
    Utilities::displayVector(point);

    for(auto& C : env->problem->numericConstraints)
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

int SolverTest(int argc, char* argv[])
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
        passed = ReadProblem("data/tls2.osil");
        std::cout << "Finished test to read OSiL files." << std::endl;
        break;
    case 2:
        std::cout << "Starting test to read NL files:" << std::endl;
        passed = ReadProblem("data/tls2.nl");
        std::cout << "Finished test to read NL files." << std::endl;
        break;
    case 3:
        std::cout << "Starting test to solve a MINLP problem in OSiL syntax:" << std::endl;
        passed = SolveProblem("data/tls2.osil");
        std::cout << "Finished test to solve a MINLP problem in OSiL syntax." << std::endl;
        break;
    case 4:
        std::cout << "Starting test to evaluate gradients in OSiL file:" << std::endl;
        passed = TestGradient("data/flay02h.osil");
        std::cout << "Finished test to evaluate gradients in OSiL file." << std::endl;
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