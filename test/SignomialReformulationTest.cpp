/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "../src/Solver.h"
#include "../src/Environment.h"
#include "../src/Report.h"
#include "../src/Settings.h"

#include "../src/Model/Variables.h"
#include "../src/Model/Terms.h"
#include "../src/Model/Constraints.h"
#include "../src/Model/NonlinearExpressions.h"
#include "../src/Model/Problem.h"

#include "../src/Tasks/TaskReformulateProblem.h"

#include <sstream>

using namespace SHOT;

bool ModelTestSignomialReformulation();

int SignomialReformulationTest(int argc, char* argv[])
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
        passed = ModelTestSignomialReformulation();
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

bool ModelTestSignomialReformulation()
{
    bool passed = true;

    // Initializing the SHOT solver class
    auto solver = std::make_unique<SHOT::Solver>();

    // Contains the environment variable unique to the created solver instance
    auto env = solver->getEnvironment();

    std::string path = "signomial-debug";

    solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(E_LogLevel::Info));
    solver->updateSetting("Debug.Enable", "Output", true);
    solver->updateSetting("Debug.Path", "Output", path);
    solver->updateSetting("TreeStrategy", "Dual", static_cast<int>(ES_TreeStrategy::MultiTree));
    solver->updateSetting("Console.Iteration.Detail", "Output", static_cast<int>(ES_IterationOutputDetail::Full));
    solver->updateSetting("MIP.SolutionLimit.Initial", "Dual", SHOT_INT_MAX);
    solver->updateSetting("Relaxation.IterationLimit", "Dual", 0);
    // solver->updateSetting("IterationLimit", "Termination", 40);
    solver->updateSetting("CutStrategy", "Dual", static_cast<int>(ES_HyperplaneCutStrategy::ECP));
    solver->updateSetting("FixedInteger.OnlyUniqueIntegerCombinations", "Primal", false);
    solver->updateSetting("FixedInteger.Use", "Primal", false);
    solver->updateSetting("FixedInteger.CallStrategy", "Primal", 0);
    solver->updateSetting("FixedInteger.Source", "Primal", 0);
    solver->updateSetting("BoundTightening.FeasibilityBased.Use", "Model", false);

    SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

    // Creating variables

    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 1.0, 7.0);
    SHOT::ExpressionVariablePtr expressionVariable_x = std::make_shared<SHOT::ExpressionVariable>(var_x);

    auto var_y = std::make_shared<SHOT::Variable>("y", 1, SHOT::E_VariableType::Integer, 1.0, 7.0);
    SHOT::ExpressionVariablePtr expressionVariable_y = std::make_shared<SHOT::ExpressionVariable>(var_y);

    SHOT::Variables variables = { var_x, var_y };
    problem->add(variables);

    SHOT::NonlinearObjectiveFunctionPtr objectiveFunction
        = std::make_shared<SHOT::NonlinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Minimize);

    SHOT::LinearTerms objLinearTerms;
    objLinearTerms.add(std::make_shared<SHOT::LinearTerm>(-3.0, var_x));
    objLinearTerms.add(std::make_shared<SHOT::LinearTerm>(1.0, var_y));
    objectiveFunction->add(objLinearTerms);
    problem->add(objectiveFunction);

    std::cout << "Objective function " << objectiveFunction << " created\n";

    SHOT::LinearTerms linearTerms1;
    linearTerms1.add(std::make_shared<SHOT::LinearTerm>(5.0, var_x));
    linearTerms1.add(std::make_shared<SHOT::LinearTerm>(1.0, var_y));
    SHOT::LinearConstraintPtr linearConstraint1
        = std::make_shared<SHOT::LinearConstraint>(0, "linconstr1", linearTerms1, -SHOT_DBL_MAX, 36);
    problem->add(linearConstraint1);

    std::cout << '\n';
    std::cout << "Linear constraint " << linearConstraint1 << " created\n";

    SHOT::LinearTerms linearTerms2;
    linearTerms2.add(std::make_shared<SHOT::LinearTerm>(0.25, var_x));
    linearTerms2.add(std::make_shared<SHOT::LinearTerm>(-1.0, var_y));
    SHOT::LinearConstraintPtr linearConstraint2
        = std::make_shared<SHOT::LinearConstraint>(0, "linconstr2", linearTerms2, -SHOT_DBL_MAX, -1.0);
    problem->add(linearConstraint2);

    std::cout << '\n';
    std::cout << "Linear constraint " << linearConstraint2 << " created\n";

    SHOT::LinearTerms linearTerms3;
    linearTerms3.add(std::make_shared<SHOT::LinearTerm>(8.0, var_x));
    linearTerms3.add(std::make_shared<SHOT::LinearTerm>(11.0, var_y));

    SHOT::QuadraticTerms quadraticTerms;
    quadraticTerms.add(std::make_shared<SHOT::QuadraticTerm>(2, var_y, var_y));

    SHOT::SignomialTerms signomialTerms;

    SHOT::SignomialElements signomialElements1;
    signomialElements1.push_back(std::make_shared<SignomialElement>(var_x, 0.5));
    signomialElements1.push_back(std::make_shared<SignomialElement>(var_y, 2.0));
    signomialTerms.add(std::make_shared<SHOT::SignomialTerm>(-2.0, signomialElements1));

    SHOT::SignomialElements signomialElements2;
    signomialElements2.push_back(std::make_shared<SignomialElement>(var_x, 1.5));
    signomialElements2.push_back(std::make_shared<SignomialElement>(var_y, 1.5));
    signomialTerms.add(std::make_shared<SHOT::SignomialTerm>(0.1, signomialElements2));

    auto nonlinearExpression = std::make_shared<ExpressionProduct>(
        std::make_shared<ExpressionConstant>(-2.0), std::make_shared<ExpressionSquareRoot>(expressionVariable_y));

    SHOT::NonlinearConstraintPtr nonlinearConstraint = std::make_shared<SHOT::NonlinearConstraint>(
        2, "nlconstr", linearTerms3, quadraticTerms, signomialTerms, nonlinearExpression, -SHOT_DBL_MAX, 39.0);
    problem->add(nonlinearConstraint);

    std::cout << '\n';
    std::cout << "Nonlinear constraint " << nonlinearConstraint << " created\n";

    problem->updateProperties();

    std::cout << '\n';
    std::cout << "Finalizing problem:\n";
    problem->finalize();

    solver->setProblem(problem);

    env->report->outputProblemInstanceReport();
    env->report->outputOptionsReport();

    passed = solver->solveProblem();

    solver->finalizeSolution();
    env->report->outputSolutionReport();

    // Writing the problem to console
    std::cout << '\n';
    std::cout << "Problem created:\n\n";
    std::cout << env->problem << '\n';

    // Writing the reformulated problem to console
    std::cout << '\n';
    std::cout << "Reformulated problem created:\n\n";
    std::cout << env->reformulatedProblem << '\n';

    return passed;
}