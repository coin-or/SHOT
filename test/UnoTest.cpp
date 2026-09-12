/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "../src/Solver.h"
#include "../src/Environment.h"
#include "../src/Settings.h"
#include "../src/Timing.h"
#include "../src/Utilities.h"

#include "../src/Model/Variables.h"
#include "../src/Model/Terms.h"
#include "../src/Model/Constraints.h"
#include "../src/Model/NonlinearExpressions.h"
#include "../src/Model/Problem.h"

#include "../src/NLPSolver/NLPSolverUno.h"

#ifdef HAS_IPOPT
#include "../src/NLPSolver/NLPSolverIpoptRelaxed.h"
#endif

#include <cmath>

using namespace SHOT;

namespace
{

/* Builds
 *
 *     min  (x - 4y)^2
 *     s.t. x + y == 3
 *          0.1 <= x <= 2, 0.1 <= y <= 10
 *
 * whose solution is x = 2.4, y = 0.6 with objective value 0. Both the objective Hessian and the constraint are
 * needed to get there, so a Hessian that is wrongly signed or transposed does not reach the optimum.
 */
ProblemPtr createMinimizationProblem(EnvironmentPtr env)
{
    ProblemPtr problem = std::make_shared<Problem>(env);

    auto var_x = std::make_shared<Variable>("x", E_VariableType::Real, 0.1, 3.0);
    ExpressionVariablePtr expressionVariable_x = std::make_shared<ExpressionVariable>(var_x);

    auto var_y = std::make_shared<Variable>("y", E_VariableType::Real, 0.1, 10.0);
    ExpressionVariablePtr expressionVariable_y = std::make_shared<ExpressionVariable>(var_y);

    Variables variables = { var_x, var_y };
    problem->add(variables);

    NonlinearObjectiveFunctionPtr objectiveFunction
        = std::make_shared<NonlinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);

    NonlinearExpressionPtr exprTimes
        = std::make_shared<ExpressionProduct>(std::make_shared<ExpressionConstant>(4.0), expressionVariable_y);
    NonlinearExpressionPtr exprMinus
        = std::make_shared<ExpressionSum>(expressionVariable_x, std::make_shared<ExpressionNegate>(exprTimes));
    NonlinearExpressionPtr exprPower
        = std::make_shared<ExpressionPower>(exprMinus, std::make_shared<ExpressionConstant>(2.0));

    objectiveFunction->add(exprPower);
    problem->add(objectiveFunction);

    LinearTerms linearTerms;
    linearTerms.add(std::make_shared<LinearTerm>(1.0, var_x));
    linearTerms.add(std::make_shared<LinearTerm>(1.0, var_y));
    // The constructor already stores the terms, so they must not be added a second time
    auto linearConstraint = std::make_shared<LinearConstraint>("linconstr", linearTerms, 3.0, 3.0);
    problem->add(linearConstraint);

    problem->finalize();

    return (problem);
}

/* Builds
 *
 *     max  -(x - 4y)^2
 *     s.t. x + y == 3
 *          0.1 <= x <= 3, 0.1 <= y <= 10
 *
 * the same problem as above but maximized, so the optimum is again x = 2.4, y = 0.6 with objective value 0. An
 * objective whose sense is dropped, or applied twice, does not land on that point.
 *
 * The objective is concave, so maximizing it is a convex problem. That matters because Uno solves the subproblems
 * with whichever solver it was built with, and HiGHS rejects negative curvature outright, so a nonconvex problem
 * would test the capabilities of the Uno build rather than this interface.
 */
ProblemPtr createMaximizationProblem(EnvironmentPtr env)
{
    ProblemPtr problem = std::make_shared<Problem>(env);

    auto var_x = std::make_shared<Variable>("x", E_VariableType::Real, 0.1, 3.0);
    ExpressionVariablePtr expressionVariable_x = std::make_shared<ExpressionVariable>(var_x);

    auto var_y = std::make_shared<Variable>("y", E_VariableType::Real, 0.1, 10.0);
    ExpressionVariablePtr expressionVariable_y = std::make_shared<ExpressionVariable>(var_y);

    Variables variables = { var_x, var_y };
    problem->add(variables);

    NonlinearObjectiveFunctionPtr objectiveFunction
        = std::make_shared<NonlinearObjectiveFunction>(E_ObjectiveFunctionDirection::Maximize);

    NonlinearExpressionPtr exprTimes
        = std::make_shared<ExpressionProduct>(std::make_shared<ExpressionConstant>(4.0), expressionVariable_y);
    NonlinearExpressionPtr exprMinus
        = std::make_shared<ExpressionSum>(expressionVariable_x, std::make_shared<ExpressionNegate>(exprTimes));
    NonlinearExpressionPtr exprPower
        = std::make_shared<ExpressionPower>(exprMinus, std::make_shared<ExpressionConstant>(2.0));

    objectiveFunction->add(std::make_shared<ExpressionNegate>(exprPower));
    problem->add(objectiveFunction);

    LinearTerms linearTerms;
    linearTerms.add(std::make_shared<LinearTerm>(1.0, var_x));
    linearTerms.add(std::make_shared<LinearTerm>(1.0, var_y));
    problem->add(std::make_shared<LinearConstraint>("linconstr", linearTerms, 3.0, 3.0));

    problem->finalize();

    return (problem);
}

bool isSolved(E_NLPSolutionStatus status)
{
    return (status == E_NLPSolutionStatus::Optimal || status == E_NLPSolutionStatus::Feasible);
}

} // namespace

/* Solves a convex QP-like problem whose optimum is known analytically. This checks the whole chain end to end:
   sparsity patterns, the Jacobian and Hessian value callbacks, the bounds, and reading the solution back. */
bool UnoTest1()
{
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    auto problem = createMinimizationProblem(env);
    env->problem = problem;

    auto NLPSolver = std::make_shared<NLPSolverUno>(env, problem);

    std::cout << "Solver: " << NLPSolver->getSolverDescription() << '\n';

    NLPSolver->setStartingPoint(std::vector<int>({ 0, 1 }), std::vector<double>({ 1.0, 1.0 }));

    auto status = NLPSolver->solveProblem();

    if(!isSolved(status))
    {
        std::cout << "FAILED: Uno did not solve the problem, status " << static_cast<int>(status) << '\n';
        return (false);
    }

    auto solution = NLPSolver->getSolution();

    std::cout << "Objective value: " << NLPSolver->getObjectiveValue() << '\n';
    Utilities::displayVector(solution);

    if(solution.size() != 2)
    {
        std::cout << "FAILED: expected a solution with 2 components, got " << solution.size() << '\n';
        return (false);
    }

    if(std::abs(solution.at(0) - 2.4) > 1e-4 || std::abs(solution.at(1) - 0.6) > 1e-4)
    {
        std::cout << "FAILED: expected the solution (2.4, 0.6)\n";
        passed = false;
    }

    if(std::abs(NLPSolver->getObjectiveValue()) > 1e-6)
    {
        std::cout << "FAILED: expected the objective value 0\n";
        passed = false;
    }

    return (passed);
}

/* Solves a maximization problem, which is where an objective sense that is dropped or applied twice shows up, and
   compares the result against Ipopt when it is available. The comparison is the guard against a Lagrangian Hessian
   that is transposed or signed the wrong way: such a Hessian usually still converges, but to a different point. */
bool UnoTest2()
{
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    auto problem = createMaximizationProblem(env);
    env->problem = problem;

    auto NLPSolver = std::make_shared<NLPSolverUno>(env, problem);

    NLPSolver->setStartingPoint(std::vector<int>({ 0, 1 }), std::vector<double>({ 0.5, 0.5 }));

    auto status = NLPSolver->solveProblem();

    if(!isSolved(status))
    {
        std::cout << "FAILED: Uno did not solve the maximization problem, status " << static_cast<int>(status)
                  << '\n';
        return (false);
    }

    auto solution = NLPSolver->getSolution();
    double objectiveValue = NLPSolver->getObjectiveValue();

    std::cout << "Uno objective value: " << objectiveValue << '\n';
    Utilities::displayVector(solution);

    /* The objective value must be the one of the point that was returned, in the sense of the model. Recomputing it
       from the solution catches a value that was reported with the wrong sign. */
    double recomputedObjective = problem->objectiveFunction->calculateValue(solution);

    if(std::abs(objectiveValue - recomputedObjective) > 1e-6)
    {
        std::cout << "FAILED: the reported objective value " << objectiveValue
                  << " does not match the objective at the returned point " << recomputedObjective << '\n';
        passed = false;
    }

    if(std::abs(objectiveValue) > 1e-6)
    {
        std::cout << "FAILED: expected the objective value 0\n";
        passed = false;
    }

    if(solution.size() != 2 || std::abs(solution.at(0) - 2.4) > 1e-4 || std::abs(solution.at(1) - 0.6) > 1e-4)
    {
        std::cout << "FAILED: expected the solution (2.4, 0.6)\n";
        passed = false;
    }

#ifdef HAS_IPOPT
    std::unique_ptr<Solver> ipoptSolverEnv = std::make_unique<Solver>();
    auto ipoptEnv = ipoptSolverEnv->getEnvironment();

    auto ipoptProblem = createMaximizationProblem(ipoptEnv);
    ipoptEnv->problem = ipoptProblem;

    auto ipoptNLPSolver = std::make_shared<NLPSolverIpoptRelaxed>(ipoptEnv, ipoptProblem);
    ipoptNLPSolver->setStartingPoint(std::vector<int>({ 0, 1 }), std::vector<double>({ 0.5, 0.5 }));

    if(isSolved(ipoptNLPSolver->solveProblem()))
    {
        std::cout << "Ipopt objective value: " << ipoptNLPSolver->getObjectiveValue() << '\n';

        if(std::abs(objectiveValue - ipoptNLPSolver->getObjectiveValue()) > 1e-4)
        {
            std::cout << "FAILED: Uno and Ipopt disagree on the objective value\n";
            passed = false;
        }
    }
    else
    {
        std::cout << "Ipopt did not solve the problem, skipping the comparison\n";
    }
#endif

    return (passed);
}

/* Fixing a variable collapses its bounds, which is how SHOT fixes the integers of the fixed-integer NLP, and is the
   path through Uno's handling of variables whose bounds are equal. Unfixing has to restore the original bounds. */
bool UnoTest3()
{
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    auto problem = createMinimizationProblem(env);
    env->problem = problem;

    auto NLPSolver = std::make_shared<NLPSolverUno>(env, problem);

    // With x fixed to 1, the constraint forces y = 2, so the objective is (1 - 8)^2 = 49.
    NLPSolver->fixVariables(std::vector<int>({ 0 }), std::vector<double>({ 1.0 }));

    auto status = NLPSolver->solveProblem();

    if(!isSolved(status))
    {
        std::cout << "FAILED: Uno did not solve the problem with a fixed variable, status "
                  << static_cast<int>(status) << '\n';
        return (false);
    }

    auto solution = NLPSolver->getSolution();

    std::cout << "Objective value with x fixed: " << NLPSolver->getObjectiveValue() << '\n';
    Utilities::displayVector(solution);

    if(std::abs(solution.at(0) - 1.0) > 1e-6)
    {
        std::cout << "FAILED: the fixed variable did not keep its value\n";
        passed = false;
    }

    if(std::abs(NLPSolver->getObjectiveValue() - 49.0) > 1e-4)
    {
        std::cout << "FAILED: expected the objective value 49 with x fixed to 1\n";
        passed = false;
    }

    // After unfixing, the unrestricted optimum must be found again.
    NLPSolver->unfixVariables();

    if(!isSolved(NLPSolver->solveProblem()))
    {
        std::cout << "FAILED: Uno did not solve the problem after unfixing\n";
        return (false);
    }

    if(std::abs(NLPSolver->getObjectiveValue()) > 1e-6)
    {
        std::cout << "FAILED: expected the objective value 0 after unfixing, got "
                  << NLPSolver->getObjectiveValue() << '\n';
        passed = false;
    }

    return (passed);
}

int UnoTest(int argc, char* argv[])
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
        std::cout << "Starting test to solve a constrained problem using Uno:" << std::endl;
        passed = UnoTest1();
        std::cout << "Finished test to solve a constrained problem using Uno." << std::endl;
        break;
    case 2:
        std::cout << "Starting test to solve a maximization problem using Uno:" << std::endl;
        passed = UnoTest2();
        std::cout << "Finished test to solve a maximization problem using Uno." << std::endl;
        break;
    case 3:
        std::cout << "Starting test to solve a problem with fixed variables using Uno:" << std::endl;
        passed = UnoTest3();
        std::cout << "Finished test to solve a problem with fixed variables using Uno." << std::endl;
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
