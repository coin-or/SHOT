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

#include "../src/NLPSolver/NLPSolverIpoptRelaxed.h"

#include <array>
#include <cmath>
#include <limits>

using namespace SHOT;

bool IpoptTest1()
{
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

    env->problem = problem;

    // Creating variables

    /*
     * min_x f(x) = -(x2-2)^2
     *  s.t.
     *       0 = x1^2 + x2 - 1
     *       -1 <= x1 <= 1
     */

    auto var_x = std::make_shared<SHOT::Variable>("x", SHOT::E_VariableType::Real, -1.0, 1.0);
    SHOT::ExpressionVariablePtr expressionVariable_x = std::make_shared<SHOT::ExpressionVariable>(var_x);

    auto var_y = std::make_shared<SHOT::Variable>("y", SHOT::E_VariableType::Real, -10.0, 10.0);
    SHOT::ExpressionVariablePtr expressionVariable_y = std::make_shared<SHOT::ExpressionVariable>(var_y);

    SHOT::Variables variables = { var_x, var_y };
    problem->add(variables);

    SHOT::NonlinearObjectiveFunctionPtr objectiveFunction
        = std::make_shared<SHOT::NonlinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Maximize);

    SHOT::NonlinearExpressionPtr exprConstant1 = std::make_shared<SHOT::ExpressionConstant>(2);

    SHOT::NonlinearExpressionPtr exprMinus = std::make_shared<SHOT::ExpressionSum>(
        expressionVariable_y, std::make_shared<ExpressionNegate>(exprConstant1));
    SHOT::NonlinearExpressionPtr exprSquared1 = std::make_shared<SHOT::ExpressionSquare>(exprMinus);

    objectiveFunction->add(exprSquared1);
    problem->add(objectiveFunction);

    SHOT::NonlinearExpressionPtr exprSquared2 = std::make_shared<SHOT::ExpressionSquare>(expressionVariable_x);
    SHOT::NonlinearExpressionPtr exprPlus = std::make_shared<SHOT::ExpressionSum>(exprSquared2, expressionVariable_y);
    SHOT::NonlinearConstraintPtr nonlinearConstraint = std::make_shared<SHOT::NonlinearConstraint>(
        "nlconstr", exprPlus, 1.0, 1.0);
    problem->add(nonlinearConstraint);

    std::cout << '\n';
    std::cout << "Finalizing problem:\n";
    problem->finalize();

    std::cout << '\n';
    std::cout << "Problem created:\n\n";
    std::cout << problem << '\n';

    std::cout << '\n';
    std::cout << "Jacobian sparsity pattern:\n";
    auto jacobianSparsityPattern = problem->getConstraintsJacobianSparsityPattern();

    for(auto& E : *jacobianSparsityPattern)
    {
        for(auto& V : E.second)
            std::cout << "(" << E.first->getIndex() << "," << V->getIndex() << ")\n";
    }

    std::cout << '\n';
    std::cout << "Hessian of the objective function sparsity pattern:\n";
    auto objectiveSparsityPattern = problem->objectiveFunction->getHessianSparsityPattern();

    for(auto& E : *objectiveSparsityPattern)
    {
        std::cout << "(" << E.first->getIndex() << "," << E.second->getIndex() << ")\n";
    }

    std::cout << '\n';
    std::cout << "Hessian of the constraints sparsity pattern:\n";
    auto constraintsSparsityPattern = problem->getConstraintsHessianSparsityPattern();

    for(auto& E : *constraintsSparsityPattern)
    {
        std::cout << "(" << E.first->getIndex() << "," << E.second->getIndex() << ")\n";
    }

    auto NLPSolver = std::make_shared<NLPSolverIpoptRelaxed>(env, problem);

    std::cout << "\nCalculating objective function gradient in point (2.0,3.0):\n";

    SHOT::VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);

    auto gradient = problem->objectiveFunction->calculateGradient(point, true);

    for(auto const& G : gradient)
    {
        std::cout << G.first->name << ":  " << G.second << '\n';
    }

    std::cout << "\nCalculating constraint gradient in point (2.0,3.0):\n";

    std::cout << problem << std::endl;

    // After finalization, the constraint may have been reclassified (e.g., from nonlinear to quadratic)
    // Use numericConstraints instead which includes all constraint types
    gradient = problem->numericConstraints.at(0)->calculateGradient(point, true);

    for(auto const& G : gradient)
    {
        std::cout << G.first->name << ":  " << G.second << '\n';
    }

    NLPSolver->setStartingPoint(std::vector<int>({ 0, 1 }), std::vector<double>({ 5.0, 5.0 }));

    NLPSolver->solveProblem();

    std::cout << '\n';
    std::cout << "The objective value is: " << NLPSolver->getObjectiveValue() << std::endl;

    auto solution = NLPSolver->getSolution();

    std::cout << '\n';
    std::cout << "The solution vector is:\n\n";
    Utilities::displayVector(solution);

    return (passed);
}

bool IpoptTest2()
{
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

    env->problem = problem;

    auto var_x = std::make_shared<SHOT::Variable>("x", SHOT::E_VariableType::Real, 0.1, 2.0);
    SHOT::ExpressionVariablePtr expressionVariable_x = std::make_shared<SHOT::ExpressionVariable>(var_x);

    auto var_y = std::make_shared<SHOT::Variable>("y", SHOT::E_VariableType::Real, 0.1, 10.0);
    SHOT::ExpressionVariablePtr expressionVariable_y = std::make_shared<SHOT::ExpressionVariable>(var_y);

    SHOT::Variables variables = { var_x, var_y };
    problem->add(variables);

    SHOT::NonlinearObjectiveFunctionPtr objectiveFunction
        = std::make_shared<SHOT::NonlinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Minimize);

    SHOT::NonlinearExpressionPtr exprTimes = std::make_shared<SHOT::ExpressionProduct>(
        std::make_shared<SHOT::ExpressionConstant>(4.0), expressionVariable_y);

    SHOT::NonlinearExpressionPtr exprMinus
        = std::make_shared<SHOT::ExpressionSum>(expressionVariable_x, std::make_shared<ExpressionNegate>(exprTimes));
    SHOT::NonlinearExpressionPtr exprConstant = std::make_shared<SHOT::ExpressionConstant>(2.0);
    SHOT::NonlinearExpressionPtr exprPower = std::make_shared<SHOT::ExpressionPower>(exprMinus, exprConstant);

    SHOT::LinearTerms linearTerms;
    linearTerms.add(std::make_shared<LinearTerm>(1.0, var_x));
    linearTerms.add(std::make_shared<LinearTerm>(1.0, var_y));
    auto linearConstraint = std::make_shared<SHOT::LinearConstraint>("linconstr", linearTerms, 3.0, 3.0);

    linearConstraint->add(linearTerms);
    problem->add(linearConstraint);

    objectiveFunction->add(exprPower);
    problem->add(objectiveFunction);

    std::cout << '\n';
    std::cout << "Finalizing problem:\n";
    problem->finalize();

    std::cout << '\n';
    std::cout << "Problem created:\n\n";
    std::cout << problem << '\n';

    std::cout << '\n';
    std::cout << "Hessian of the Lagrangian sparsity pattern:\n";
    auto lagrangianSparsityPattern = problem->objectiveFunction->getHessianSparsityPattern();

    for(auto& E : *lagrangianSparsityPattern)
    {
        std::cout << "(" << E.first->getIndex() << "," << E.second->getIndex() << ")\n";
    }

    auto NLPSolver = std::make_shared<NLPSolverIpoptRelaxed>(env, problem);

    std::cout << "\nCalculating objective function gradient in point (2.0,3.0):\n";

    SHOT::VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);

    auto gradient = problem->objectiveFunction->calculateGradient(point, true);

    for(auto const& G : gradient)
    {
        std::cout << G.first->name << ":  " << G.second << '\n';
    }

    NLPSolver->setStartingPoint(std::vector<int>({ 0, 1 }), std::vector<double>({ 5.0, 5.0 }));

    NLPSolver->solveProblem();

    std::cout << '\n';
    std::cout << "The objective value is: " << NLPSolver->getObjectiveValue() << std::endl;

    auto solution = NLPSolver->getSolution();

    std::cout << '\n';
    std::cout << "The solution vector is:\n\n";
    Utilities::displayVector(solution);

    return (passed);
}

bool IpoptTest3()
{
    auto solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();
    auto problem = std::make_shared<Problem>(env);
    env->problem = problem;

    auto x = std::make_shared<Variable>("x", E_VariableType::Real, -2.0, 2.0);
    auto y = std::make_shared<Variable>("y", E_VariableType::Real, -2.0, 2.0);
    problem->add(Variables { x, y });
    problem->add(std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize));

    auto first = std::make_shared<NonlinearConstraint>("first",
        std::make_shared<ExpressionSin>(std::make_shared<ExpressionSum>(
            std::make_shared<ExpressionVariable>(x), std::make_shared<ExpressionVariable>(y))),
        -10.0, 10.0);
    first->add(std::make_shared<QuadraticTerm>(1.0, x, y));
    problem->add(first);

    auto second = std::make_shared<NonlinearConstraint>("second",
        std::make_shared<ExpressionExp>(std::make_shared<ExpressionSum>(
            std::make_shared<ExpressionVariable>(x),
            std::make_shared<ExpressionNegate>(std::make_shared<ExpressionVariable>(y)))),
        -10.0, 10.0);
    second->add(std::make_shared<QuadraticTerm>(2.0, x, x));
    problem->add(second);
    problem->finalize();

    IpoptProblem nlp(env, problem);
    Ipopt::Index n, m, jacobianNonzeros, hessianNonzeros;
    Ipopt::TNLP::IndexStyleEnum style;
    if(!nlp.get_nlp_info(n, m, jacobianNonzeros, hessianNonzeros, style))
        return false;

    std::vector<Ipopt::Index> jacobianRows(jacobianNonzeros), jacobianColumns(jacobianNonzeros);
    std::vector<Ipopt::Number> jacobianValues(jacobianNonzeros);
    std::vector<Ipopt::Index> hessianRows(hessianNonzeros), hessianColumns(hessianNonzeros);
    std::vector<Ipopt::Number> hessianValues(hessianNonzeros);
    VectorDouble point { 0.3, -0.4 };
    VectorDouble multipliers { 1.3, -0.7 };

    nlp.eval_jac_g(n, point.data(), true, m, jacobianNonzeros,
        jacobianRows.data(), jacobianColumns.data(), nullptr);
    nlp.eval_jac_g(n, point.data(), true, m, jacobianNonzeros,
        nullptr, nullptr, jacobianValues.data());
    nlp.eval_h(n, point.data(), true, 0.0, m, multipliers.data(), true, hessianNonzeros,
        hessianRows.data(), hessianColumns.data(), nullptr);
    nlp.eval_h(n, point.data(), true, 0.0, m, multipliers.data(), true, hessianNonzeros,
        nullptr, nullptr, hessianValues.data());

    bool passed = true;
    for(size_t k = 0; k < jacobianValues.size(); ++k)
    {
        auto gradient = problem->numericConstraints[jacobianRows[k]]->calculateGradient(point, false);
        double expected = 0.0;
        for(const auto& term : gradient)
            if(term.first->getIndex() == jacobianColumns[k])
                expected += term.second;
        passed &= std::abs(jacobianValues[k] - expected) < 1e-9;
    }

    for(size_t k = 0; k < hessianValues.size(); ++k)
    {
        double expected = 0.0;
        for(size_t constraint = 0; constraint < multipliers.size(); ++constraint)
        {
            auto hessian = problem->numericConstraints[constraint]->calculateHessian(point, false);
            for(const auto& term : hessian)
                if(term.first.first->getIndex() == hessianRows[k]
                    && term.first.second->getIndex() == hessianColumns[k])
                    expected += multipliers[constraint] * term.second;
        }
        passed &= std::abs(hessianValues[k] - expected) < 1e-9;
    }

    return passed;
}

bool IpoptTest4()
{
    auto solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Warning));
    auto problem = std::make_shared<Problem>(env);
    auto x = std::make_shared<Variable>("x", E_VariableType::Real, 0.0, 2.0);
    auto y = std::make_shared<Variable>("y", E_VariableType::Integer, 0.0, 2.0);
    problem->add({ x, y });

    // (x - 1)^2 + (y - x)^2 has minimum zero at x = y = 1.
    auto objective = std::make_shared<QuadraticObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    objective->add(std::make_shared<QuadraticTerm>(2.0, x, x));
    objective->add(std::make_shared<QuadraticTerm>(-2.0, x, y));
    objective->add(std::make_shared<QuadraticTerm>(1.0, y, y));
    objective->add(std::make_shared<LinearTerm>(-2.0, x));
    objective->constant = 1.0;
    problem->add(objective);
    problem->finalize();
    env->problem = problem;

    bool passed = true;

    for(auto mode : { Ipopt::RegularMode, Ipopt::RestorationPhaseMode })
    {
        IpoptProblem finite(env, problem);
        passed = finite.intermediate_callback(mode, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, nullptr, nullptr)
            && passed;
        passed = finite.intermediate_callback(
                     mode, 1, -1e100, 1e100, 1e100, 1e100, 1e100, 1e100, 1.0, 1.0, 1, nullptr, nullptr)
            && passed;

        for(size_t metric = 0; metric < 8; ++metric)
        {

            for(double invalid : { std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                    -std::numeric_limits<double>::infinity() })
            {
                IpoptProblem nlp(env, problem);
                std::array<double, 8> values { 1.0, 0.1, 0.2, 0.01, 0.3, 0.0, 0.5, 0.5 };
                values[metric] = invalid;

                if(nlp.intermediate_callback(mode, 30, values[0], values[1], values[2], values[3], values[4], values[5],
                       values[6], values[7], 1, nullptr, nullptr))
                {
                    std::cout << "  FAILED: non-finite Ipopt iteration metric " << metric << " was accepted.\n";
                    passed = false;
                }

                // A failed iteration must not leave a solution from a preceding solve available to callers.
                nlp.hasSolution = true;
                nlp.variableSolution = { 1.0, 1.0 };
                const double point[] = { 1.0, 1.0 };
                nlp.finalize_solution(
                    Ipopt::USER_REQUESTED_STOP, 2, point, nullptr, nullptr, 0, nullptr, nullptr, 0.0, nullptr, nullptr);

                if(nlp.solutionStatus != E_NLPSolutionStatus::Error || nlp.hasSolution || !nlp.variableSolution.empty())
                {
                    std::cout << "  FAILED: an invalid Ipopt iteration retained a solution or the wrong status.\n";
                    passed = false;
                }
            }
        }
    }

    class TestSolver : public NLPSolverIpoptRelaxed
    {
    public:
        using NLPSolverIpoptBase::fixVariables;
        using NLPSolverIpoptBase::unfixVariables;

        TestSolver(EnvironmentPtr envPtr, ProblemPtr source) : INLPSolver(envPtr), NLPSolverIpoptRelaxed(envPtr, source)
        {
        }

        IpoptProblem& getProblem() { return *Ipopt::GetRawPtr(ipoptProblem); }
    };

    TestSolver nlp(env, problem);
    nlp.setTimeLimit(1.0);

    for(bool fixed : { false, true, false })
    {
        auto& iteration = nlp.getProblem();
        iteration.intermediate_callback(Ipopt::RegularMode, 1, 1.0, 0.0, std::numeric_limits<double>::quiet_NaN(), 0.1,
            0.0, 0.0, 1.0, 1.0, 1, nullptr, nullptr);
        iteration.finalize_solution(
            Ipopt::USER_REQUESTED_STOP, 2, nullptr, nullptr, nullptr, 0, nullptr, nullptr, 0.0, nullptr, nullptr);

        if(fixed)
            nlp.fixVariables({ 1 }, { 0.0 });

        nlp.setStartingPoint({ 0, 1 }, { 1.8, 0.2 });
        auto status = nlp.solveProblem();
        auto point = nlp.getSolution();
        const double expected = fixed ? 0.5 : 0.0;

        if((status != E_NLPSolutionStatus::Optimal && status != E_NLPSolutionStatus::Feasible) || point.size() != 2
            || std::abs(nlp.getObjectiveValue() - expected) > 1e-6
            || (point.size() == 2 && !problem->areVariableBoundsFulfilled(point, 1e-6)))
        {
            std::cout << "  FAILED: Ipopt did not recover for the next valid solve.\n";
            passed = false;
        }

        if(fixed)
            nlp.unfixVariables();
    }

    return passed;
}

int IpoptTest(int argc, char* argv[])
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
        std::cout << "Starting test to solve 1D constrained problem using Ipopt:" << std::endl;
        passed = IpoptTest1();
        std::cout << "Finished test to solve 1D constrained problem using Ipopt." << std::endl;
        break;
    case 2:
        std::cout << "Starting test to solve 2D unconstrained problem using Ipopt:" << std::endl;
        passed = IpoptTest2();
        std::cout << "Finished test to solve 2D unconstrained problem using Ipopt." << std::endl;
        break;
    case 3:
        passed = IpoptTest3();
        break;
    case 4:
        passed = IpoptTest4();
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
