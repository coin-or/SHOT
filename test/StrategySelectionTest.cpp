/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "../src/DualSolver.h"
#include "../src/Environment.h"
#include "../src/Results.h"
#include "../src/Solver.h"
#include "../src/Model/Constraints.h"
#include "../src/Model/NonlinearExpressions.h"
#include "../src/Model/ObjectiveFunction.h"
#include "../src/Model/Problem.h"
#include "../src/Model/Terms.h"
#include "../src/Model/Variables.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

using namespace SHOT;

namespace
{
enum class FunctionType
{
    Linear,
    ConvexQuadratic,
    NonconvexQuadratic,
    ConvexNonlinear,
    NonconvexNonlinear
};

struct Strategies
{
    E_SolutionStrategy continuous;
    E_SolutionStrategy discrete;
};

struct ProblemCase
{
    const char* name;
    FunctionType objective;
    FunctionType constraints;
    std::array<Strategies, 4> expected; // Cbc, HiGHS, CPLEX, Gurobi 10+
};

bool isNonconvex(FunctionType type)
{
    return type == FunctionType::NonconvexQuadratic || type == FunctionType::NonconvexNonlinear;
}

bool isQuadratic(FunctionType type)
{
    return type == FunctionType::ConvexQuadratic || type == FunctionType::NonconvexQuadratic;
}

bool isNonlinear(FunctionType type)
{
    return type == FunctionType::ConvexNonlinear || type == FunctionType::NonconvexNonlinear;
}

ProblemPtr makeProblem(EnvironmentPtr env, const ProblemCase& test, bool discrete, bool maximize)
{
    auto problem = std::make_shared<Problem>(env);
    problem->name = test.name;
    auto x = std::make_shared<Variable>("x", discrete ? E_VariableType::Integer : E_VariableType::Real, 0.0, 2.0);
    auto y = std::make_shared<Variable>("y", E_VariableType::Real, 0.0, 2.0);
    problem->add({ x, y });

    auto direction = maximize ? E_ObjectiveFunctionDirection::Maximize : E_ObjectiveFunctionDirection::Minimize;
    double sign = (maximize ? -1.0 : 1.0) * (isNonconvex(test.objective) ? -1.0 : 1.0);
    if(isQuadratic(test.objective))
    {
        auto objective = std::make_shared<QuadraticObjectiveFunction>(direction);
        objective->add(std::make_shared<QuadraticTerm>(sign, x, x));
        objective->add(std::make_shared<QuadraticTerm>(sign, y, y));
        problem->add(objective);
    }
    else if(isNonlinear(test.objective))
    {
        auto objective = std::make_shared<NonlinearObjectiveFunction>(direction);
        for(auto variable : { x, y })
            objective->add(std::make_shared<ExpressionProduct>(std::make_shared<ExpressionConstant>(sign),
                std::make_shared<ExpressionExp>(std::make_shared<ExpressionVariable>(variable))));
        problem->add(objective);
    }
    else
    {
        auto objective = std::make_shared<LinearObjectiveFunction>(direction);
        objective->add(std::make_shared<LinearTerm>(sign, x));
        objective->add(std::make_shared<LinearTerm>(sign, y));
        problem->add(objective);
    }

    for(int i : { 1, 2 })
    {
        auto linear = std::make_shared<LinearConstraint>("linear" + std::to_string(i), SHOT_DBL_MIN, 2.0 + i);
        linear->add(std::make_shared<LinearTerm>(static_cast<double>(i), x));
        linear->add(std::make_shared<LinearTerm>(1.0, y));
        problem->add(linear);

        sign = isNonconvex(test.constraints) ? -1.0 : 1.0;
        if(isQuadratic(test.constraints))
        {
            auto constraint = std::make_shared<QuadraticConstraint>("quadratic" + std::to_string(i), SHOT_DBL_MIN,
                isNonconvex(test.constraints) ? -static_cast<double>(i) : 2.0 + i);
            constraint->add(std::make_shared<QuadraticTerm>(sign, x, x));
            constraint->add(std::make_shared<QuadraticTerm>(sign * i, y, y));
            problem->add(constraint);
        }
        else if(isNonlinear(test.constraints))
        {
            auto constraint = std::make_shared<NonlinearConstraint>("nonlinear" + std::to_string(i), SHOT_DBL_MIN,
                isNonconvex(test.constraints) ? -2.0 * (i + 1) : 4.0 + 6.0 * i);
            constraint->add(std::make_shared<ExpressionProduct>(std::make_shared<ExpressionConstant>(sign),
                std::make_shared<ExpressionExp>(std::make_shared<ExpressionVariable>(x))));
            constraint->add(std::make_shared<ExpressionProduct>(std::make_shared<ExpressionConstant>(sign * i),
                std::make_shared<ExpressionExp>(std::make_shared<ExpressionVariable>(y))));
            problem->add(constraint);
        }
    }
    problem->finalize();
    return problem;
}

bool testDefaultStrategies(ES_MIPSolver backend, size_t column)
{
    using F = FunctionType;
    using S = E_SolutionStrategy;
    constexpr Strategies nlpMulti { S::NLP, S::MultiTree };
    constexpr Strategies nlpSingle { S::NLP, S::SingleTree };
    constexpr Strategies directQP { S::MIQP, S::MIQP };
    constexpr Strategies directQCQP { S::MIQCQP, S::MIQCQP };
    constexpr Strategies directLinear { S::NLP, S::MILP };

    // Freeze the expected routing independently of the settings and selector implementation.
    const ProblemCase cases[] = {
        { "linear", F::Linear, F::Linear, { directLinear, directLinear, directLinear, directLinear } },
        { "convex_qp", F::ConvexQuadratic, F::Linear, { nlpMulti, nlpMulti, directQP, directQP } },
        { "nonconvex_qp", F::NonconvexQuadratic, F::Linear, { nlpMulti, nlpMulti, directQP, directQP } },
        { "convex_qcqp_linear_objective", F::Linear, F::ConvexQuadratic,
            { nlpMulti, nlpMulti, directQCQP, directQCQP } },
        { "convex_qcqp_quadratic_objective", F::ConvexQuadratic, F::ConvexQuadratic,
            { nlpMulti, nlpMulti, directQCQP, directQCQP } },
        { "nonconvex_objective_convex_quadratic_constraints", F::NonconvexQuadratic, F::ConvexQuadratic,
            { nlpMulti, nlpMulti, nlpMulti, directQCQP } },
        { "linear_objective_nonconvex_quadratic_constraints", F::Linear, F::NonconvexQuadratic,
            { nlpMulti, nlpMulti, nlpMulti, directQCQP } },
        { "convex_objective_nonconvex_quadratic_constraints", F::ConvexQuadratic, F::NonconvexQuadratic,
            { nlpMulti, nlpMulti, nlpMulti, directQCQP } },
        { "nonconvex_objective_nonconvex_quadratic_constraints", F::NonconvexQuadratic, F::NonconvexQuadratic,
            { nlpMulti, nlpMulti, nlpMulti, directQCQP } },
        { "convex_nonlinear_objective", F::ConvexNonlinear, F::Linear, { nlpMulti, nlpMulti, nlpSingle, nlpSingle } },
        { "nonconvex_nonlinear_objective", F::NonconvexNonlinear, F::Linear,
            { nlpMulti, nlpMulti, nlpMulti, nlpMulti } },
        { "convex_nonlinear_constraints", F::Linear, F::ConvexNonlinear, { nlpMulti, nlpMulti, nlpSingle, nlpSingle } },
        { "nonconvex_nonlinear_constraints", F::Linear, F::NonconvexNonlinear,
            { nlpMulti, nlpMulti, nlpMulti, nlpMulti } },
        { "convex_quadratic_objective_convex_nonlinear_constraints", F::ConvexQuadratic, F::ConvexNonlinear,
            { nlpMulti, nlpMulti, nlpSingle, nlpSingle } },
        { "convex_quadratic_objective_nonconvex_nonlinear_constraints", F::ConvexQuadratic, F::NonconvexNonlinear,
            { nlpMulti, nlpMulti, nlpMulti, nlpMulti } },
        { "nonconvex_quadratic_objective_convex_nonlinear_constraints", F::NonconvexQuadratic, F::ConvexNonlinear,
            { nlpMulti, nlpMulti, nlpMulti, nlpMulti } },
        { "nonconvex_quadratic_objective_nonconvex_nonlinear_constraints", F::NonconvexQuadratic, F::NonconvexNonlinear,
            { nlpMulti, nlpMulti, nlpMulti, nlpMulti } },
        { "convex_nonlinear_objective_convex_quadratic_constraints", F::ConvexNonlinear, F::ConvexQuadratic,
            { nlpMulti, nlpMulti, nlpSingle, nlpSingle } },
        { "convex_nonlinear_objective_nonconvex_quadratic_constraints", F::ConvexNonlinear, F::NonconvexQuadratic,
            { nlpMulti, nlpMulti, nlpMulti, nlpMulti } },
        { "nonconvex_nonlinear_objective_convex_quadratic_constraints", F::NonconvexNonlinear, F::ConvexQuadratic,
            { nlpMulti, nlpMulti, nlpMulti, nlpMulti } },
        { "nonconvex_nonlinear_objective_nonconvex_quadratic_constraints", F::NonconvexNonlinear, F::NonconvexQuadratic,
            { nlpMulti, nlpMulti, nlpMulti, nlpMulti } },
        { "convex_nonlinear_objective_convex_nonlinear_constraints", F::ConvexNonlinear, F::ConvexNonlinear,
            { nlpMulti, nlpMulti, nlpSingle, nlpSingle } },
        { "convex_nonlinear_objective_nonconvex_nonlinear_constraints", F::ConvexNonlinear, F::NonconvexNonlinear,
            { nlpMulti, nlpMulti, nlpMulti, nlpMulti } },
        { "nonconvex_nonlinear_objective_convex_nonlinear_constraints", F::NonconvexNonlinear, F::ConvexNonlinear,
            { nlpMulti, nlpMulti, nlpMulti, nlpMulti } },
        { "nonconvex_nonlinear_objective_nonconvex_nonlinear_constraints", F::NonconvexNonlinear, F::NonconvexNonlinear,
            { nlpMulti, nlpMulti, nlpMulti, nlpMulti } },
    };
    const char* names[] = { "Cbc", "HiGHS", "CPLEX", "Gurobi" };
    int checked = 0;
    for(const auto& test : cases)
    {
        for(bool discrete : { false, true })
        {
            for(bool maximize : { false, true })
            {
                auto solver = std::make_unique<Solver>();
                auto env = solver->getEnvironment();
                // Only select the backend and silence logging; all algorithm settings retain their defaults.
                solver->updateSetting("Dual.MIP.Solver", static_cast<int>(backend));
                solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
                solver->updateSetting("Output.File.LogLevel", static_cast<int>(E_LogLevel::Off));
                auto problem = makeProblem(env, test, discrete, maximize);
                bool nonconvex = isNonconvex(test.objective) || isNonconvex(test.constraints);
                std::string type;
                if(isNonlinear(test.objective) || isNonlinear(test.constraints))
                    type = discrete ? "MINLP" : "NLP";
                else if(isQuadratic(test.constraints))
                    type = discrete ? "MIQCQP" : "QCQP";
                else if(isQuadratic(test.objective))
                    type = discrete ? "MIQP" : "QP";
                else
                    type = discrete ? "MILP" : "LP";
                type += nonconvex ? ", nonconvex" : ", convex";
                if(problem->properties.getClassification() != type)
                {
                    std::cout << "Invalid strategy fixture " << test.name << ": expected " << type << ", got "
                              << problem->properties.getClassification() << ", maximize " << maximize << '\n';
                    return false;
                }
                if(!solver->setProblem(problem))
                {
                    std::cout << "Cannot initialize " << names[column] << " default strategy for " << test.name << '\n';
                    return false;
                }

                auto expected = test.expected[column];
                auto strategy = discrete ? expected.discrete : expected.continuous;

                if(env->results->usedSolutionStrategy != strategy)
                {
                    std::cout << names[column] << " default strategy changed for " << test.name << ", discrete "
                              << discrete << ", maximize " << maximize << ": expected " << static_cast<int>(strategy)
                              << ", got " << static_cast<int>(env->results->usedSolutionStrategy) << '\n';
                    return false;
                }

                if(std::getenv("SHOT_STRATEGY_SNAPSHOT"))
                {
                    const auto& reformulated = env->reformulatedProblem;
                    const auto& p = reformulated->properties;
                    size_t quadraticTerms = 0;

                    for(const auto& constraint : reformulated->quadraticConstraints)
                        quadraticTerms += constraint->quadraticTerms.size();

                    if(reformulated->objectiveFunction->properties.hasQuadraticTerms)
                        quadraticTerms
                            += std::dynamic_pointer_cast<QuadraticObjectiveFunction>(reformulated->objectiveFunction)
                                   ->quadraticTerms.size();

                    std::cout << "Strategy snapshot " << column << ' ' << test.name << ' ' << discrete << ' '
                              << maximize << ' ' << static_cast<int>(env->results->usedSolutionStrategy) << ' '
                              << p.numberOfVariables << ' ' << p.numberOfBinaryVariables << ' '
                              << p.numberOfIntegerVariables << ' ' << p.numberOfLinearConstraints << ' '
                              << p.numberOfQuadraticConstraints << ' ' << p.numberOfNonlinearConstraints << ' '
                              << static_cast<int>(reformulated->objectiveFunction->properties.classification) << ' '
                              << quadraticTerms << '\n';
                }

                checked++;
            }
        }
    }
    std::cout << "Checked " << checked << " default strategy combinations for " << names[column] << '\n';

    return true;
}

bool testDirectReformulatedMILP(ES_MIPSolver backend)
{
    for(bool maximize : { false, true })
    {
        for(auto tree : { ES_TreeStrategy::SingleTree, ES_TreeStrategy::MultiTree })
        {
            auto solver = std::make_unique<Solver>();
            auto env = solver->getEnvironment();
            solver->updateSetting("Dual.MIP.Solver", static_cast<int>(backend));
            solver->updateSetting("Dual.TreeStrategy", static_cast<int>(tree));
            solver->updateSetting("Dual.MIP.NumberOfThreads", 1);
            solver->updateSetting("Dual.MIP.SolutionLimit.Initial", 1);
            solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
            solver->updateSetting("Output.File.LogLevel", static_cast<int>(E_LogLevel::Off));

            auto problem = std::make_shared<Problem>(env);
            problem->name = "shared_binary_products";
            auto x = std::make_shared<Variable>("x", E_VariableType::Binary, 0.0, 1.0);
            auto y = std::make_shared<Variable>("y", E_VariableType::Binary, 0.0, 1.0);
            auto z = std::make_shared<Variable>("z", E_VariableType::Binary, 0.0, 1.0);
            auto w = std::make_shared<Variable>("w", E_VariableType::Binary, 0.0, 1.0);
            problem->add({ x, y, z, w });

            auto objective = std::make_shared<NonlinearObjectiveFunction>(
                maximize ? E_ObjectiveFunctionDirection::Maximize : E_ObjectiveFunctionDirection::Minimize);
            objective->add(std::make_shared<LinearTerm>(-3.0, x));
            objective->add(std::make_shared<LinearTerm>(-2.0, y));
            objective->add(std::make_shared<LinearTerm>(-1.0, z));
            objective->add(std::make_shared<LinearTerm>(-1.0, w));
            objective->add(std::make_shared<MonomialTerm>(8.0, Variables { x, y, z }));
            objective->add(std::make_shared<MonomialTerm>(7.0, Variables { y, z, w }));
            problem->add(objective);

            for(int i : { 1, 2 })
            {
                auto constraint = std::make_shared<NonlinearConstraint>(
                    "products" + std::to_string(i), SHOT_DBL_MIN, static_cast<double>(i));
                constraint->add(std::make_shared<MonomialTerm>(static_cast<double>(i), Variables { x, y, z }));
                constraint->add(std::make_shared<MonomialTerm>(1.0, Variables { y, z, w }));
                problem->add(constraint);
            }

            problem->finalize();

            if(!solver->setProblem(problem) || !env->reformulatedProblem->properties.isMILPProblem
                || env->results->usedSolutionStrategy != E_SolutionStrategy::MILP || env->dualSolver->isSingleTree)
            {
                std::cout << "Binary products did not select a direct MILP solve\n";
                return false;
            }

            if(!solver->solveProblem() || !solver->hasPrimalSolution())
            {
                std::cout << "Direct reformulated MILP solve failed\n";
                return false;
            }

            double optimum = maximize ? 3.0 : -6.0;
            const auto& point = solver->getPrimalSolution().point;

            if(std::abs(solver->getPrimalBound() - optimum) > 1e-6
                || std::abs(solver->getGlobalDualBound() - optimum) > 1e-6 || env->results->getNumberOfIterations() != 1
                || !problem->areNonlinearConstraintsFulfilled(point, 1e-6)
                || (solver->getTerminationReason() != E_TerminationReason::AbsoluteGap
                    && solver->getTerminationReason() != E_TerminationReason::RelativeGap))
            {
                std::cout << "Direct reformulated MILP did not prove optimum " << optimum << " in one solve: primal "
                          << solver->getPrimalBound() << ", dual " << solver->getGlobalDualBound() << ", iterations "
                          << env->results->getNumberOfIterations() << ", termination "
                          << static_cast<int>(solver->getTerminationReason()) << '\n';
                return false;
            }
        }
    }

    return true;
}

bool testBinaryQuadraticConstraints(ES_MIPSolver backend)
{
    for(bool linearize : { false, true })
    {
        for(bool maximize : { false, true })
        {
            Solver solver;
            auto env = solver.getEnvironment();
            solver.updateSetting("Dual.MIP.Solver", static_cast<int>(backend));
            solver.updateSetting("Dual.MIP.NumberOfThreads", 1);
            solver.updateSetting("Primal.FixedInteger.Use", false);
            solver.updateSetting("Primal.PolishSolution.NumberOfPoints", 0);
            solver.updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
            solver.updateSetting("Output.File.LogLevel", static_cast<int>(E_LogLevel::Off));

            if(linearize)
                solver.updateSetting("Model.Reformulation.Quadratics.Strategy",
                    static_cast<int>(ES_QuadraticProblemStrategy::Nonlinear));

            auto problem = std::make_shared<Problem>(env);
            problem->name = "binary_quadratic_constraints";
            auto x = std::make_shared<Variable>("x", E_VariableType::Binary, 0.0, 1.0);
            auto y = std::make_shared<Variable>("y", E_VariableType::Binary, 0.0, 1.0);
            auto z = std::make_shared<Variable>("z", E_VariableType::Binary, 0.0, 1.0);
            problem->add({ x, y, z });
            auto objective = std::make_shared<LinearObjectiveFunction>(
                maximize ? E_ObjectiveFunctionDirection::Maximize : E_ObjectiveFunctionDirection::Minimize);
            objective->add(std::make_shared<LinearTerm>(1.0, x));
            objective->add(std::make_shared<LinearTerm>(2.0, y));
            objective->add(std::make_shared<LinearTerm>(3.0, z));
            problem->add(objective);

            for(int i : { 1, 2 })
            {
                auto constraint = std::make_shared<QuadraticConstraint>(
                    "products" + std::to_string(i), SHOT_DBL_MIN, static_cast<double>(i));
                constraint->add(std::make_shared<QuadraticTerm>(static_cast<double>(i), x, y));
                constraint->add(std::make_shared<QuadraticTerm>(1.0, y, z));
                problem->add(constraint);
            }

            problem->finalize();
            bool retainQuadratics = backend == ES_MIPSolver::Gurobi && !linearize;
            auto expected = retainQuadratics ? E_SolutionStrategy::MIQCQP : E_SolutionStrategy::MILP;

            if(!solver.setProblem(problem) || env->results->usedSolutionStrategy != expected
                || (retainQuadratics
                    && (env->reformulatedProblem->properties.numberOfAuxiliaryVariables != 0
                        || env->reformulatedProblem->quadraticConstraints.size() != 2
                        || env->reformulatedProblem->quadraticConstraints[0]->quadraticTerms.size() != 2
                        || env->reformulatedProblem->quadraticConstraints[1]->quadraticTerms.size() != 2)))
            {
                std::cout << "Binary quadratic constraints did not follow the backend's quadratic strategy\n";
                return false;
            }

            double optimum = maximize ? 5.0 : 0.0;

            if(!solver.solveProblem() || !solver.hasPrimalSolution()
                || std::abs(solver.getPrimalBound() - optimum) > 1e-6
                || std::abs(solver.getGlobalDualBound() - optimum) > 1e-6
                || solver.getModelReturnStatus() != E_ModelReturnStatus::OptimalGlobal
                || !problem->areQuadraticConstraintsFulfilled(solver.getPrimalSolution().point, 1e-6))
            {
                std::cout << "Binary quadratic constraints did not produce the expected global optimum\n";
                return false;
            }
        }
    }

    return true;
}

bool testConvexObjectiveWithNonconvexConstraints(ES_MIPSolver backend)
{
    if(backend != ES_MIPSolver::Cplex && backend != ES_MIPSolver::Gurobi)
        return true;

    const ProblemCase test { "convex_objective_nonconvex_constraints", FunctionType::ConvexQuadratic,
        FunctionType::NonconvexQuadratic, { } };

    for(bool discrete : { false, true })
    {
        for(bool maximize : { false, true })
        {
            for(int strategy : { -1, 2 })
            {
                Solver solver;
                auto env = solver.getEnvironment();
                solver.updateSetting("Dual.MIP.Solver", static_cast<int>(backend));
                solver.updateSetting("Dual.MIP.NumberOfThreads", 1);
                solver.updateSetting("Primal.FixedInteger.Use", false);
                solver.updateSetting("Primal.PolishSolution.NumberOfPoints", 0);
                solver.updateSetting("Termination.TimeLimit", 10.0);
                solver.updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
                solver.updateSetting("Output.File.LogLevel", static_cast<int>(E_LogLevel::Off));

                if(strategy >= 0)
                    solver.updateSetting("Model.Reformulation.Quadratics.Strategy", strategy);

                auto problem = makeProblem(env, test, discrete, maximize);
                auto x = problem->getVariable(0);
                auto y = problem->getVariable(1);
                auto objective = std::dynamic_pointer_cast<QuadraticObjectiveFunction>(problem->objectiveFunction);
                objective->add(std::make_shared<QuadraticTerm>(maximize ? -2.0 : 2.0, x, y));
                auto constraint = std::make_shared<LinearConstraint>("objective_lower_bound", 2.0, SHOT_DBL_MAX);
                constraint->add(std::make_shared<LinearTerm>(1.0, x));
                constraint->add(std::make_shared<LinearTerm>(1.0, y));
                problem->add(constraint);
                problem->finalize();

                if(!solver.setProblem(problem)
                    || env->reformulatedProblem->objectiveFunction->properties.classification
                        != E_ObjectiveFunctionClassification::Quadratic
                    || env->reformulatedProblem->properties.numberOfAuxiliaryVariables != 0
                    || std::dynamic_pointer_cast<QuadraticObjectiveFunction>(
                           env->reformulatedProblem->objectiveFunction)
                            ->quadraticTerms.size()
                        != 3)
                {
                    std::cout << "Nonconvex constraints caused a supported quadratic objective to be partitioned\n";
                    return false;
                }

                double optimum = maximize ? -4.0 : 4.0;

                if(!solver.solveProblem() || !solver.hasPrimalSolution()
                    || std::abs(solver.getPrimalBound() - optimum) > 1e-5
                    || std::abs(solver.getGlobalDualBound() - optimum) > 1e-5
                    || !problem->areNumericConstraintsFulfilled(solver.getPrimalSolution().point, 1e-5))
                {
                    std::cout << "Retained quadratic objective did not produce the expected optimum\n";
                    return false;
                }
            }
        }
    }

    return true;
}

bool testRemovedCutCallback(ES_MIPSolver backend)
{
    const ProblemCase test { "removed_cut_callback", FunctionType::Linear, FunctionType::Linear, { } };

    for(auto tree : { ES_TreeStrategy::MultiTree, ES_TreeStrategy::SingleTree })
    {
        Solver solver;
        auto env = solver.getEnvironment();
        solver.updateSetting("Dual.MIP.Solver", static_cast<int>(backend));
        solver.updateSetting("Dual.TreeStrategy", static_cast<int>(tree));
        solver.updateSetting("Dual.MIP.NumberOfThreads", 1);
        solver.updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
        solver.updateSetting("Output.File.LogLevel", static_cast<int>(E_LogLevel::Off));
        auto handle = solver.registerCallback<HyperplaneSelectionContext>([](HyperplaneSelectionContext&) { });
        auto problem = makeProblem(env, test, true, false);

        if(!solver.setProblem(problem) || env->results->usedSolutionStrategy == E_SolutionStrategy::MILP
            || !solver.removeCallback(handle) || !solver.solveProblem()
            || env->results->usedSolutionStrategy != E_SolutionStrategy::MILP || env->dualSolver->isSingleTree
            || !solver.hasPrimalSolution() || std::abs(solver.getPrimalBound()) > 1e-6
            || std::abs(solver.getGlobalDualBound()) > 1e-6 || env->results->getNumberOfIterations() != 1)
        {
            std::cout << "Removing a cut callback did not restore a direct MILP solve\n";
            return false;
        }
    }

    return true;
}

bool testStrategies(ES_MIPSolver backend, size_t column)
{
    return testDefaultStrategies(backend, column) && testDirectReformulatedMILP(backend)
        && testBinaryQuadraticConstraints(backend) && testConvexObjectiveWithNonconvexConstraints(backend)
        && testRemovedCutCallback(backend);
}
} // namespace

int StrategySelectionTest(int argc, char* argv[])
{
    int choice = 0;
    if(argc < 2 || std::sscanf(argv[1], "%d", &choice) != 1)
        return -1;
    switch(choice)
    {
#ifdef HAS_CBC
    case 1:
        return testStrategies(ES_MIPSolver::Cbc, 0) ? 0 : -1;
#endif
#ifdef HAS_HIGHS
    case 2:
        return testStrategies(ES_MIPSolver::Highs, 1) ? 0 : -1;
#endif
#ifdef HAS_CPLEX
    case 3:
        return testStrategies(ES_MIPSolver::Cplex, 2) ? 0 : -1;
#endif
#ifdef HAS_GUROBI
    case 4:
        return testStrategies(ES_MIPSolver::Gurobi, 3) ? 0 : -1;
#endif
    default:
        std::cout << "Strategy selection test " << choice << " does not exist\n";
        return -1;
    }
}
