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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <tuple>
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

bool testExplicitQuadraticStrategies(ES_MIPSolver backend)
{
    using F = FunctionType;
    int checked = 0;

    for(auto objectiveType : { F::Linear, F::ConvexQuadratic, F::NonconvexQuadratic })
    {
        for(auto constraintType : { F::Linear, F::ConvexQuadratic, F::NonconvexQuadratic })
        {
            if(objectiveType == F::Linear && constraintType == F::Linear)
                continue;

            const ProblemCase test { "explicit_quadratic_policy", objectiveType, constraintType, { } };

            for(bool discrete : { false, true })
            {
                for(bool maximize : { false, true })
                {
                    for(int strategy : { 0, 1, 2, 3 })
                    {
                        Solver solver;
                        auto env = solver.getEnvironment();
                        solver.updateSetting("Dual.MIP.Solver", static_cast<int>(backend));
                        solver.updateSetting("Model.Reformulation.Quadratics.Strategy", strategy);
                        solver.updateSetting("Model.BoundTightening.FeasibilityBased.Use", false);
                        solver.updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
                        solver.updateSetting("Output.File.LogLevel", static_cast<int>(E_LogLevel::Off));
                        auto problem = makeProblem(env, test, discrete, maximize);

                        if(!solver.setProblem(problem))
                            return false;

                        bool commercial = backend == ES_MIPSolver::Cplex || backend == ES_MIPSolver::Gurobi;
                        bool objectiveSupported = objectiveType == F::Linear || (commercial && strategy >= 1);
                        bool constraintsSupported = constraintType == F::Linear
                            || (commercial && strategy >= 2
                                && (constraintType == F::ConvexQuadratic
                                    || (backend == ES_MIPSolver::Gurobi && strategy == 3)));
                        bool excludedCombination = backend == ES_MIPSolver::Cplex
                            && objectiveType == F::NonconvexQuadratic && constraintType != F::Linear;
                        bool direct = objectiveSupported && constraintsSupported && !excludedCombination;
                        bool nonconvex = isNonconvex(objectiveType) || isNonconvex(constraintType);
                        auto expected = !discrete      ? E_SolutionStrategy::NLP
                            : nonconvex || !commercial ? E_SolutionStrategy::MultiTree
                                                       : E_SolutionStrategy::SingleTree;

                        if(direct)
                            expected
                                = constraintType == F::Linear ? E_SolutionStrategy::MIQP : E_SolutionStrategy::MIQCQP;

                        if(env->results->usedSolutionStrategy != expected
                            || (direct && env->reformulatedProblem->properties.numberOfAuxiliaryVariables != 0))
                        {
                            std::cout << "Explicit quadratic policy mismatch: backend " << static_cast<int>(backend)
                                      << ", objective " << static_cast<int>(objectiveType) << ", constraints "
                                      << static_cast<int>(constraintType) << ", discrete " << discrete << ", maximize "
                                      << maximize << ", strategy " << strategy << ", expected "
                                      << static_cast<int>(expected) << ", got "
                                      << static_cast<int>(env->results->usedSolutionStrategy) << '\n';

                            return false;
                        }

                        ++checked;
                    }
                }
            }
        }
    }

    std::cout << "Checked " << checked << " explicit pure-quadratic policies for backend " << static_cast<int>(backend)
              << '\n';

    return true;
}

bool testForcedIntegerBilinearReformulation(ES_MIPSolver backend)
{
    int checked = 0;

    for(bool inObjective : { false, true })
    {
        for(bool maximize : { false, true })
        {
            for(int strategy : { -1, 0, 1, 2, 3 })
            {
                Solver solver;
                auto env = solver.getEnvironment();
                solver.updateSetting("Dual.MIP.Solver", static_cast<int>(backend));
                solver.updateSetting("Model.Reformulation.Bilinear.IntegerFormulation",
                    static_cast<int>(ES_ReformulateBilinearInteger::Yes));
                solver.updateSetting("Model.BoundTightening.FeasibilityBased.Use", false);
                solver.updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
                solver.updateSetting("Output.File.LogLevel", static_cast<int>(E_LogLevel::Off));

                if(strategy >= 0)
                    solver.updateSetting("Model.Reformulation.Quadratics.Strategy", strategy);

                auto problem = std::make_shared<Problem>(env);
                auto x = std::make_shared<Variable>("x", E_VariableType::Integer, 0.0, 2.0);
                auto y = std::make_shared<Variable>("y", E_VariableType::Integer, 0.0, 2.0);
                auto z = std::make_shared<Variable>("z", E_VariableType::Integer, 0.0, 2.0);
                problem->add({ x, y, z });
                auto direction
                    = maximize ? E_ObjectiveFunctionDirection::Maximize : E_ObjectiveFunctionDirection::Minimize;

                if(inObjective)
                {
                    double sign = maximize ? -1.0 : 1.0;
                    auto objective = std::make_shared<QuadraticObjectiveFunction>(direction);
                    objective->add(std::make_shared<QuadraticTerm>(4.0 * sign, x, y));
                    objective->add(std::make_shared<QuadraticTerm>(-3.0 * sign, y, z));
                    problem->add(objective);
                }
                else
                {
                    auto objective = std::make_shared<LinearObjectiveFunction>(direction);
                    objective->add(std::make_shared<LinearTerm>(1.0, x));
                    problem->add(objective);

                    for(int i : { 1, 2 })
                    {
                        auto constraint = std::make_shared<QuadraticConstraint>(
                            "products" + std::to_string(i), i == 1 ? SHOT_DBL_MIN : -2.0, 3.0);
                        constraint->add(std::make_shared<QuadraticTerm>(2.0 * i, x, y));
                        constraint->add(std::make_shared<QuadraticTerm>(-1.0, y, z));
                        problem->add(constraint);
                    }
                }

                problem->finalize();

                if(!solver.setProblem(problem) || !env->reformulatedProblem->properties.isMILPProblem
                    || env->results->usedSolutionStrategy != E_SolutionStrategy::MILP)
                {
                    std::cout << "Forced integer-bilinear reformulation was overridden: backend "
                              << static_cast<int>(backend) << ", objective " << inObjective << ", maximize " << maximize
                              << ", strategy " << strategy << '\n';

                    return false;
                }

                ++checked;
            }
        }
    }

    std::cout << "Checked " << checked << " forced integer-bilinear policies for backend " << static_cast<int>(backend)
              << '\n';

    return true;
}

ProblemPtr makeMixedPolynomialProblem(
    EnvironmentPtr env, bool inObjective, bool convex, bool maximize, bool expressions)
{
    auto problem = std::make_shared<Problem>(env);
    problem->name = "mixed_polynomial_quadratics";
    Variables binaries;

    for(const auto& name : { "x", "y", "z", "w" })
    {
        auto variable = std::make_shared<Variable>(name, E_VariableType::Binary, 0.0, 1.0);
        binaries.push_back(variable);
        problem->add(variable);
    }

    Variables quadraticVariables = binaries;

    if(convex)
    {
        // Continuous aliases preserve the positive-definite form, while every feasible point can still be enumerated.
        quadraticVariables.clear();

        for(int i = 0; i < 3; ++i)
        {
            auto variable = std::make_shared<Variable>("q" + std::to_string(i), E_VariableType::Real, 0.0, 1.0);
            quadraticVariables.push_back(variable);
            problem->add(variable);
            auto alias = std::make_shared<LinearConstraint>("alias" + std::to_string(i), 0.0, 0.0);
            alias->add(std::make_shared<LinearTerm>(1.0, variable));
            alias->add(std::make_shared<LinearTerm>(-1.0, binaries[i]));
            problem->add(alias);
        }
    }

    auto addPolynomial = [&](auto& function, double quadraticScale, double cubicCoefficient, double quarticCoefficient)
    {
        auto sum = std::make_shared<ExpressionSum>();
        auto addProduct = [&](double coefficient, const Variables& variables)
        {
            auto product = std::make_shared<ExpressionProduct>();
            product->children.push_back(std::make_shared<ExpressionConstant>(coefficient));

            for(const auto& variable : variables)
                product->children.push_back(std::make_shared<ExpressionVariable>(variable));

            sum->children.push_back(product);
        };

        std::vector<std::tuple<double, int, int>> terms = convex
            ? std::vector<std::tuple<double, int, int>> { { 2.0, 0, 0 }, { 3.0, 1, 1 }, { 2.0, 2, 2 }, { 1.0, 0, 1 },
                  { 1.0, 0, 1 }, { 1.0, 1, 2 }, { 1.0, 1, 2 } }
            : std::vector<std::tuple<double, int, int>> { { 2.0, 0, 1 }, { 2.0, 0, 1 }, { -1.5, 1, 2 },
                  { -1.5, 1, 2 } };

        for(const auto& [coefficient, first, second] : terms)
        {
            if(expressions)
                addProduct(quadraticScale * coefficient, { quadraticVariables[first], quadraticVariables[second] });
            else
                function->add(std::make_shared<QuadraticTerm>(
                    quadraticScale * coefficient, quadraticVariables[first], quadraticVariables[second]));
        }

        for(int i = 0; i < 3; ++i)
        {
            double coefficient = i == 2 ? quarticCoefficient : cubicCoefficient / 2.0;
            Variables variables = i == 2 ? binaries : Variables { binaries[0], binaries[1], binaries[2] };

            if(expressions)
                addProduct(coefficient, variables);
            else
                function->add(std::make_shared<MonomialTerm>(coefficient, variables));
        }

        if(expressions)
            function->add(sum);
    };

    auto direction = maximize ? E_ObjectiveFunctionDirection::Maximize : E_ObjectiveFunctionDirection::Minimize;
    double sign = maximize ? -1.0 : 1.0;
    const std::array<double, 4> linearCoefficients { -3.0, -2.0, -1.0, -1.0 };

    if(inObjective)
    {
        auto objective = std::make_shared<NonlinearObjectiveFunction>(direction);
        addPolynomial(objective, sign, 8.0 * sign, 7.0 * sign);

        for(int i = 0; i < 4; ++i)
            objective->add(std::make_shared<LinearTerm>(sign * linearCoefficients[i], binaries[i]));

        problem->add(objective);

        for(int i : { 0, 2 })
        {
            auto constraint = std::make_shared<LinearConstraint>("nonzero" + std::to_string(i), 1.0, SHOT_DBL_MAX);
            constraint->add(std::make_shared<LinearTerm>(1.0, binaries[i]));
            constraint->add(std::make_shared<LinearTerm>(1.0, binaries[i + 1]));
            problem->add(constraint);
        }
    }
    else
    {
        auto objective = std::make_shared<LinearObjectiveFunction>(direction);

        for(int i = 0; i < 4; ++i)
            objective->add(std::make_shared<LinearTerm>(sign * linearCoefficients[i], binaries[i]));

        problem->add(objective);

        for(int i : { 1, 2 })
        {
            auto constraint
                = std::make_shared<NonlinearConstraint>("mixed" + std::to_string(i), SHOT_DBL_MIN, 2.0 * i + 2.0);
            addPolynomial(constraint, static_cast<double>(i), i == 1 ? 2.0 : 3.0, i == 1 ? 3.0 : 2.0);
            problem->add(constraint);
        }
    }

    problem->finalize();

    return problem;
}

bool testMixedPolynomialQuadratics(ES_MIPSolver backend)
{
    int checked = 0;
    int solved = 0;
    std::vector<std::tuple<bool, bool, bool, bool, int, int>> cases;

    for(bool inObjective : { false, true })
        for(bool convex : { false, true })
            for(bool maximize : { false, true })
                for(bool expressions : { false, true })
                    for(int quadraticStrategy : { -1, 0, 1, 2, 3 })
                        for(int partitionStrategy : { -1, 0, 1, 2 })
                            cases.emplace_back(
                                inObjective, convex, maximize, expressions, quadraticStrategy, partitionStrategy);

    for(const auto& test : cases)
    {
        bool inObjective = std::get<0>(test);
        bool convex = std::get<1>(test);
        bool maximize = std::get<2>(test);
        bool expressions = std::get<3>(test);
        int quadraticStrategy = std::get<4>(test);
        int partitionStrategy = std::get<5>(test);
        Solver solver;
        auto env = solver.getEnvironment();
        solver.updateSetting("Dual.MIP.Solver", static_cast<int>(backend));
        solver.updateSetting("Dual.MIP.NumberOfThreads", 1);
        solver.updateSetting("Model.BoundTightening.FeasibilityBased.Use", false);
        solver.updateSetting("Primal.FixedInteger.Use", false);
        solver.updateSetting("Primal.PolishSolution.NumberOfPoints", 0);
        solver.updateSetting("Termination.TimeLimit", 5.0);
        solver.updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
        solver.updateSetting("Output.File.LogLevel", static_cast<int>(E_LogLevel::Off));

        if(quadraticStrategy >= 0)
            solver.updateSetting("Model.Reformulation.Quadratics.Strategy", quadraticStrategy);

        if(partitionStrategy >= 0)
            solver.updateSetting(inObjective ? "Model.Reformulation.ObjectiveFunction.PartitionQuadraticTerms"
                                             : "Model.Reformulation.Constraint.PartitionQuadraticTerms",
                partitionStrategy);

        auto problem = makeMixedPolynomialProblem(env, inObjective, convex, maximize, expressions);
        auto fail = [&](const char* message)
        {
            std::cout << message << ": backend " << static_cast<int>(backend) << ", objective " << inObjective
                      << ", convex " << convex << ", maximize " << maximize << ", expressions " << expressions
                      << ", quadratic strategy " << quadraticStrategy << ", partition strategy " << partitionStrategy
                      << '\n';

            return false;
        };

        if(!solver.setProblem(problem))
            return fail("Cannot initialize mixed polynomial problem");

        auto reformulated = env->reformulatedProblem;
        bool commercial = backend == ES_MIPSolver::Gurobi || backend == ES_MIPSolver::Cplex;
        int effectiveStrategy = !commercial ? 0
            : quadraticStrategy < 0         ? (backend == ES_MIPSolver::Gurobi ? 3 : 2)
                                            : quadraticStrategy;
        bool native = commercial
            && (inObjective ? effectiveStrategy >= 1
                            : effectiveStrategy >= 2
                        && (convex || (backend == ES_MIPSolver::Gurobi && effectiveStrategy == 3)));
        bool retain = partitionStrategy == 2 || (partitionStrategy == -1 && native);
        bool allRetained = true;

        if(solver.getSetting<int>("Model.Reformulation.Quadratics.Strategy") != effectiveStrategy)
            return fail("Incorrect effective quadratic strategy");
        int firstIndex = convex ? 4 : 0;
        int secondIndex = convex ? 5 : 1;
        int thirdIndex = convex ? 6 : 2;
        std::vector<std::pair<NumericConstraintPtr, NumericConstraintPtr>> mixedConstraints;
        auto checkTerms = [&](const QuadraticTerms& terms, double scale)
        {
            double firstCoefficient = 0.0;
            double secondCoefficient = 0.0;
            std::array<double, 3> diagonalCoefficients { 0.0, 0.0, 0.0 };

            for(const auto& term : terms)
            {
                int first = std::min(term->firstVariable->getIndex(), term->secondVariable->getIndex());
                int second = std::max(term->firstVariable->getIndex(), term->secondVariable->getIndex());

                if(first == firstIndex && second == secondIndex)
                    firstCoefficient += term->coefficient;

                if(first == secondIndex && second == thirdIndex)
                    secondCoefficient += term->coefficient;

                if(first == second && first >= firstIndex && first <= thirdIndex)
                    diagonalCoefficients[first - firstIndex] += term->coefficient;
            }

            bool preserved = std::abs(firstCoefficient - scale * (convex ? 2.0 : 4.0)) < 1e-8
                && std::abs(secondCoefficient - scale * (convex ? 2.0 : -3.0)) < 1e-8;
            bool partitioned = std::abs(firstCoefficient) < 1e-8 && std::abs(secondCoefficient) < 1e-8;
            bool diagonalPreserved = !convex
                || (std::abs(diagonalCoefficients[0] - 2.0 * scale) < 1e-8
                    && std::abs(diagonalCoefficients[1] - 3.0 * scale) < 1e-8
                    && std::abs(diagonalCoefficients[2] - 2.0 * scale) < 1e-8);
            bool diagonalPartitioned = std::all_of(diagonalCoefficients.begin(), diagonalCoefficients.end(),
                [](double coefficient) { return std::abs(coefficient) < 1e-8; });
            allRetained = allRetained && preserved && diagonalPreserved;

            if(retain)
                return preserved && diagonalPreserved;

            if(!convex)
                return partitioned;

            // Continuous products can remain when squares are partitioned. IfConvex can also keep a convex sum
            // intact when its decomposition is unavailable, e.g., with reversed signs.
            return (preserved || partitioned)
                && (partitionStrategy == 0 ? diagonalPartitioned : diagonalPreserved || diagonalPartitioned);
        };

        if(inObjective)
        {
            auto objective = std::dynamic_pointer_cast<QuadraticObjectiveFunction>(reformulated->objectiveFunction);

            if(!checkTerms(objective ? objective->quadraticTerms : QuadraticTerms { }, 1.0))
                return fail("Incorrect retention of mixed objective quadratic terms");
        }
        else
        {
            for(int i : { 1, 2 })
            {
                auto constraint
                    = std::find_if(reformulated->numericConstraints.begin(), reformulated->numericConstraints.end(),
                        [&](const auto& candidate) { return candidate->name == "mixed" + std::to_string(i); });

                if(constraint == reformulated->numericConstraints.end())
                    return fail("Original mixed constraint is missing");

                auto original = std::find_if(problem->numericConstraints.begin(), problem->numericConstraints.end(),
                    [&](const auto& candidate) { return candidate->name == (*constraint)->name; });

                if(original == problem->numericConstraints.end())
                    return fail("Cannot match the original mixed constraint");

                mixedConstraints.emplace_back(*original, *constraint);
                auto quadratic = std::dynamic_pointer_cast<QuadraticConstraint>(*constraint);

                if(!checkTerms(quadratic ? quadratic->quadraticTerms : QuadraticTerms { }, static_cast<double>(i)))
                    return fail("Incorrect retention of mixed constraint quadratic terms");
            }
        }

        if(allRetained && native)
        {
            auto expected = inObjective ? E_SolutionStrategy::MIQP : E_SolutionStrategy::MIQCQP;

            if(env->results->usedSolutionStrategy != expected
                || reformulated->properties.numberOfAuxiliaryVariables != 2
                || reformulated->nonlinearConstraints.size() != 0)
                return fail("Supported mixed quadratics did not select a direct solve without quadratic auxiliaries");
        }

        double optimum = maximize ? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();

        for(int mask = 0; mask < 16; ++mask)
        {
            VectorDouble point;

            for(int i = 0; i < 4; ++i)
                point.push_back((mask >> i) & 1);

            if(convex)
            {
                for(int i = 0; i < 3; ++i)
                    point.push_back(point[i]);
            }

            bool feasible = problem->areNumericConstraintsFulfilled(point, 1e-8);
            double value = problem->objectiveFunction->calculateValue(point);

            if(feasible)
                optimum = maximize ? std::max(optimum, value) : std::min(optimum, value);

            auto augmented = point;
            reformulated->augmentAuxiliaryVariableValues(augmented);

            if(!std::all_of(augmented.begin(), augmented.end(), [](double value) { return std::isfinite(value); }))
                return fail("Mixed quadratic auxiliary values are not finite");

            double objectiveSign = problem->objectiveFunction->properties.isMinimize
                    == reformulated->objectiveFunction->properties.isMinimize
                ? 1.0
                : -1.0;

            if(reformulated->areNumericConstraintsFulfilled(augmented, 1e-8) != feasible
                || std::abs(reformulated->objectiveFunction->calculateValue(augmented) - objectiveSign * value) > 1e-8)
                return fail("Mixed quadratic reformulation changed the feasible points or objective values");

            for(const auto& [original, destination] : mixedConstraints)
            {
                double originalResidual = original->calculateFunctionValue(point) - original->valueRHS;
                double reformulatedResidual = destination->calculateFunctionValue(augmented) - destination->valueRHS;

                if(!std::isfinite(reformulatedResidual) || std::abs(originalResidual - reformulatedResidual) > 1e-8)
                    return fail("Mixed quadratic reformulation changed a constraint value");
            }
        }

        bool direct = env->results->usedSolutionStrategy == E_SolutionStrategy::MILP
            || env->results->usedSolutionStrategy == E_SolutionStrategy::MIQP
            || env->results->usedSolutionStrategy == E_SolutionStrategy::MIQCQP;

        if(partitionStrategy == -1 && (quadraticStrategy == -1 || quadraticStrategy == 0 || quadraticStrategy == 3)
            && direct)
        {
            if(!solver.solveProblem() || !solver.hasPrimalSolution()
                || std::abs(solver.getPrimalBound() - optimum) > 1e-6
                || std::abs(solver.getGlobalDualBound() - optimum) > 1e-6
                || !problem->areNumericConstraintsFulfilled(solver.getPrimalSolution().point, 1e-6))
                return fail("Direct mixed quadratic solve disagrees with exhaustive enumeration");

            ++solved;
        }

        ++checked;
    }

    std::cout << "Checked " << checked << " mixed quadratic cases and " << solved << " direct solves for backend "
              << static_cast<int>(backend) << '\n';

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
        && testRemovedCutCallback(backend) && testExplicitQuadraticStrategies(backend)
        && testMixedPolynomialQuadratics(backend) && testForcedIntegerBilinearReformulation(backend);
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
