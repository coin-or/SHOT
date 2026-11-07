/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "../src/Environment.h"
#include "../src/Results.h"
#include "../src/Solver.h"
#include "../src/Model/Constraints.h"
#include "../src/Model/NonlinearExpressions.h"
#include "../src/Model/ObjectiveFunction.h"
#include "../src/Model/Problem.h"
#include "../src/Model/Terms.h"
#include "../src/Model/Variables.h"

#ifdef HAS_GUROBI
#include "gurobi_c.h"
#endif

#include <array>
#include <cstdio>
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
    std::array<Strategies, 4> expected; // Cbc, HiGHS, CPLEX, Gurobi 9+
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

    // Freeze the expected routing independently of the settings and selector implementation.
    const ProblemCase cases[] = {
        { "linear", F::Linear, F::Linear, { nlpMulti, nlpMulti, nlpSingle, nlpSingle } },
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
#if defined(HAS_GUROBI) && GRB_VERSION_MAJOR < 9
                // Older Gurobi versions do not get the recommended nonconvex quadratic strategy.
                if(backend == ES_MIPSolver::Gurobi && nonconvex)
                    expected = nlpMulti;
#endif
                auto strategy = discrete ? expected.discrete : expected.continuous;
                if(env->results->usedSolutionStrategy != strategy)
                {
                    std::cout << names[column] << " default strategy changed for " << test.name << ", discrete "
                              << discrete << ", maximize " << maximize << ": expected " << static_cast<int>(strategy)
                              << ", got " << static_cast<int>(env->results->usedSolutionStrategy) << '\n';
                    return false;
                }
                checked++;
            }
        }
    }
    std::cout << "Checked " << checked << " default strategy combinations for " << names[column] << '\n';
    return true;
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
        return testDefaultStrategies(ES_MIPSolver::Cbc, 0) ? 0 : -1;
#endif
#ifdef HAS_HIGHS
    case 2:
        return testDefaultStrategies(ES_MIPSolver::Highs, 1) ? 0 : -1;
#endif
#ifdef HAS_CPLEX
    case 3:
        return testDefaultStrategies(ES_MIPSolver::Cplex, 2) ? 0 : -1;
#endif
#ifdef HAS_GUROBI
    case 4:
        return testDefaultStrategies(ES_MIPSolver::Gurobi, 3) ? 0 : -1;
#endif
    default:
        std::cout << "Strategy selection test " << choice << " does not exist\n";
        return -1;
    }
}
