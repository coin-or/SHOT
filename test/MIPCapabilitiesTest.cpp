/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "../src/Solver.h"
#include "../src/Results.h"
#include "../src/MIPSolver/MIPSolverCapabilities.h"
#include "../src/Model/Problem.h"
#include "../src/Tasks/TaskCreateMIPProblem.h"

#ifdef HAS_CBC
#include "../src/MIPSolver/MIPSolverCbc.h"
#endif
#ifdef HAS_CPLEX
#include "../src/MIPSolver/MIPSolverCplex.h"
#endif
#ifdef HAS_GUROBI
#include "../src/MIPSolver/MIPSolverGurobi.h"
#endif
#ifdef HAS_HIGHS
#include "../src/MIPSolver/MIPSolverHighs.h"
#endif

#include <cmath>
#include <cstdio>
#include <iostream>

using namespace SHOT;

namespace
{
void configure(Solver& solver, ES_MIPSolver backend)
{
    solver.updateSetting("Dual.MIP.Solver", static_cast<int>(backend));
    solver.updateSetting("Dual.MIP.NumberOfThreads", 1);
    solver.updateSetting("Primal.FixedInteger.Use", false);
    solver.updateSetting("Primal.PolishSolution.NumberOfPoints", 0);
    solver.updateSetting("Model.BoundTightening.FeasibilityBased.Use", false);
    solver.updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
    solver.updateSetting("Termination.TimeLimit", 10.0);
}

ProblemPtr makeLinearProblem(EnvironmentPtr env, E_VariableType type = E_VariableType::Real, double semiBound = 0.0,
    bool negative = false, bool forceNonzero = false)
{
    auto problem = std::make_shared<Problem>(env);
    auto x = std::make_shared<Variable>("x", type, negative ? -5.0 : 0.0, negative ? 0.0 : 5.0, semiBound);
    problem->add(x);
    auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    objective->add(std::make_shared<LinearTerm>(negative ? -1.0 : 1.0, x));
    problem->add(objective);

    if(forceNonzero)
    {
        auto constraint = std::make_shared<LinearConstraint>(
            "nonzero", negative ? SHOT_DBL_MIN : 1.0, negative ? -1.0 : SHOT_DBL_MAX);
        constraint->add(std::make_shared<LinearTerm>(1.0, x));
        problem->add(constraint);
    }

    problem->finalize();

    return problem;
}

bool lookupTest()
{
    for(auto backend :
        { ES_MIPSolver::Cbc, ES_MIPSolver::Cplex, ES_MIPSolver::Gurobi, ES_MIPSolver::Highs, ES_MIPSolver::NotUsed })
    {
        auto caps = getMIPSolverCapabilities(backend);

        if(caps.has_value() != Solver::hasMIPSolver(backend))
            return false;

        if(!caps)
            continue;

        bool quadratic = backend == ES_MIPSolver::Cplex || backend == ES_MIPSolver::Gurobi;

        if(caps->convexQuadraticObjective != quadratic || caps->nonconvexQuadraticObjective != quadratic
            || caps->convexQuadraticConstraints != quadratic || caps->singleTree != quadratic
            || caps->nonconvexQuadraticConstraints != (backend == ES_MIPSolver::Gurobi)
            || caps->nonconvexObjectiveExcludesQuadraticConstraints != (backend == ES_MIPSolver::Cplex)
            || caps->SOS1 != (backend != ES_MIPSolver::Highs) || caps->SOS2 != (backend != ES_MIPSolver::Highs))
            return false;

        auto semi = backend == ES_MIPSolver::Highs || backend == ES_MIPSolver::Cplex
            ? E_SemiVariableSupport::NonnegativeInterval
            : E_SemiVariableSupport::SignedInterval;

        if(caps->semicontinuous != semi || caps->semiinteger != semi)
            return false;

        // Settings verification must use the selected backend's finite default without overriding user bounds.
        for(bool userBounds : { false, true })
        {
            Solver solver;
            configure(solver, backend);

            if(userBounds)
            {
                solver.updateSetting("Model.Variables.Continuous.MinimumLowerBound", -1e60);
                solver.updateSetting("Model.Variables.Continuous.MaximumUpperBound", 1e60);
            }

            if(!solver.setProblem(makeLinearProblem(solver.getEnvironment())))
                return false;

            double expectedBound = userBounds ? 1e60 : (backend == ES_MIPSolver::Cbc ? 1e50 : 1e20);

            if(caps->unboundedVariableBound != (backend == ES_MIPSolver::Cbc ? 1e50 : 1e20)
                || solver.getSetting<double>("Model.Variables.Continuous.MinimumLowerBound") != -expectedBound
                || solver.getSetting<double>("Model.Variables.Continuous.MaximumUpperBound") != expectedBound)
                return false;
        }
    }

    return !getMIPSolverCapabilities(static_cast<ES_MIPSolver>(-1));
}

bool validationTest()
{
    Solver solver;
    auto problem = makeLinearProblem(solver.getEnvironment());
    MIPSolverCapabilities caps;

    if(getUnsupportedMIPFeature(caps, *problem))
        return false;

    problem->add(std::make_shared<SpecialOrderedSet>(E_SOSType::One, problem->allVariables));

    if(!getUnsupportedMIPFeature(caps, *problem))
        return false;

    caps.SOS1 = true;

    if(getUnsupportedMIPFeature(caps, *problem))
        return false;

    problem->specialOrderedSets.front()->type = E_SOSType::Two;

    if(!getUnsupportedMIPFeature(caps, *problem))
        return false;

    caps.SOS2 = true;

    if(getUnsupportedMIPFeature(caps, *problem))
        return false;

    for(auto type : { E_VariableType::Semicontinuous, E_VariableType::Semiinteger })
    {
        caps = { };
        problem = makeLinearProblem(solver.getEnvironment(), type, 2.5);

        if(!getUnsupportedMIPFeature(caps, *problem))
            return false;

        auto& support = type == E_VariableType::Semicontinuous ? caps.semicontinuous : caps.semiinteger;
        support = E_SemiVariableSupport::NonnegativeInterval;

        if(getUnsupportedMIPFeature(caps, *problem))
            return false;

        problem = makeLinearProblem(solver.getEnvironment(), type, -2.5, true);

        if(!getUnsupportedMIPFeature(caps, *problem))
            return false;

        support = E_SemiVariableSupport::SignedInterval;

        if(getUnsupportedMIPFeature(caps, *problem))
            return false;
    }

    return true;
}

bool sosTest()
{
    for(auto backend : Solver::getSupportedMIPSolvers())
    {
        for(auto type : { E_SOSType::One, E_SOSType::Two })
        {
            Solver solver;
            configure(solver, backend);
            auto problem = std::make_shared<Problem>(solver.getEnvironment());
            Variables variables;
            auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Maximize);

            for(int i = 0; i < 3; ++i)
            {
                auto variable = std::make_shared<Variable>("x" + std::to_string(i), E_VariableType::Real, 0.0, 2.0);
                variables.push_back(variable);
                problem->add(variable);
                objective->add(std::make_shared<LinearTerm>(i == 1 ? 1.0 : 3.0, variable));
            }

            problem->add(objective);
            problem->add(std::make_shared<SpecialOrderedSet>(type, variables, VectorDouble { 1.0, 2.0, 3.0 }));
            problem->finalize();
            bool accepted = solver.setProblem(problem);

            if(backend == ES_MIPSolver::Highs)
            {
                if(accepted)
                    return false;

                continue;
            }

            if(!accepted || solver.getEnvironment()->results->usedSolutionStrategy == E_SolutionStrategy::NLP
                || !solver.solveProblem() || !solver.hasPrimalSolution()
                || std::abs(solver.getPrimalBound() - (type == E_SOSType::One ? 6.0 : 8.0)) > 1e-6
                || std::abs(solver.getGlobalDualBound() - solver.getPrimalBound()) > 1e-6)
            {
                std::cout << "SOS model failed for backend " << static_cast<int>(backend) << '\n';

                return false;
            }
        }
    }

    return true;
}

bool semiTest()
{
    for(auto backend : Solver::getSupportedMIPSolvers())
    {
        for(auto type : { E_VariableType::Semicontinuous, E_VariableType::Semiinteger })
        {
            for(bool negative : { false, true })
            {
                for(bool forceNonzero : { false, true })
                {
                    Solver solver;
                    configure(solver, backend);
                    auto problem = makeLinearProblem(
                        solver.getEnvironment(), type, negative ? -2.5 : 2.5, negative, forceNonzero);
                    bool accepted = solver.setProblem(problem);

                    if(negative && (backend == ES_MIPSolver::Highs || backend == ES_MIPSolver::Cplex))
                    {
                        if(accepted)
                            return false;

                        continue;
                    }

                    double expected = !forceNonzero ? 0.0 : (type == E_VariableType::Semiinteger ? 3.0 : 2.5);

                    if(!accepted || !solver.solveProblem() || !solver.hasPrimalSolution()
                        || std::abs(solver.getPrimalBound() - expected) > 1e-6
                        || std::abs(solver.getGlobalDualBound() - expected) > 1e-6)
                    {
                        std::cout << "Semi-variable model failed for backend " << static_cast<int>(backend)
                                  << ", negative " << negative << ", nonzero " << forceNonzero << ", accepted "
                                  << accepted << ", bound " << solver.getPrimalBound() << '\n';

                        return false;
                    }
                }
            }
        }
    }

    return true;
}

bool freeVariableParsingTest()
{
    for(auto backend : Solver::getSupportedMIPSolvers())
    {
        if(!getMIPSolverCapabilities(backend)->convexQuadraticConstraints)
            continue;

        Solver solver;
        configure(solver, backend);
        solver.updateSetting("Model.BoundTightening.FeasibilityBased.Use", true);

        // Early compatibility checks must not replace the reader's missing-bound defaults with solver infinity
        // thresholds. Propagating those thresholds through a linear row can create huge finite bounds in the QCP.
        if(!solver.setProblem("data/instances/ampl_mp/ellipse_min_rot.osil") || !solver.solveProblem()
            || !solver.hasPrimalSolution() || std::abs(solver.getPrimalBound() + 4.0909618) > 1e-5
            || std::abs(solver.getGlobalDualBound() + 4.0909618) > 1e-5)
            return false;
    }

    return true;
}

#if defined(HAS_HIGHS)
using TrackingBackend = MIPSolverHighs;
constexpr auto trackingSolver = ES_MIPSolver::Highs;
#elif defined(HAS_CBC)
using TrackingBackend = MIPSolverCbc;
constexpr auto trackingSolver = ES_MIPSolver::Cbc;
#elif defined(HAS_CPLEX)
using TrackingBackend = MIPSolverCplex;
constexpr auto trackingSolver = ES_MIPSolver::Cplex;
#else
using TrackingBackend = MIPSolverGurobi;
constexpr auto trackingSolver = ES_MIPSolver::Gurobi;
#endif

class TrackingMIPSolver : public TrackingBackend
{
public:
    using TrackingBackend::TrackingBackend;
    bool failVariable = false;
    bool failFinalization = false;
    int finalizations = 0;
    int settingsInitializations = 0;

    bool addVariable(std::string name, E_VariableType type, double lower, double upper, double semiBound) override
    {
        return !failVariable && TrackingBackend::addVariable(name, type, lower, upper, semiBound);
    }

    bool finalizeProblem() override
    {
        ++finalizations;

        return !failFinalization && TrackingBackend::finalizeProblem();
    }

    void initializeSolverSettings() override
    {
        ++settingsInitializations;
        TrackingBackend::initializeSolverSettings();
    }
};

bool constructionTest()
{
    for(int failure : { 0, 1, 2 })
    {
        Solver solver;
        configure(solver, trackingSolver);
        auto env = solver.getEnvironment();
        auto problem = makeLinearProblem(env);
        env->problem = problem;
        env->reformulatedProblem = problem;
        auto backend = std::make_shared<TrackingMIPSolver>(env);

        if(!backend->initializeProblem())
            return false;

        backend->failVariable = failure == 1;
        backend->failFinalization = failure == 2;
        bool threw = false;

        try
        {
            TaskCreateMIPProblem(env, backend, problem, TrackingBackend::capabilities).run();
        }
        catch(const Exception&)
        {
            threw = true;
        }

        if(threw != (failure != 0) || backend->finalizations != (failure == 1 ? 0 : 1)
            || backend->settingsInitializations != (failure == 0 ? 1 : 0))
            return false;
    }

    return true;
}

bool fallbackTest()
{
    for(auto unavailable : { ES_MIPSolver::Cplex, ES_MIPSolver::Gurobi, ES_MIPSolver::Cbc, ES_MIPSolver::Highs })
    {
        if(Solver::hasMIPSolver(unavailable))
            continue;

        for(bool fromFile : { false, true })
        {
            Solver solver;
            configure(solver, unavailable);
            solver.updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::SingleTree));
            solver.updateSetting("Model.Reformulation.Quadratics.Strategy",
                static_cast<int>(ES_QuadraticProblemStrategy::NonconvexQuadraticallyConstrained));
            bool checkedBeforeReformulation = false;
            bool compatibleBeforeReformulation = false;
            solver.registerCallback<BoundTighteningContext>(
                [&](BoundTighteningContext&)
                {
                    checkedBeforeReformulation = true;
                    auto effective = static_cast<ES_MIPSolver>(solver.getSetting<int>("Dual.MIP.Solver"));
                    auto caps = getMIPSolverCapabilities(effective);
                    compatibleBeforeReformulation = caps.has_value()
                        && (caps->singleTree
                            || solver.getSetting<int>("Dual.TreeStrategy")
                                == static_cast<int>(ES_TreeStrategy::MultiTree))
                        && (caps->convexQuadraticObjective
                            || solver.getSetting<int>("Model.Reformulation.Quadratics.Strategy")
                                == static_cast<int>(ES_QuadraticProblemStrategy::Nonlinear));
                });
            bool accepted = fromFile
                ? solver.setProblem("data/ex4.osil")
                : solver.setProblem(makeLinearProblem(solver.getEnvironment(), E_VariableType::Integer));
            auto effective = static_cast<ES_MIPSolver>(solver.getSetting<int>("Dual.MIP.Solver"));

            if(!accepted || !checkedBeforeReformulation || !compatibleBeforeReformulation
                || !Solver::hasMIPSolver(effective) || effective == unavailable)
                return false;

            auto caps = getMIPSolverCapabilities(effective).value();

            if(!caps.singleTree
                && solver.getSetting<int>("Dual.TreeStrategy") != static_cast<int>(ES_TreeStrategy::MultiTree))
                return false;

            if(!caps.convexQuadraticObjective)
            {
                if(solver.getSetting<int>("Model.Reformulation.Quadratics.Strategy")
                        != static_cast<int>(ES_QuadraticProblemStrategy::Nonlinear)
                    || solver.getReformulatedProblem()->properties.numberOfQuadraticConstraints != 0)
                    return false;
            }
        }
    }

    return true;
}
} // namespace

int MIPCapabilitiesTest(int argc, char* argv[])
{
    int choice = 0;

    if(argc < 2 || std::sscanf(argv[1], "%d", &choice) != 1)
        return -1;

    bool passed = false;

    switch(choice)
    {
    case 1:
        passed = lookupTest() && validationTest();
        break;
    case 2:
        passed = sosTest();
        break;
    case 3:
        passed = semiTest();
        break;
    case 4:
        passed = constructionTest();
        break;
    case 5:
        passed = fallbackTest();
        break;
    case 6:
        passed = freeVariableParsingTest();
        break;
    }

    return passed ? 0 : -1;
}
