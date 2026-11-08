/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "../src/Solver.h"
#include "../src/Iteration.h"
#include "../src/PrimalSolver.h"
#include "../src/Results.h"
#include "../src/Model/Problem.h"
#include "../src/Tasks/TaskSelectPrimalCandidatesFromNLP.h"
#include "../src/Tasks/TaskSelectPrimalFixedNLPPointsFromSolutionPool.h"

#include "spdlog/sinks/ostream_sink.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>

using namespace SHOT;

namespace
{
bool expect(bool condition, const std::string& description)
{
    if(!condition)
        std::cout << "  FAILED: " << description << '\n';

    return condition;
}

ProblemPtr makeProblem(EnvironmentPtr env, bool discrete, bool convex = false)
{
    auto problem = std::make_shared<Problem>(env);
    auto x = std::make_shared<Variable>("x", E_VariableType::Real, -2.0, 2.0);
    auto y = std::make_shared<Variable>("y", E_VariableType::Real, -2.0, 2.0);
    problem->add({ x, y });
    auto objective = std::make_shared<QuadraticObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    objective->add(std::make_shared<QuadraticTerm>(1.0, x, x));
    objective->add(std::make_shared<QuadraticTerm>(1.0, convex ? y : x, y));

    if(discrete)
    {
        auto b = std::make_shared<Variable>("b", E_VariableType::Binary, 0.0, 1.0);
        problem->add(b);
        objective->add(std::make_shared<LinearTerm>(0.1, b));
    }

    problem->add(objective);
    problem->finalize();

    return problem;
}

bool defaultsTest()
{
    bool passed = true;

    for(bool discrete : { false, true })
    {
        for(bool convex : { false, true })
        {
            for(int overrideSource : { -1, 0, 1, 3 })
            {
                Solver solver;
                solver.updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
                solver.updateSetting("Model.BoundTightening.FeasibilityBased.Use", false);
                solver.updateSetting("Primal.FixedInteger.Use", false);
                auto env = solver.getEnvironment();

                if(overrideSource >= 0)
                    env->settings->updateSetting(
                        "Primal.FixedInteger.Source", overrideSource, E_SettingPriority::OptionsFile);

                if(!solver.setProblem(makeProblem(env, discrete, convex)))
                    return expect(false, "could not set the problem");

                int expected = overrideSource >= 0 ? overrideSource : (convex ? 3 : (discrete ? 0 : 1));
                passed = expect(solver.getSetting<int>("Primal.FixedInteger.Source") == expected,
                             "wrong source for discrete=" + std::to_string(discrete) + ", convex="
                                 + std::to_string(convex) + ", override=" + std::to_string(overrideSource))
                    && passed;
            }
        }
    }

    Solver solver;
    auto markup = solver.getSettingsAsMarkup();
    passed = expect(markup.find("Nonconvex continuous default: 1") != std::string::npos,
                 "the continuous recommendation is missing from the options reference")
        && passed;

    return passed;
}

bool selectionTest()
{
    bool passed = true;

    for(bool discrete : { false, true })
    {
        for(int limit : { -1, 1, 2, 0 })
        {
            Solver solver;
            solver.updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
            solver.updateSetting("Primal.FixedInteger.Source", 0);
            solver.updateSetting("Primal.FixedInteger.CallStrategy", 0);
            solver.updateSetting("Primal.FixedInteger.OnlyUniqueIntegerCombinations", false);
            auto env = solver.getEnvironment();
            env->problem = makeProblem(env, discrete);
            env->reformulatedProblem = env->problem;

            if(limit >= 0)
                solver.updateSetting("Primal.FixedInteger.MaxStartsPerCall", limit);

            for(int i = 0; i < 8; i++)
            {
                env->results->createIteration();
                auto iteration = env->results->getCurrentIteration();
                iteration->isDualProblemDiscrete = discrete;
                iteration->solutionStatus = E_ProblemSolutionStatus::Feasible;
                VectorDouble point { 0.1 * i, 1.0 };

                if(discrete)
                    point.push_back(i % 2);

                iteration->solutionPoints.push_back(
                    SolutionPoint { point, 0.0, iteration->iterationNumber, { 0, 1.0 }, false, 0.0 });
            }

            auto iteration = env->results->getCurrentIteration();
            iteration->solutionPoints.clear();

            for(auto& previous : env->results->iterations)
            {
                if(previous != iteration)
                    iteration->solutionPoints.push_back(previous->solutionPoints.front());
            }

            VectorDouble lastPoint { 0.7, 1.0 };

            if(discrete)
                lastPoint.push_back(1.0);

            iteration->solutionPoints.push_back(SolutionPoint { lastPoint, 0.0, 8, { 0, 1.0 }, false, 0.0 });
            TaskSelectPrimalFixedNLPPointsFromSolutionPool select(env);
            select.run();
            auto& candidates = env->primalSolver->fixedPrimalNLPCandidates;
            size_t expected = limit == -1 ? 5 : (limit == 0 ? 8 : limit);
            passed = expect(candidates.size() == expected, "wrong number of selected starts") && passed;
            passed = expect(env->primalSolver->usedPrimalNLPCandidates.empty(),
                         "selection marked unsolved candidates as tested")
                && passed;

            if(candidates.size() >= 2)
            {
                env->primalSolver->usedPrimalNLPCandidates.assign(candidates.begin(), candidates.begin() + 2);
                candidates.clear();
                select.run();
                passed = expect(candidates.size() == std::min(expected, size_t(6))
                                 && candidates.front().point.front() == 0.2,
                             "previously unselected starts were lost")
                    && passed;
            }

            candidates.clear();
            env->primalSolver->usedPrimalNLPCandidates.clear();
            solver.updateSetting("Primal.FixedInteger.MaxStartsPerCall", 1);
            solver.updateSetting("Primal.PolishSolution.NumberOfPoints", 8);
            TaskSelectPrimalFixedNLPPointsFromSolutionPool polish(env, true);
            polish.run();
            passed = expect(candidates.size() == 8, "the search limit incorrectly capped final polishing") && passed;
        }
    }

    return passed;
}

#ifdef HAS_IPOPT
bool loggingTest()
{
    std::ostringstream log;
    Solver solver(std::make_shared<spdlog::sinks::ostream_sink_mt>(log));
    solver.updateSetting("Primal.FixedInteger.Solver", static_cast<int>(ES_PrimalNLPSolver::Ipopt));
    solver.updateSetting("Primal.FixedInteger.CreateSupportingHyperplanes", false);
    solver.updateSetting("Output.Console.Iteration.Detail", static_cast<int>(ES_IterationOutputDetail::Full));
    auto env = solver.getEnvironment();
    env->output->setLogLevels(E_LogLevel::Info, E_LogLevel::Off);
    auto problem = std::make_shared<Problem>(env);
    auto x = std::make_shared<Variable>("x", E_VariableType::Real, -2.0, 2.0);
    auto y = std::make_shared<Variable>("y", E_VariableType::Real, -2.0, 2.0);
    problem->add({ x, y });
    auto objective = std::make_shared<QuadraticObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    objective->add(std::make_shared<QuadraticTerm>(1.0, x, x));
    objective->add(std::make_shared<QuadraticTerm>(1.0, y, y));
    problem->add(objective);
    auto constraint = std::make_shared<LinearConstraint>("lower", 1.0, SHOT_DBL_MAX);
    constraint->add(std::make_shared<LinearTerm>(1.0, problem->allVariables[0]));
    constraint->add(std::make_shared<LinearTerm>(1.0, problem->allVariables[1]));
    problem->add(constraint);
    auto quadratic = std::make_shared<QuadraticConstraint>("product", SHOT_DBL_MIN, 0.5);
    quadratic->add(std::make_shared<QuadraticTerm>(1.0, problem->allVariables[0], problem->allVariables[1]));
    problem->add(quadratic);
    problem->finalize();
    env->problem = problem;
    env->reformulatedProblem = problem;

    for(int i = 0; i < 7; i++)
        env->results->createIteration();

    env->solutionStatistics.numberOfProblemsFixedNLP = 40;
    std::vector<PrimalFixedNLPCandidate> candidates;
    candidates.push_back(env->primalSolver->createFixedNLPCandidate(
        { 0.7, 0.7 }, E_PrimalNLPSource::FirstSolution, 0.98, 7, { 0, 0.0 }));
    candidates.push_back(env->primalSolver->createFixedNLPCandidate(
        { 1.0, 1.0 }, E_PrimalNLPSource::InfeasibleSolution, 2.0, 7, { 0, 0.5 }));
    TaskSelectPrimalCandidatesFromNLP solve(env, false);
    auto status = solve.solveFixedNLPCandidates(candidates);
    env->output->flush();
    auto output = log.str();

    return expect(status == E_NLPSolutionStatus::Optimal || status == E_NLPSolutionStatus::Feasible,
               "the NLP did not converge")
        && expect(std::abs(solver.getPrimalBound() - 0.5) < 1e-6, "the NLP solution has the wrong objective")
        && expect(output.find("NLP solve #41 (SHOT iteration 7), start 1/2") != std::string::npos,
            "the first NLP solve and batch counters are missing")
        && expect(output.find("NLP solve #42 (SHOT iteration 7), start 2/2") != std::string::npos,
            "the second NLP solve and batch counters are missing")
        && expect(
            output.find("dual pool point violating the nonlinear constraints (original problem)") != std::string::npos,
            "the candidate source is unclear")
        && expect(output.find("     7: NLPPOOL-O") != std::string::npos,
            "the NLP row does not show the outer iteration and the new pool label")
        && expect(output.find("NLPUNFEA") == std::string::npos, "the ambiguous infeasibility label remains");
}
#endif
} // namespace

int PrimalNLPTest(int argc, char* argv[])
{
    int choice = 1;

    if(argc > 1 && sscanf(argv[1], "%d", &choice) != 1)
        return -1;

    bool passed = false;

    switch(choice)
    {
    case 1:
        passed = defaultsTest();
        break;
    case 2:
        passed = selectionTest();
        break;
#ifdef HAS_IPOPT
    case 3:
        passed = loggingTest();
        break;
#endif
    }

    return passed ? 0 : -1;
}
