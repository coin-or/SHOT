/** SHOT CONOPT integration tests. Licensed under EPL-2.0. */
#include "../src/Solver.h"
#include "../src/Settings.h"
#include "../src/Results.h"
#include "../src/PrimalSolver.h"
#include "../src/TaskHandler.h"
#include "../src/EventHandler.h"
#include <cstdlib>
#include "../src/Model/Problem.h"
#include "../src/NLPSolver/NLPSolverConopt.h"
#include "../src/Tasks/TaskSelectPrimalCandidatesFromNLP.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace SHOT;

namespace
{
void require(bool condition, const char* message)
{
    if(!condition)
        throw std::runtime_error(message);
}
void expect(NLPSolverConopt& nlp, double objective)
{
    auto status = nlp.solveProblem();
    require(status == E_NLPSolutionStatus::Optimal || status == E_NLPSolutionStatus::Feasible, "NLP failed");
    require(std::abs(nlp.getObjectiveValue() - objective) < 1e-5, "Incorrect objective");
}
ProblemPtr makeProblem(EnvironmentPtr env, int part)
{
    auto problem = std::make_shared<Problem>(env);
    auto x = std::make_shared<Variable>("x", E_VariableType::Real, 0.1, 3.0);
    auto y = std::make_shared<Variable>("y", E_VariableType::Real, -3.0, 3.0);
    // x is deliberately not the first column.
    problem->add(Variables { y, x });
    auto ex = std::make_shared<ExpressionVariable>(x);
    auto ey = std::make_shared<ExpressionVariable>(y);
    auto square = std::make_shared<ExpressionSquare>(ex);
    if(part == 2)
    {
        auto obj = std::make_shared<NonlinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        obj->add(std::make_shared<ExpressionSum>(square, std::make_shared<ExpressionSquare>(ey)));
        problem->add(obj);
    }
    else
    {
        // A mixed affine/nonlinear expression, plus an explicit constant.
        auto obj = std::make_shared<NonlinearObjectiveFunction>(
            part == 3 ? E_ObjectiveFunctionDirection::Maximize : E_ObjectiveFunctionDirection::Minimize);
        obj->add(std::make_shared<ExpressionSum>(square, ey));
        obj->constant = 5.0;
        problem->add(obj);
        auto row = std::make_shared<NonlinearConstraint>("range", square, 3.0, 6.0);
        row->constant = 2.0; // 1 <= x*x <= 4.
        problem->add(row);
        auto linear = std::make_shared<LinearConstraint>("y", 0.0, 0.0);
        linear->add(std::make_shared<LinearTerm>(1.0, y));
        problem->add(linear);
    }
    problem->finalize();
    return problem;
}
}
int ConoptTest(int argc, char* argv[])
{
    try
    {
        int part = argc > 1 ? std::stoi(argv[1]) : 1;
        Solver solver;
        auto env = solver.getEnvironment();
        if(argc > 2)
        {
            solver.updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Trace));
            solver.updateSetting("Output.Console.PrimalSolver.Show", true);
            solver.updateLogLevels();
        }
        auto problem = makeProblem(env, part);
        env->problem = problem;
        NLPSolverConopt nlp(env, problem);
        nlp.setStartingPoint({ 0, 1 }, { 0.0, 1.5 });
        if(part == 1)
            expect(nlp, 6.0);
        else if(part == 2)
            expect(nlp, 0.01);
        else if(part == 3)
            expect(nlp, 9.0);
        else if(part == 4)
        {
            nlp.fixVariables({ 1 }, { 2.0 });
            expect(nlp, 9.0);
            nlp.unfixVariables();
            expect(nlp, 6.0);
            nlp.updateVariableLowerBound(1, 1.5);
            nlp.fixVariables({ 1 }, { 2.0 });
            expect(nlp, 9.0);
            nlp.unfixVariables();
            expect(nlp, 7.25);
            require(nlp.getVariableLowerBounds()[1] == 1.5, "Lost updated bound after unfix");
        }
        else if(part == 5)
        {
            expect(nlp, 6.0);
            nlp.fixVariables({ 1 }, { 10.0 });
            require(nlp.solveProblem() == E_NLPSolutionStatus::Infeasible, "Invalid fix accepted");
            require(nlp.getSolution().empty(), "Stale solution after failure");
            nlp.unfixVariables();
            env->settings->updateSetting("Primal.FixedInteger.TimeLimit", 0.0);
            require(nlp.solveProblem() == E_NLPSolutionStatus::TimeLimit, "Time limit ignored");
            require(nlp.getSolution().empty(), "Stale solution after time limit");
            env->settings->updateSetting("Primal.FixedInteger.TimeLimit", 10.0);
            expect(nlp, 6.0);
            env->tasks->terminate();
            require(nlp.solveProblem() == E_NLPSolutionStatus::Error, "Termination ignored");
        }
        else if(part == 6)
        {
            // Exercise the production task constructor for both source selections.
            env->reformulatedProblem = problem;
            env->settings->updateSetting("Primal.FixedInteger.Solver", static_cast<int>(ES_PrimalNLPSolver::Conopt));
            env->results->createIteration();
            env->settings->updateSetting("Primal.FixedInteger.CreateInfeasibilityCut", false);
            env->settings->updateSetting("Dual.HyperplaneCuts.UseIntegerCuts", false);
            PrimalFixedNLPCandidate candidate { };
            candidate.point = { 0.0, 1.5 };
            candidate.sourceType = E_PrimalNLPSource::FirstSolution;
            env->primalSolver->fixedPrimalNLPCandidates.push_back(candidate);
            TaskSelectPrimalCandidatesFromNLP original(env, false, true);
            original.run();
            require(env->solutionStatistics.numberOfProblemsFixedNLP == 1, "Original NLP was not invoked");
            require(env->results->hasPrimalSolution(), "Original candidate not accepted");
            env->primalSolver->fixedPrimalNLPCandidates.clear();
            env->primalSolver->fixedPrimalNLPCandidates.push_back(candidate);
            require(env->results->usedPrimalNLPSolver == ES_PrimalNLPSolver::Conopt, "Wrong original solver");
            TaskSelectPrimalCandidatesFromNLP reformulated(env, true, true);
            reformulated.run();
            require(env->solutionStatistics.numberOfProblemsFixedNLP == 2, "Reformulated NLP was not invoked");
            require(env->results->usedPrimalNLPSolver == ES_PrimalNLPSolver::Conopt, "Wrong reformulated solver");
            require(Solver::hasNLPSolver(ES_PrimalNLPSolver::Conopt), "Missing solver availability");
            expect(nlp, 6.0);
        }
        else if(part == 7)
        {
            auto linearProblem = std::make_shared<Problem>(env);
            auto x = std::make_shared<Variable>("x", E_VariableType::Real, -1.0, 3.0);
            linearProblem->add(Variables { x });
            auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Maximize, 5.0);
            objective->add(std::make_shared<LinearTerm>(2.0, x));
            linearProblem->add(objective);
            linearProblem->finalize();
            NLPSolverConopt linear(env, linearProblem);
            expect(linear, 11.0);
            linear.fixVariables({ 0 }, { 1.0 });
            expect(linear, 7.0);
            auto constantProblem = std::make_shared<Problem>(env);
            auto y = std::make_shared<Variable>("y", E_VariableType::Real, 0.0, 1.0);
            constantProblem->add(Variables { y });
            constantProblem->add(
                std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize, 7.0));
            constantProblem->finalize();
            NLPSolverConopt constant(env, constantProblem);
            expect(constant, 7.0);
        }
        else if(part == 8)
        {
            // The free objective row pushes a 1000-row NLP over the demo limit.
            if(std::getenv("CONOPT_LICENSE_TEXT"))
            {
                std::cout << "Skipping demo limit test with configured license\n";
                return 0;
            }
            auto large = std::make_shared<Problem>(env);
            auto x = std::make_shared<Variable>("x", E_VariableType::Real, 0.1, 3.0);
            large->add(Variables { x });
            auto expression = std::make_shared<ExpressionSquare>(std::make_shared<ExpressionVariable>(x));
            auto objective = std::make_shared<NonlinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
            objective->add(expression);
            large->add(objective);
            for(int i = 0; i < 1000; ++i)
                large->add(std::make_shared<NonlinearConstraint>("c" + std::to_string(i), expression, 1.0, 4.0));
            large->finalize();
            NLPSolverConopt limited(env, large);
            require(limited.solveProblem() == E_NLPSolutionStatus::Error, "Expected demo limit error");
            require(limited.getSolution().empty(), "Demo failure returned a point");
            env->problem = large;
            env->reformulatedProblem = large;
            env->settings->updateSetting("Primal.FixedInteger.Solver", static_cast<int>(ES_PrimalNLPSolver::Conopt));
            env->settings->updateSetting("Primal.FixedInteger.CreateInfeasibilityCut", false);
            env->settings->updateSetting("Dual.HyperplaneCuts.UseIntegerCuts", false);
            env->results->createIteration();
            PrimalFixedNLPCandidate candidate { };
            candidate.point = { 1.5 };
            candidate.sourceType = E_PrimalNLPSource::FirstSolution;
            env->primalSolver->fixedPrimalNLPCandidates.push_back(candidate);
            TaskSelectPrimalCandidatesFromNLP task(env, false, true);
            task.run();
            require(!env->tasks->isTerminated(), "Demo failure terminated the overall solve");
            require(!env->results->hasPrimalSolution(), "Demo failure submitted a candidate");
            expect(nlp, 6.0); // A subsequent, smaller NLP must still work.
        }
        else if(part == 11 || part == 12)
        {
            env->settings->updateSetting("Primal.FixedInteger.IterationLimit", 0);
            nlp.setStartingPoint({ 0, 1 }, { 0.0, part == 11 ? 0.1 : 1.5 });
            auto status = nlp.solveProblem();
            if(part == 11)
            {
                // Conopt 4.39.2 can report solver status 11 at a zero iteration
                // limit. Neither that nor an iteration interrupt proves infeasibility.
                require(status == E_NLPSolutionStatus::IterationLimit || status == E_NLPSolutionStatus::Error,
                    "Interrupted solve incorrectly classified as infeasible");
                require(nlp.getSolution().empty(), "Infeasible iterate exposed after iteration limit");
                require(std::isnan(nlp.getObjectiveValue()), "Failed solve exposed an objective");
            }
            else
            {
                require(status == E_NLPSolutionStatus::Feasible, "Feasible limit candidate discarded");
                require(std::abs(nlp.getObjectiveValue() - 7.25) < 1e-5, "Incorrect limit candidate");
            }
            env->settings->updateSetting("Primal.FixedInteger.IterationLimit", 10000000);
            expect(nlp, 6.0);
        }
        else if(part == 10)
        {
            auto coupled = std::make_shared<Problem>(env);
            auto x = std::make_shared<Variable>("x", E_VariableType::Real, -3.0, 3.0);
            auto y = std::make_shared<Variable>("y", E_VariableType::Real, -3.0, 3.0);
            coupled->add(Variables { x, y });
            auto objective = std::make_shared<QuadraticObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
            objective->add(std::make_shared<QuadraticTerm>(1.0, x, x));
            objective->add(std::make_shared<QuadraticTerm>(2.0, x, y));
            objective->add(std::make_shared<QuadraticTerm>(2.0, y, y));
            objective->add(std::make_shared<LinearTerm>(-4.0, x));
            objective->add(std::make_shared<LinearTerm>(-6.0, y));
            objective->constant = 5.0;
            coupled->add(objective);
            coupled->finalize();
            NLPSolverConopt cross(env, coupled);
            expect(cross, 0.0);
            require(std::abs(cross.getSolution(0) - 1.0) < 1e-5, "Incorrect cross derivative optimum");
            cross.fixVariables({ 0 }, { 2.0 });
            expect(cross, 0.5);
        }
        else if(part == 9)
        {
            int calls = 0;
            env->events->registerCallback(E_EventType::UserTerminationCheck,
                [&calls]()
                {
                    ++calls;
                    return true;
                });
            require(nlp.solveProblem() == E_NLPSolutionStatus::Error, "User callback termination ignored");
            require(calls > 0 && env->tasks->isTerminated(), "Termination callback not called");
            require(nlp.getSolution().empty(), "Cancelled solve returned a stale point");
        }
        else
            return 1;
        return 0;
    }
    catch(const std::exception& error)
    {
        std::cerr << "CONOPT test failed: " << error.what() << '\n';
        return 1;
    }
}
