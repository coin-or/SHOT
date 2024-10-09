/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskPerformDualBounding.h"

#include "../DualSolver.h"
#include "../Output.h"
#include "../Settings.h"
#include "../Timing.h"

#include "../MIPSolver/IMIPSolver.h"

#ifdef HAS_CPLEX
#include "../MIPSolver/MIPSolverCplex.h"
#include "../MIPSolver/MIPSolverCplexSingleTree.h"
#include "../MIPSolver/MIPSolverCplexSingleTreeLegacy.h"
#endif

#ifdef HAS_GUROBI
#include "../MIPSolver/MIPSolverGurobi.h"
#include "../MIPSolver/MIPSolverGurobiSingleTree.h"
#endif

#ifdef HAS_CBC
#include "../MIPSolver/MIPSolverCbc.h"
#endif

#include "../Results.h"
#include "../Tasks/TaskCreateMIPProblem.h"

#include "../PrimalSolver.h"

namespace SHOT
{

TaskPerformDualBounding::TaskPerformDualBounding(EnvironmentPtr envPtr) : TaskBase(envPtr) { }

TaskPerformDualBounding::~TaskPerformDualBounding() = default;

void TaskPerformDualBounding::run()
{

    if(env->solutionStatistics.numberOfHyperplanesWithConvexSource == 0)
    {
        env->output->outputInfo(
            " Dual bounding not performed since no hyperplanes with convex source have been added.");
        return;
    }

    if(env->solutionStatistics.numberOfHyperplanesWithNonconvexSource == 0)
    {
        env->output->outputInfo(
            " Dual bounding not performed since no hyperplanes with nonconvex source have been added.");
        return;
    }

    if(lastNumberOfHyperplanesWithNonconvexSource == env->solutionStatistics.numberOfHyperplanesWithNonconvexSource
        && lastNumberOfHyperplanesWithConvexSource == env->solutionStatistics.numberOfHyperplanesWithConvexSource)
    {
        env->output->outputInfo(
            " Dual bounding not performed since no hyperplanes with both convex and nonconvex source have been added.");
        return;
    }

    lastNumberOfHyperplanesWithConvexSource = env->solutionStatistics.numberOfHyperplanesWithConvexSource;
    lastNumberOfHyperplanesWithNonconvexSource = env->solutionStatistics.numberOfHyperplanesWithNonconvexSource;

    env->output->outputInfo(" Performing dual bounding.");

    MIPSolverPtr MIPSolver;

#ifdef HAS_CPLEX
    if(env->results->usedMIPSolver == ES_MIPSolver::Cplex)
    {
        MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverCplex>(env));
    }
#endif

#ifdef HAS_GUROBI
    if(env->results->usedMIPSolver == ES_MIPSolver::Gurobi)
    {
        MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverGurobi>(env));
    }
#endif

#ifdef HAS_CBC
    if(env->results->usedMIPSolver == ES_MIPSolver::Cbc)
    {
        MIPSolver = MIPSolverPtr(std::make_shared<MIPSolverCbc>(env));
    }
#endif

    assert(MIPSolver);

    if(!MIPSolver->initializeProblem())
        throw Exception(" Cannot initialize selected MIP solver.");

    env->output->outputInfo("  Creating dual bounding problem");

    taskCreateMIPProblem = std::make_shared<TaskCreateMIPProblem>(env, MIPSolver, env->reformulatedProblem);
    taskCreateMIPProblem->run();

    int numberHyperplanesAdded = 0;

    for(auto HP : env->dualSolver->generatedHyperplanes)
    {
        if(HP.isSourceConvex)
        {
            if(MIPSolver->createHyperplane((Hyperplane)HP))
                numberHyperplanesAdded++;
        }
    }

    env->output->outputInfo(fmt::format(
        "   Number of hyperplanes added: {}/{}", numberHyperplanesAdded, env->dualSolver->generatedHyperplanes.size()));

    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        MIPSolver->writeProblemToFile(
            env->settings->getSetting<std::string>("Debug.Path", "Output") + "/dualbounding_problem.lp");
    }

    auto timeLim = env->settings->getSetting<double>("TimeLimit", "Termination") - env->timing->getElapsedTime("Total");
    MIPSolver->setTimeLimit(timeLim);

    if(MIPSolver->getDiscreteVariableStatus() && env->results->hasPrimalSolution())
    {
        auto primalSol = env->results->primalSolution;
        env->reformulatedProblem->augmentAuxiliaryVariableValues(primalSol);
        MIPSolver->addMIPStart(primalSol);
    }

    MIPSolver->setSolutionLimit(2100000000);

    env->output->outputInfo("  Dual bounding problem created");
    auto solutionStatus = MIPSolver->solveProblem();

    env->output->outputInfo(
        fmt::format("        Dual bounding problem solved with return code: {}", (int)solutionStatus));

    auto solutionPoints = MIPSolver->getAllVariableSolutions();
    double objectiveBound = MIPSolver->getDualObjectiveValue();
    env->output->outputInfo(fmt::format("        Dual bounding problem objective bound: {}", objectiveBound));

    int iterationNumber = env->results->getCurrentIteration()->iterationNumber;

    if(solutionPoints.size() > 0)
    {
        env->output->outputInfo(
            fmt::format("        Number of solutions in solution pool: {} ", solutionPoints.size()));

        double objectiveValue = MIPSolver->getObjectiveValue();
        DualSolution sol = { solutionPoints.at(0).point, E_DualSolutionSource::ConvexBounding, objectiveValue,
            iterationNumber, false };
        env->dualSolver->addDualSolutionCandidate(sol);

        if(env->reformulatedProblem->antiEpigraphObjectiveVariable)
        {
            for(auto& SOL : solutionPoints)
                SOL.point.at(env->reformulatedProblem->antiEpigraphObjectiveVariable->index) = objectiveValue;
        }

        env->primalSolver->addPrimalSolutionCandidates(solutionPoints, E_PrimalSolutionSource::ConvexBounding);

        for(auto& SOL : solutionPoints)
            env->primalSolver->addFixedNLPCandidate(SOL.point, E_PrimalNLPSource::FirstSolutionNewDualBound,
                SOL.objectiveValue, SOL.iterFound, SOL.maxDeviation);
    }
    else
    {
        DualSolution sol = { {}, E_DualSolutionSource::ConvexBounding, objectiveBound, iterationNumber, false };
        env->dualSolver->addDualSolutionCandidate(sol);
    }

    env->output->outputInfo(" Dual bounding finished.");
}

std::string TaskPerformDualBounding::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT