/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskPerformConvexBounding.h"

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

TaskPerformConvexBounding::TaskPerformConvexBounding(EnvironmentPtr envPtr) : TaskBase(envPtr) { }

TaskPerformConvexBounding::~TaskPerformConvexBounding() = default;

void TaskPerformConvexBounding::run()
{
    /*
        if(env->solutionStatistics.numberOfHyperplanesWithConvexSource == 0)
        {
            env->output->outputInfo(
                " Convex bounding not performed since no hyperplanes with convex source have been added.");
            return;
        }

        if(env->solutionStatistics.numberOfHyperplanesWithNonconvexSource == 0)
        {
            env->output->outputInfo(
                " Convex bounding not performed since no hyperplanes with nonconvex source have been added.");
            return;
        }

        if(lastNumberOfHyperplanesWithNonconvexSource == env->solutionStatistics.numberOfHyperplanesWithNonconvexSource
            && lastNumberOfHyperplanesWithConvexSource == env->solutionStatistics.numberOfHyperplanesWithConvexSource)
        {
            env->output->outputInfo(" Convex bounding not performed since no hyperplanes with both convex and nonconvex
       " "source have been added."); return;
        }*/

    /*if(this->idleIterations < env->settings->getSetting<int>("ConvexBounding.IdleIterations", "Dual"))
    {
        this->idleIterations++;
        env->output->outputInfo(" Convex bounding not performed since number of idle iterations has not been met.");
        return;
    }

    this->idleIterations = 0;*/

    lastNumberOfHyperplanesWithConvexSource = env->solutionStatistics.numberOfHyperplanesWithConvexSource;
    lastNumberOfHyperplanesWithNonconvexSource = env->solutionStatistics.numberOfHyperplanesWithNonconvexSource;

    env->output->outputInfo(" Performing convex bounding.");

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

    env->output->outputDebug("  Creating convex bounding problem");

    taskCreateMIPProblem = std::make_shared<TaskCreateMIPProblem>(env, MIPSolver, env->reformulatedProblem);
    taskCreateMIPProblem->run();

    int numberHyperplanesAdded = 0;

    for(auto HP : env->dualSolver->generatedHyperplanes)
    {
        if(HP->sourceHyperplane->isGlobal)
        {
            env->output->outputDebug("Convex hyperplane added to convex bounding problem");
            if(MIPSolver->createHyperplane(HP->sourceHyperplane))
                numberHyperplanesAdded++;
        }
    }

    double currDual = env->results->getGlobalDualBound();

    env->output->outputDebug(fmt::format(
        "   Number of hyperplanes added: {}/{}", numberHyperplanesAdded, env->dualSolver->generatedHyperplanes.size()));

    int iterationNumber = env->results->getCurrentIteration()->iterationNumber;

    if(env->settings->getSetting<bool>("Debug.Enable", "Output"))
    {
        auto filename = fmt::format("{}/convexbounding_problem{}.lp",
            env->settings->getSetting<std::string>("Debug.Path", "Output"), iterationNumber - 1);

        MIPSolver->writeProblemToFile(filename);
    }

    auto timeLim = env->settings->getSetting<double>("TimeLimit", "Termination") - env->timing->getElapsedTime("Total");
    MIPSolver->setTimeLimit(timeLim);

    if(MIPSolver->getDiscreteVariableStatus() && env->results->hasPrimalSolution())
    {
        auto primalSol = env->results->primalSolution;
        env->reformulatedProblem->augmentAuxiliaryVariableValues(primalSol);

        if(env->dualSolver->MIPSolver->hasDualAuxiliaryObjectiveVariable())
        {
            primalSol.push_back(env->results->getPrimalBound());
        }

        MIPSolver->addMIPStart(primalSol);
    }

    MIPSolver->setSolutionLimit(2100000000);

    env->output->outputInfo("  Convex bounding problem created");
    auto solutionStatus = MIPSolver->solveProblem();

    env->output->outputDebug(
        fmt::format("        Convex bounding problem solved with return code: {}", (int)solutionStatus));

    auto solutionPoints = MIPSolver->getAllVariableSolutions();
    double objectiveBound = MIPSolver->getDualObjectiveValue();
    env->output->outputDebug(fmt::format("        Convex bounding problem objective bound: {}", objectiveBound));

    if(solutionPoints.size() > 0)
    {
        env->output->outputDebug(
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

    if(currDual != env->results->getGlobalDualBound())
        env->output->outputDebug(" Convex bounding returned new global bound.");

    env->output->outputDebug(" Convex bounding finished.");
}

std::string TaskPerformConvexBounding::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT