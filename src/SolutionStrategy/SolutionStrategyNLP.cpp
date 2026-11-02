/**
        The Supporting Hyperplane Optimization Toolkit (SHOT).

        @author Andreas Lundell, Åbo Akademi University

        @section LICENSE
        This software is licensed under the Eclipse Public License 2.0.
        Please see the README and LICENSE files for more information.
*/

#include "SolutionStrategyNLP.h"

#include "../TaskHandler.h"

#include "../Tasks/TaskFindInteriorPoint.h"
#include "../Tasks/TaskBase.h"
#include "../Tasks/TaskSequential.h"
#include "../Tasks/TaskGoto.h"
#include "../Tasks/TaskConditional.h"

#include "../Tasks/TaskInitializeIteration.h"
#include "../Tasks/TaskTerminate.h"

#include "../Tasks/TaskInitializeDualSolver.h"
#include "../Tasks/TaskCreateDualProblem.h"

#include "../Tasks/TaskExecuteSolutionLimitStrategy.h"
#include "../Tasks/TaskExecuteRelaxationStrategy.h"

#include "../Tasks/TaskPrintIterationReport.h"

#include "../Tasks/TaskSolveIteration.h"

#include "../Tasks/TaskRepairInfeasibleDualProblem.h"

#include "../Tasks/TaskCheckAbsoluteGap.h"
#include "../Tasks/TaskCheckIterationError.h"
#include "../Tasks/TaskCheckIterationLimit.h"
#include "../Tasks/TaskCheckDualStagnation.h"
#include "../Tasks/TaskCheckPrimalStagnation.h"
#include "../Tasks/TaskCheckConstraintTolerance.h"
#include "../Tasks/TaskCheckRelativeGap.h"
#include "../Tasks/TaskCheckTimeLimit.h"
#include "../Tasks/TaskCheckUserTermination.h"

#include "../Tasks/TaskInitializeRootsearch.h"
#include "../Tasks/TaskSelectHyperplanesESH.h"
#include "../Tasks/TaskSelectHyperplanesECP.h"
#include "../Tasks/TaskSelectHyperplanesObjectiveFunction.h"
#include "../Tasks/TaskSelectHyperplanesExternal.h"
#include "../Tasks/TaskAddHyperplanes.h"
#include "../Tasks/TaskCalculateSolutionChangeNorm.h"

#include "../Tasks/TaskAddIntegerCuts.h"

#include "../Tasks/TaskAddPrimalReductionCut.h"
#include "../Tasks/TaskCheckMaxNumberOfPrimalReductionCuts.h"

#include "../Tasks/TaskSelectPrimalCandidatesFromSolutionPool.h"
#include "../Tasks/TaskSelectPrimalCandidatesFromRootsearch.h"
#include "../Tasks/TaskSelectPrimalFixedNLPPointsFromSolutionPool.h"
#include "../Tasks/TaskEnumerateFixedIntegerCombinations.h"
#include "../Tasks/TaskSelectPrimalCandidatesFromNLP.h"
#include "../Tasks/TaskClearFixedPrimalCandidates.h"
#include "../Tasks/TaskSelectPrimalCandidatesFromExternalSource.h"

#include "../Tasks/TaskUpdateInteriorPoint.h"

#include "../Tasks/TaskUpdateExternalDualBound.h"

#include "../Output.h"
#include "../Model/Problem.h"
#include "../Model/ObjectiveFunction.h"
#include "../Settings.h"
#include "../Timing.h"

namespace SHOT
{

SolutionStrategyNLP::SolutionStrategyNLP(EnvironmentPtr envPtr)
{
    env = envPtr;

    env->timing->createTimer("InteriorPointSearch", "- interior point search");

    env->timing->createTimer("DualStrategy", "- dual strategy");
    env->timing->createTimer("DualProblemsRelaxed", "  - solving relaxed problems");
    env->timing->createTimer("DualProblemsDiscrete", "  - solving MIP problems");
    env->timing->createTimer("DualCutGenerationRootSearch", "  - root search for constraint cuts");
    env->timing->createTimer("DualObjectiveRootSearch", "  - root search for objective cut");

    env->timing->createTimer("PrimalStrategy", "- primal strategy");
    env->timing->createTimer("PrimalBoundStrategyRootSearch", "  - performing root searches");

    env->timing->createTimer("CallbackExternalHyperplaneGeneration", "  - callback: external hyperplane generation");

    auto tFinalizeSolution = std::make_shared<TaskSequential>(env);

    auto tInitMIPSolver = std::make_shared<TaskInitializeDualSolver>(env, false);
    env->tasks->addTask(tInitMIPSolver, "InitMIPSolver");

    if(env->settings->getSetting<int>("Dual.CutStrategy") == (int)ES_HyperplaneCutStrategy::ESH
        && env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        auto tFindIntPoint = std::make_shared<TaskFindInteriorPoint>(env);
        env->tasks->addTask(tFindIntPoint, "FindIntPoint");
    }

    auto tCreateDualProblem = std::make_shared<TaskCreateDualProblem>(env);
    env->tasks->addTask(tCreateDualProblem, "CreateDualProblem");

    auto tInitializeRootsearch = std::make_shared<TaskInitializeRootsearch>(env);
    env->tasks->addTask(tInitializeRootsearch, "InitializeRootsearch");

    auto tInitializeIteration = std::make_shared<TaskInitializeIteration>(env);
    env->tasks->addTask(tInitializeIteration, "InitIter");

    auto tAddHPs = std::make_shared<TaskAddHyperplanes>(env);
    env->tasks->addTask(tAddHPs, "AddHPs");

    auto tUpdateExternalDualBound = std::make_shared<TaskUpdateExternalDualBound>(env);
    env->tasks->addTask(tUpdateExternalDualBound, "UpdateExternalDualBound");

    // Review external candidates before the first dual solve, including MIP starts for linear problems.
    auto tSelectPrimExternalInitial = std::make_shared<TaskSelectPrimalCandidatesFromExternalSource>(env);
    env->tasks->addTask(tSelectPrimExternalInitial, "SelectPrimExternalInitial");

    // A termination requested by a callback before the first dual problem is solved, e.g., at the interior point
    // search, stops the solution process here
    auto tCheckInitialUserTerm = std::make_shared<TaskCheckUserTermination>(env, "FinalizeSolution", false);
    env->tasks->addTask(tCheckInitialUserTerm, "CheckUserTerminationInitial");

    // An NLP problem is solved from a given starting point, e.g. the levels of the variables of a GAMS model, before
    // the first dual problem, which can take the remaining time of a nonconvex problem. Without a starting point,
    // there are no candidates and nothing is solved.
    if(env->settings->getSetting<bool>("Primal.FixedInteger.Use")
        && env->settings->getSetting<bool>("Primal.FixedInteger.UseStartingPoint"))
    {
        auto NLPProblemSource = static_cast<ES_PrimalNLPProblemSource>(
            env->settings->getSetting<int>("Primal.FixedInteger.SourceProblem"));

        // With both problems, the points are solved on each, since they are only cleared after the last task
        if(NLPProblemSource == ES_PrimalNLPProblemSource::Both
            || NLPProblemSource == ES_PrimalNLPProblemSource::OriginalProblem)
        {
            auto tSolveStartingPointNLP = std::make_shared<TaskSelectPrimalCandidatesFromNLP>(env, false);
            env->tasks->addTask(tSolveStartingPointNLP, "SolveStartingPointNLPOriginal");
        }

        if(NLPProblemSource == ES_PrimalNLPProblemSource::Both
            || NLPProblemSource == ES_PrimalNLPProblemSource::ReformulatedProblem)
        {
            auto tSolveStartingPointNLP = std::make_shared<TaskSelectPrimalCandidatesFromNLP>(env, true);
            env->tasks->addTask(tSolveStartingPointNLP, "SolveStartingPointNLPReformulated");
        }

        auto tClearStartingPointNLP = std::make_shared<TaskClearFixedPrimalCandidates>(env);
        env->tasks->addTask(tClearStartingPointNLP, "ClearStartingPointNLP");
    }

    auto tSolveIteration = std::make_shared<TaskSolveIteration>(env);
    env->tasks->addTask(tSolveIteration, "SolveIter");

    auto tSelectPrimSolPool = std::make_shared<TaskSelectPrimalCandidatesFromSolutionPool>(env);
    env->tasks->addTask(tSelectPrimSolPool, "SelectPrimSolPool");
    std::dynamic_pointer_cast<TaskSequential>(tFinalizeSolution)->addTask(tSelectPrimSolPool);

    if(env->settings->getSetting<bool>("Primal.Rootsearch.Use")
        && env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        auto tSelectPrimRootsearch = std::make_shared<TaskSelectPrimalCandidatesFromRootsearch>(env);
        env->tasks->addTask(tSelectPrimRootsearch, "SelectPrimRootsearch");
        std::dynamic_pointer_cast<TaskSequential>(tFinalizeSolution)->addTask(tSelectPrimRootsearch);
    }

    auto tSelectPrimExternal = std::make_shared<TaskSelectPrimalCandidatesFromExternalSource>(env);
    env->tasks->addTask(tSelectPrimExternal, "SelectPrimExternal");
    std::dynamic_pointer_cast<TaskSequential>(tFinalizeSolution)->addTask(tSelectPrimExternal);

    // A nonconvex problem otherwise only gets primal solutions from the points of the dual problems, which can improve
    // very slowly, e.g., with the cuts of a concave objective function, and from the final polish, so the NLP problem
    // is also solved from the dual solutions during the search, the first time there is one and then with the
    // iteration and time frequencies
    if(env->settings->getSetting<bool>("Primal.FixedInteger.Use")
        && env->reformulatedProblem->properties.convexity != E_ProblemConvexity::Convex)
    {
        auto tSelectPrimFixedNLPSolPool = std::make_shared<TaskSelectPrimalFixedNLPPointsFromSolutionPool>(env);
        env->tasks->addTask(tSelectPrimFixedNLPSolPool, "SelectPrimFixedNLPSolPool");

        auto NLPProblemSource = static_cast<ES_PrimalNLPProblemSource>(
            env->settings->getSetting<int>("Primal.FixedInteger.SourceProblem"));

        if(NLPProblemSource == ES_PrimalNLPProblemSource::Both
            || NLPProblemSource == ES_PrimalNLPProblemSource::OriginalProblem)
        {
            auto tSelectPrimNLPCheck = std::make_shared<TaskSelectPrimalCandidatesFromNLP>(env, false);
            env->tasks->addTask(tSelectPrimNLPCheck, "SelectPrimNLPCheckOriginal");
        }

        if(NLPProblemSource == ES_PrimalNLPProblemSource::Both
            || NLPProblemSource == ES_PrimalNLPProblemSource::ReformulatedProblem)
        {
            auto tSelectPrimNLPCheck = std::make_shared<TaskSelectPrimalCandidatesFromNLP>(env, true);
            env->tasks->addTask(tSelectPrimNLPCheck, "SelectPrimNLPCheckReformulated");
        }

        auto tClearPrimNLPCands = std::make_shared<TaskClearFixedPrimalCandidates>(env);
        env->tasks->addTask(tClearPrimNLPCands, "SelectClearNLPCandidates");
    }

    // If the objective gap could not be closed, e.g., since the cuts of nonconvex constraints made the dual problem
    // infeasible, the NLP problem is solved without the starting point from the dual strategy
    if(env->settings->getSetting<bool>("Primal.FixedInteger.Use")
        && env->settings->getSetting<bool>("Primal.FixedInteger.Enumeration.UseAsFallback"))
    {
        auto tEnumerateFixedIntegerFallback = std::make_shared<TaskEnumerateFixedIntegerCombinations>(env, true);
        std::dynamic_pointer_cast<TaskSequential>(tFinalizeSolution)->addTask(tEnumerateFixedIntegerFallback);
    }

    // Once the search has finished, solve an NLP problem starting from the solution found to try to improve it.
    // The dual solver's own tolerances bound how accurate its point is, and near an optimum the objective is
    // often flat, so a converged objective can still sit on a point that is some way off. This is a separate
    // task instance from any used during the search: it must not be paced by the iteration and time heuristics
    // that apply there.
    // Not for convex problems: an LP problem has already been solved by the MIP solver, and for a convex NLP problem
    // the search closes the gap on its own.
    if(env->settings->getSetting<int>("Primal.PolishSolution.NumberOfPoints") > 0
        && env->reformulatedProblem->properties.convexity != E_ProblemConvexity::Convex)
    {
        auto tPolishPoint = std::make_shared<TaskSelectPrimalFixedNLPPointsFromSolutionPool>(env, true);
        std::dynamic_pointer_cast<TaskSequential>(tFinalizeSolution)->addTask(tPolishPoint);

        auto NLPProblemSource = static_cast<ES_PrimalNLPProblemSource>(
            env->settings->getSetting<int>("Primal.FixedInteger.SourceProblem"));

        // With both problems, the points are solved on each, since they are only cleared after the last task
        if(NLPProblemSource == ES_PrimalNLPProblemSource::Both
            || NLPProblemSource == ES_PrimalNLPProblemSource::OriginalProblem)
        {
            auto tPolishNLP = std::make_shared<TaskSelectPrimalCandidatesFromNLP>(env, false, true);
            std::dynamic_pointer_cast<TaskSequential>(tFinalizeSolution)->addTask(tPolishNLP);
        }

        if(NLPProblemSource == ES_PrimalNLPProblemSource::Both
            || NLPProblemSource == ES_PrimalNLPProblemSource::ReformulatedProblem)
        {
            auto tPolishNLP = std::make_shared<TaskSelectPrimalCandidatesFromNLP>(env, true, true);
            std::dynamic_pointer_cast<TaskSequential>(tFinalizeSolution)->addTask(tPolishNLP);
        }

        auto tPolishClear = std::make_shared<TaskClearFixedPrimalCandidates>(env);
        std::dynamic_pointer_cast<TaskSequential>(tFinalizeSolution)->addTask(tPolishClear);
    }

    auto tPrintIterReport = std::make_shared<TaskPrintIterationReport>(env);
    env->tasks->addTask(tPrintIterReport, "PrintIterReport");

    if(env->reformulatedProblem->properties.convexity != E_ProblemConvexity::Convex)
    {
        auto tRepairInfeasibility
            = std::make_shared<TaskRepairInfeasibleDualProblem>(env, "SolveIter", "CheckIterError");
        env->tasks->addTask(tRepairInfeasibility, "RepairInfeasibility");
    }

    auto tCheckIterError = std::make_shared<TaskCheckIterationError>(env, "FinalizeSolution");
    env->tasks->addTask(tCheckIterError, "CheckIterError");

    auto tCheckAbsGap = std::make_shared<TaskCheckAbsoluteGap>(env, "FinalizeSolution");
    env->tasks->addTask(tCheckAbsGap, "CheckAbsGap");

    auto tCheckRelGap = std::make_shared<TaskCheckRelativeGap>(env, "FinalizeSolution");
    env->tasks->addTask(tCheckRelGap, "CheckRelGap");

    auto tCheckIterLim = std::make_shared<TaskCheckIterationLimit>(env, "FinalizeSolution");
    env->tasks->addTask(tCheckIterLim, "CheckIterLim");

    auto tCheckTimeLim = std::make_shared<TaskCheckTimeLimit>(env, "FinalizeSolution");
    env->tasks->addTask(tCheckTimeLim, "CheckTimeLim");

    auto tCheckUserTerm = std::make_shared<TaskCheckUserTermination>(env, "FinalizeSolution");
    env->tasks->addTask(tCheckUserTerm, "CheckUserTermination");

    auto tCheckConstrTol = std::make_shared<TaskCheckConstraintTolerance>(env, "FinalizeSolution");
    env->tasks->addTask(tCheckConstrTol, "CheckConstrTol");

    if(env->reformulatedProblem->properties.convexity != E_ProblemConvexity::Convex
        && env->settings->getSetting<bool>("Dual.ReductionCut.Use"))
    {
        auto tCheckMaxNumberOfObjectiveCuts
            = std::make_shared<TaskCheckMaxNumberOfPrimalReductionCuts>(env, "FinalizeSolution");
        env->tasks->addTask(tCheckMaxNumberOfObjectiveCuts, "CheckMaxObjectiveCuts");

        auto tCheckPrimalStag = std::make_shared<TaskCheckPrimalStagnation>(env, "AddObjectiveCut", "CheckDualStag");
        env->tasks->addTask(tCheckPrimalStag, "CheckPrimalStag");

        auto tAddObjectiveCut = std::make_shared<TaskAddPrimalReductionCut>(env, "CheckDualStag", "CheckDualStag");
        env->tasks->addTask(tAddObjectiveCut, "AddObjectiveCut");
    }

    auto tCheckDualStag = std::make_shared<TaskCheckDualStagnation>(env, "FinalizeSolution");
    env->tasks->addTask(tCheckDualStag, "CheckDualStag");

    env->tasks->addTask(tInitializeIteration, "InitIter2");

    if(env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("Dual.CutStrategy"))
            == ES_HyperplaneCutStrategy::ESH)
        {
            auto tUpdateInteriorPoint = std::make_shared<TaskUpdateInteriorPoint>(env);
            env->tasks->addTask(tUpdateInteriorPoint, "UpdateInteriorPoint");

            auto tSelectHPPts = std::make_shared<TaskSelectHyperplanesESH>(env);
            env->tasks->addTask(tSelectHPPts, "SelectHPPts");
        }
        else if(static_cast<ES_HyperplaneCutStrategy>(env->settings->getSetting<int>("Dual.CutStrategy"))
            == ES_HyperplaneCutStrategy::ECP)
        {
            auto tSelectHPPts = std::make_shared<TaskSelectHyperplanesECP>(env);
            env->tasks->addTask(tSelectHPPts, "SelectHPPts");
        }
    }

    if(env->reformulatedProblem->objectiveFunction->properties.classification
        > E_ObjectiveFunctionClassification::Quadratic)
    {
        auto tSelectObjectiveHPPts = std::make_shared<TaskSelectHyperplanesObjectiveFunction>(env);
        env->tasks->addTask(tSelectObjectiveHPPts, "SelectObjectiveHPPts");
    }

    auto tSelectExternalHPs = std::make_shared<TaskSelectHyperplanesExternal>(env);
    env->tasks->addTask(tSelectExternalHPs, "SelectExternalHPs");

    env->tasks->addTask(tAddHPs, "AddHPs");

    // The distance between the points the hyperplanes are generated in is calculated once they have been
    // generated
    if(env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        auto tCalculateSolutionChangeNorm = std::make_shared<TaskCalculateSolutionChangeNorm>(env);
        env->tasks->addTask(tCalculateSolutionChangeNorm, "CalculateSolutionChangeNorm");
    }

    env->tasks->addTask(tUpdateExternalDualBound, "UpdateExternalDualBound");

    auto tGoto = std::make_shared<TaskGoto>(env, "SolveIter");
    env->tasks->addTask(tGoto, "Goto");

    env->tasks->addTask(tFinalizeSolution, "FinalizeSolution");

    if(env->reformulatedProblem->properties.convexity != E_ProblemConvexity::Convex
        && env->settings->getSetting<bool>("Dual.ReductionCut.Use"))
    {
        // The final reduction cut continues the search below the primal bound whatever stopped it, except a limit
        auto tAddObjectiveCutFinal
            = std::make_shared<TaskAddPrimalReductionCut>(env, "InitIter2", "Terminate", true, true);
        std::dynamic_pointer_cast<TaskSequential>(tFinalizeSolution)->addTask(tAddObjectiveCutFinal);
    }

    auto tTerminate = std::make_shared<TaskTerminate>(env);
    env->tasks->addTask(tTerminate, "Terminate");
}

SolutionStrategyNLP::~SolutionStrategyNLP() = default;

bool SolutionStrategyNLP::solveProblem()
{
    TaskPtr nextTask;

    try
    {
        while(env->tasks->getNextTask(nextTask))
        {
#ifdef SIMPLE_OUTPUT_CHARS
            env->output->outputTrace("---- Started task:  " + nextTask->getType());
            nextTask->run();
            env->output->outputTrace("---- Finished task: " + nextTask->getType());
#else
            env->output->outputTrace("┌─── Started task:  " + nextTask->getType());
            nextTask->run();
            env->output->outputTrace("└─── Finished task: " + nextTask->getType());
#endif
        }
    }
    catch(Exception& e)
    {
        env->output->outputCritical(fmt::format(" Cannot solve problem:  {}", e.what()));
        return (false);
    }

    return (true);
}

void SolutionStrategyNLP::initializeStrategy() { }
} // namespace SHOT