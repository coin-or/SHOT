/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "SolutionStrategyMIQCQP.h"

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
#include "../Tasks/TaskPresolve.h"

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
#include "../Tasks/TaskSelectHyperplanesExternal.h"
#include "../Tasks/TaskAddHyperplanes.h"
#include "../Tasks/TaskAddPrimalReductionCut.h"
#include "../Tasks/TaskCheckMaxNumberOfPrimalReductionCuts.h"

#include "../Tasks/TaskSelectPrimalCandidatesFromSolutionPool.h"
#include "../Tasks/TaskSelectPrimalCandidatesFromRootsearch.h"
#include "../Tasks/TaskSelectPrimalCandidatesFromNLP.h"
#include "../Tasks/TaskSelectPrimalFixedNLPPointsFromSolutionPool.h"
#include "../Tasks/TaskClearFixedPrimalCandidates.h"

#include "../Tasks/TaskSelectHyperplanesObjectiveFunction.h"

#include "../Output.h"
#include "../Settings.h"
#include "../Timing.h"

namespace SHOT
{

SolutionStrategyMIQCQP::SolutionStrategyMIQCQP(EnvironmentPtr envPtr)
{
    env = envPtr;

    env->timing->createTimer("InteriorPointSearch", "- interior point search");

    env->timing->createTimer("DualStrategy", "- dual strategy");
    env->timing->createTimer("DualProblemsDiscrete", "  - solving MIP problems");

    env->timing->createTimer("PrimalStrategy", "- primal strategy");
    env->timing->createTimer("PrimalBoundStrategyNLP", "  - solving NLP problems");

    env->timing->createTimer("CallbackExternalHyperplaneGeneration", "  - callback: external hyperplane generation");

    auto tFinalizeSolution = std::make_shared<TaskSequential>(env);

    auto tInitMIPSolver = std::make_shared<TaskInitializeDualSolver>(env, false);
    env->tasks->addTask(tInitMIPSolver, "InitMIPSolver");

    auto tCreateDualProblem = std::make_shared<TaskCreateDualProblem>(env);
    env->tasks->addTask(tCreateDualProblem, "CreateDualProblem");

    auto tInitializeIteration = std::make_shared<TaskInitializeIteration>(env);
    env->tasks->addTask(tInitializeIteration, "InitIter");

    auto tSolveIteration = std::make_shared<TaskSolveIteration>(env);
    env->tasks->addTask(tSolveIteration, "SolveIter");

    auto tSelectPrimSolPool = std::make_shared<TaskSelectPrimalCandidatesFromSolutionPool>(env);
    env->tasks->addTask(tSelectPrimSolPool, "SelectPrimSolPool");
    std::dynamic_pointer_cast<TaskSequential>(tFinalizeSolution)->addTask(tSelectPrimSolPool);

    auto tPrintIterReport = std::make_shared<TaskPrintIterationReport>(env);
    env->tasks->addTask(tPrintIterReport, "PrintIterReport");

    auto tCheckAbsGap = std::make_shared<TaskCheckAbsoluteGap>(env, "FinalizeSolution");
    env->tasks->addTask(tCheckAbsGap, "CheckAbsGap");

    auto tCheckRelGap = std::make_shared<TaskCheckRelativeGap>(env, "FinalizeSolution");
    env->tasks->addTask(tCheckRelGap, "CheckRelGap");

    if(env->settings->getSetting<bool>("FixedInteger.Use", "Primal") && env->reformulatedProblem->properties.isDiscrete)
    {
        auto tSelectPrimFixedNLPSolPool = std::make_shared<TaskSelectPrimalFixedNLPPointsFromSolutionPool>(env);
        env->tasks->addTask(tSelectPrimFixedNLPSolPool, "SelectPrimFixedNLPSolPool");
        std::dynamic_pointer_cast<TaskSequential>(tFinalizeSolution)->addTask(tSelectPrimFixedNLPSolPool);

        auto NLPProblemSource
            = static_cast<ES_PrimalNLPProblemSource>(env->settings->getSetting<int>("FixedInteger.Source", "Primal"));

        if(NLPProblemSource == ES_PrimalNLPProblemSource::Both
            || NLPProblemSource == ES_PrimalNLPProblemSource::OriginalProblem)
        {
            auto tSelectPrimNLPCheck = std::make_shared<TaskSelectPrimalCandidatesFromNLP>(env, false);
            env->tasks->addTask(tSelectPrimNLPCheck, "SelectPrimNLPCheckOriginal");
            std::dynamic_pointer_cast<TaskSequential>(tFinalizeSolution)->addTask(tSelectPrimNLPCheck);
        }

        if(NLPProblemSource == ES_PrimalNLPProblemSource::Both
            || NLPProblemSource == ES_PrimalNLPProblemSource::ReformulatedProblem)
        {
            auto tSelectPrimNLPCheck = std::make_shared<TaskSelectPrimalCandidatesFromNLP>(env, true);
            env->tasks->addTask(tSelectPrimNLPCheck, "SelectPrimNLPCheckReformulated");
            std::dynamic_pointer_cast<TaskSequential>(tFinalizeSolution)->addTask(tSelectPrimNLPCheck);
        }

        auto tClearPrimNLPCands = std::make_shared<TaskClearFixedPrimalCandidates>(env);
        env->tasks->addTask(tClearPrimNLPCands, "SelectClearNLPCandidates");
        std::dynamic_pointer_cast<TaskSequential>(tFinalizeSolution)->addTask(tClearPrimNLPCands);

        auto tCheckAbsGap2 = std::make_shared<TaskCheckAbsoluteGap>(env, "FinalizeSolution");
        env->tasks->addTask(tCheckAbsGap, "CheckAbsGap2");

        auto tCheckRelGap2 = std::make_shared<TaskCheckRelativeGap>(env, "FinalizeSolution");
        env->tasks->addTask(tCheckRelGap, "CheckRelGap2");
    }

    auto tCheckTimeLim = std::make_shared<TaskCheckTimeLimit>(env, "FinalizeSolution");
    env->tasks->addTask(tCheckTimeLim, "CheckTimeLim");

    auto tCheckUserTerm = std::make_shared<TaskCheckUserTermination>(env, "FinalizeSolution");
    env->tasks->addTask(tCheckUserTerm, "CheckUserTermination");

    auto tCheckIterLim = std::make_shared<TaskCheckIterationLimit>(env, "FinalizeSolution");
    env->tasks->addTask(tCheckIterLim, "CheckIterLim");

    auto tCheckIterError = std::make_shared<TaskCheckIterationError>(env, "FinalizeSolution");
    env->tasks->addTask(tCheckIterError, "CheckIterError");

    auto tCheckDualStag = std::make_shared<TaskCheckDualStagnation>(env, "FinalizeSolution");
    env->tasks->addTask(tCheckDualStag, "CheckDualStag");

    auto tCheckConstrTol = std::make_shared<TaskCheckConstraintTolerance>(env, "FinalizeSolution");
    env->tasks->addTask(tCheckConstrTol, "CheckConstrTol");

    env->tasks->addTask(tFinalizeSolution, "FinalizeSolution");
}

SolutionStrategyMIQCQP::~SolutionStrategyMIQCQP() = default;

bool SolutionStrategyMIQCQP::solveProblem()
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

void SolutionStrategyMIQCQP::initializeStrategy() { }
} // namespace SHOT
