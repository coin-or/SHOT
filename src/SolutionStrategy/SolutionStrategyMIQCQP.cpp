/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "SolutionStrategyMIQCQP.h"

#include "../TaskHandler.h"

#include "../Tasks/TaskAddIntegerCuts.h"
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

#include "../Tasks/TaskInitializeLinesearch.h"
#include "../Tasks/TaskSelectHyperplanePointsESH.h"
#include "../Tasks/TaskSelectHyperplanePointsECP.h"
#include "../Tasks/TaskAddHyperplanes.h"
#include "../Tasks/TaskAddPrimalReductionCut.h"
#include "../Tasks/TaskCheckMaxNumberOfPrimalReductionCuts.h"

#include "../Tasks/TaskSelectPrimalCandidatesFromSolutionPool.h"
#include "../Tasks/TaskSelectPrimalCandidatesFromLinesearch.h"
#include "../Tasks/TaskSelectPrimalCandidatesFromNLP.h"
#include "../Tasks/TaskSelectPrimalFixedNLPPointsFromSolutionPool.h"

#include "../Tasks/TaskUpdateInteriorPoint.h"

#include "../Tasks/TaskSelectHyperplanePointsByObjectiveLinesearch.h"
#include "../Tasks/TaskSolveFixedDualProblem.h"

#include "../Tasks/TaskAddIntegerCuts.h"

#include "../Output.h"
#include "../Timing.h"

namespace SHOT
{

SolutionStrategyMIQCQP::SolutionStrategyMIQCQP(EnvironmentPtr envPtr)
{
    env = envPtr;

    env->timing->createTimer("InteriorPointSearch", " - interior point search");

    env->timing->createTimer("DualStrategy", " - dual strategy");
    env->timing->createTimer("DualProblemsDiscrete", "   - solving MIP problems");

    env->timing->createTimer("PrimalStrategy", " - primal strategy");

    TaskBase* tFinalizeSolution = new TaskSequential(env);

    TaskBase* tInitMIPSolver = new TaskInitializeDualSolver(env, false);
    env->tasks->addTask(tInitMIPSolver, "InitMIPSolver");

    TaskBase* tCreateDualProblem = new TaskCreateDualProblem(env);
    env->tasks->addTask(tCreateDualProblem, "CreateDualProblem");

    TaskBase* tInitializeIteration = new TaskInitializeIteration(env);
    env->tasks->addTask(tInitializeIteration, "InitIter");

    TaskBase* tSolveIteration = new TaskSolveIteration(env);
    env->tasks->addTask(tSolveIteration, "SolveIter");

    TaskBase* tSelectPrimSolPool = new TaskSelectPrimalCandidatesFromSolutionPool(env);
    env->tasks->addTask(tSelectPrimSolPool, "SelectPrimSolPool");
    dynamic_cast<TaskSequential*>(tFinalizeSolution)->addTask(tSelectPrimSolPool);

    TaskBase* tPrintIterReport = new TaskPrintIterationReport(env);
    env->tasks->addTask(tPrintIterReport, "PrintIterReport");

    TaskBase* tCheckAbsGap = new TaskCheckAbsoluteGap(env, "FinalizeSolution");
    env->tasks->addTask(tCheckAbsGap, "CheckAbsGap");

    TaskBase* tCheckRelGap = new TaskCheckRelativeGap(env, "FinalizeSolution");
    env->tasks->addTask(tCheckRelGap, "CheckRelGap");

    TaskBase* tCheckTimeLim = new TaskCheckTimeLimit(env, "FinalizeSolution");
    env->tasks->addTask(tCheckTimeLim, "CheckTimeLim");

    TaskBase* tCheckUserTerm = new TaskCheckUserTermination(env, "FinalizeSolution");
    env->tasks->addTask(tCheckUserTerm, "CheckUserTermination");

    TaskBase* tCheckIterLim = new TaskCheckIterationLimit(env, "FinalizeSolution");
    env->tasks->addTask(tCheckIterLim, "CheckIterLim");

    TaskBase* tCheckIterError = new TaskCheckIterationError(env, "FinalizeSolution");
    env->tasks->addTask(tCheckIterError, "CheckIterError");

    TaskBase* tCheckDualStag = new TaskCheckDualStagnation(env, "FinalizeSolution");
    env->tasks->addTask(tCheckDualStag, "CheckDualStag");

    TaskBase* tCheckConstrTol = new TaskCheckConstraintTolerance(env, "FinalizeSolution");
    env->tasks->addTask(tCheckConstrTol, "CheckConstrTol");

    env->tasks->addTask(tFinalizeSolution, "FinalizeSolution");
}

SolutionStrategyMIQCQP::~SolutionStrategyMIQCQP() {}

bool SolutionStrategyMIQCQP::solveProblem()
{
    TaskBase* nextTask;

    while(env->tasks->getNextTask(nextTask))
    {
        env->output->outputDebug("┌─── Started task:  " + nextTask->getType());
        nextTask->run();
        env->output->outputDebug("└─── Finished task: " + nextTask->getType());
    }

    return (true);
}

void SolutionStrategyMIQCQP::initializeStrategy() {}
} // namespace SHOT
