/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "SolutionStrategyMIQCQP.h"

SolutionStrategyMIQCQP::SolutionStrategyMIQCQP(EnvironmentPtr envPtr, OSInstance *osInstance)
{
    env = envPtr;

    env->process->createTimer("ProblemInitialization", " - problem initialization");
    env->process->createTimer("InteriorPointSearch", " - interior point search");

    env->process->createTimer("DualStrategy", " - dual strategy");
    env->process->createTimer("DualProblemsDiscrete", "   - solving MIP problems");

    env->process->createTimer("PrimalStrategy", " - primal strategy");

    TaskBase *tFinalizeSolution = new TaskSequential(env);

    TaskBase *tInitMIPSolver = new TaskInitializeDualSolver(env, false);
    env->process->tasks->addTask(tInitMIPSolver, "InitMIPSolver");

    TaskBase *tInitOrigProblem = new TaskInitializeOriginalProblem(env, osInstance);
    env->process->tasks->addTask(tInitOrigProblem, "InitOrigProb");

    TaskBase *tCreateDualProblem = new TaskCreateDualProblem(env);
    env->process->tasks->addTask(tCreateDualProblem, "CreateDualProblem");

    TaskBase *tInitializeIteration = new TaskInitializeIteration(env);
    env->process->tasks->addTask(tInitializeIteration, "InitIter");

    TaskBase *tSolveIteration = new TaskSolveIteration(env);
    env->process->tasks->addTask(tSolveIteration, "SolveIter");

    TaskBase *tSelectPrimSolPool = new TaskSelectPrimalCandidatesFromSolutionPool(env);
    env->process->tasks->addTask(tSelectPrimSolPool, "SelectPrimSolPool");
    dynamic_cast<TaskSequential *>(tFinalizeSolution)->addTask(tSelectPrimSolPool);

    TaskBase *tPrintIterReport = new TaskPrintIterationReport(env);
    env->process->tasks->addTask(tPrintIterReport, "PrintIterReport");

    TaskBase *tCheckIterError = new TaskCheckIterationError(env, "FinalizeSolution");
    env->process->tasks->addTask(tCheckIterError, "CheckIterError");

    TaskBase *tCheckAbsGap = new TaskCheckAbsoluteGap(env, "FinalizeSolution");
    env->process->tasks->addTask(tCheckAbsGap, "CheckAbsGap");

    TaskBase *tCheckRelGap = new TaskCheckRelativeGap(env, "FinalizeSolution");
    env->process->tasks->addTask(tCheckRelGap, "CheckRelGap");

    TaskBase *tCheckTimeLim = new TaskCheckTimeLimit(env, "FinalizeSolution");
    env->process->tasks->addTask(tCheckTimeLim, "CheckTimeLim");

    TaskBase *tCheckIterLim = new TaskCheckIterationLimit(env, "FinalizeSolution");
    env->process->tasks->addTask(tCheckIterLim, "CheckIterLim");

    TaskBase *tCheckConstrTol = new TaskCheckConstraintTolerance(env, "FinalizeSolution");
    env->process->tasks->addTask(tCheckConstrTol, "CheckConstrTol");

    env->process->tasks->addTask(tFinalizeSolution, "FinalizeSolution");
}

SolutionStrategyMIQCQP::~SolutionStrategyMIQCQP()
{
}

bool SolutionStrategyMIQCQP::solveProblem()
{
    TaskBase *nextTask;

    while (env->process->tasks->getNextTask(nextTask))
    {
        env->output->outputInfo("┌─── Started task:  " + nextTask->getType());
        nextTask->run();
        env->output->outputInfo("└─── Finished task: " + nextTask->getType());
    }

    return (true);
}

void SolutionStrategyMIQCQP::initializeStrategy()
{
}
