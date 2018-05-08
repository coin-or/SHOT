/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "SolutionStrategySingleTree.h"

SolutionStrategySingleTree::SolutionStrategySingleTree(EnvironmentPtr envPtr, OSInstance *osInstance)
{
    env = envPtr;

    env->process->createTimer("ProblemInitialization", " - problem initialization");
    env->process->createTimer("InteriorPointSearch", " - interior point search");

    env->process->createTimer("DualProblemsRelaxed", "   - solving relaxed problems");
    env->process->createTimer("DualStrategy", " - dual strategy");
    env->process->createTimer("DualProblemsDiscrete", "   - solving MIP problems");
    env->process->createTimer("DualCutGenerationRootSearch", "   - performing root search for cuts");
    env->process->createTimer("DualObjectiveLiftRootSearch", "   - performing root search for objective lift");

    env->process->createTimer("PrimalStrategy", " - primal strategy");
    env->process->createTimer("PrimalBoundStrategyNLP", "   - solving NLP problems");
    env->process->createTimer("PrimalBoundStrategyRootSearch", "   - performing root searches");

    TaskBase *tFinalizeSolution = new TaskSequential(env);

    TaskBase *tInitMIPSolver = new TaskInitializeDualSolver(env, true);
    env->process->tasks->addTask(tInitMIPSolver, "InitMIPSolver");

    TaskBase *tInitOrigProblem = new TaskInitializeOriginalProblem(env, osInstance);
    env->process->tasks->addTask(tInitOrigProblem, "InitOrigProb");

    if (env->settings->getIntSetting("CutStrategy", "Dual") == (int)ES_HyperplaneCutStrategy::ESH && (env->process->originalProblem->getObjectiveFunctionType() != E_ObjectiveFunctionType::Quadratic || env->process->originalProblem->getNumberOfNonlinearConstraints() != 0))
    {
        TaskBase *tFindIntPoint = new TaskFindInteriorPoint(env);
        env->process->tasks->addTask(tFindIntPoint, "FindIntPoint");
    }

    TaskBase *tCreateDualProblem = new TaskCreateDualProblem(env);
    env->process->tasks->addTask(tCreateDualProblem, "CreateDualProblem");

    TaskBase *tInitializeLinesearch = new TaskInitializeLinesearch(env);
    env->process->tasks->addTask(tInitializeLinesearch, "InitializeLinesearch");

    TaskBase *tInitializeIteration = new TaskInitializeIteration(env);
    env->process->tasks->addTask(tInitializeIteration, "InitIter");

    TaskBase *tAddHPs = new TaskAddHyperplanes(env);
    env->process->tasks->addTask(tAddHPs, "AddHPs");

    if (static_cast<ES_MIPPresolveStrategy>(env->settings->getIntSetting("MIP.Presolve.Frequency", "Dual")) != ES_MIPPresolveStrategy::Never)
    {
        TaskBase *tPresolve = new TaskPresolve(env);
        env->process->tasks->addTask(tPresolve, "Presolve");
    }

    TaskBase *tSolveIteration = new TaskSolveIteration(env);
    env->process->tasks->addTask(tSolveIteration, "SolveIter");

    TaskBase *tSelectPrimSolPool = new TaskSelectPrimalCandidatesFromSolutionPool(env);
    env->process->tasks->addTask(tSelectPrimSolPool, "SelectPrimSolPool");
    dynamic_cast<TaskSequential *>(tFinalizeSolution)->addTask(tSelectPrimSolPool);

    TaskBase *tPrintIterReport = new TaskPrintIterationReport(env);
    env->process->tasks->addTask(tPrintIterReport, "PrintIterReport");

    TaskBase *tCheckAbsGap = new TaskCheckAbsoluteGap(env, "FinalizeSolution");
    env->process->tasks->addTask(tCheckAbsGap, "CheckAbsGap");

    TaskBase *tCheckRelGap = new TaskCheckRelativeGap(env, "FinalizeSolution");
    env->process->tasks->addTask(tCheckRelGap, "CheckRelGap");

    TaskBase *tCheckTimeLim = new TaskCheckTimeLimit(env, "FinalizeSolution");
    env->process->tasks->addTask(tCheckTimeLim, "CheckTimeLim");

    TaskBase *tCheckIterError = new TaskCheckIterationError(env, "FinalizeSolution");
    env->process->tasks->addTask(tCheckIterError, "CheckIterError");

    TaskBase *tCheckConstrTol = new TaskCheckConstraintTolerance(env, "FinalizeSolution");
    env->process->tasks->addTask(tCheckConstrTol, "CheckConstrTol");

    TaskBase *tCheckObjectiveGapNotMet = new TaskCheckObjectiveGapNotMet(env, "FinalizeSolution");
    env->process->tasks->addTask(tCheckObjectiveGapNotMet, "CheckObjGapNotMet");

    if (env->settings->getIntSetting("FixedInteger.CallStrategy", "Primal") && env->process->originalProblem->getNumberOfNonlinearConstraints() > 0 && env->process->originalProblem->getNumberOfDiscreteVariables() > 0)
    {
        TaskBase *tSelectPrimFixedNLPSolPool = new TaskSelectPrimalFixedNLPPointsFromSolutionPool(env);
        env->process->tasks->addTask(tSelectPrimFixedNLPSolPool, "SelectPrimFixedNLPSolPool");
        dynamic_cast<TaskSequential *>(tFinalizeSolution)->addTask(tSelectPrimFixedNLPSolPool);

        TaskBase *tSelectPrimNLPCheck = new TaskSelectPrimalCandidatesFromNLP(env);
        env->process->tasks->addTask(tSelectPrimNLPCheck, "SelectPrimNLPCheck");
        dynamic_cast<TaskSequential *>(tFinalizeSolution)->addTask(tSelectPrimNLPCheck);

        env->process->tasks->addTask(tCheckAbsGap, "CheckAbsGap");
        env->process->tasks->addTask(tCheckRelGap, "CheckRelGap");
    }

    env->process->tasks->addTask(tFinalizeSolution, "FinalizeSolution");
}

SolutionStrategySingleTree::~SolutionStrategySingleTree()
{
}

bool SolutionStrategySingleTree::solveProblem()
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

void SolutionStrategySingleTree::initializeStrategy()
{
}
