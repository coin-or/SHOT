/**
	The Supporting Hyperplane Optimization Toolkit (SHOT).

	@author Andreas Lundell, Åbo Akademi University

	@section LICENSE 
	This software is licensed under the Eclipse Public License 2.0. 
	Please see the README and LICENSE files for more information.
*/

#include "SolutionStrategyNLP.h"

namespace SHOT
{

SolutionStrategyNLP::SolutionStrategyNLP(EnvironmentPtr envPtr)
{
    env = envPtr;

    env->process->createTimer("ProblemInitialization", " - problem initialization");
    env->process->createTimer("ProblemReformulation", " - problem reformulation");
    env->process->createTimer("InteriorPointSearch", " - interior point search");

    env->process->createTimer("DualStrategy", " - dual strategy");
    env->process->createTimer("DualProblemsRelaxed", "   - solving relaxed problems");
    env->process->createTimer("DualProblemsDiscrete", "   - solving MIP problems");
    env->process->createTimer("DualCutGenerationRootSearch", "   - root search for constraint cuts");
    env->process->createTimer("DualObjectiveRootSearch", "   - root search for objective cut");

    env->process->createTimer("PrimalStrategy", " - primal strategy");
    env->process->createTimer("PrimalBoundStrategyRootSearch", "   - performing root searches");

    TaskBase *tFinalizeSolution = new TaskSequential(env);

    TaskBase *tInitMIPSolver = new TaskInitializeDualSolver(env, false);
    env->tasks->addTask(tInitMIPSolver, "InitMIPSolver");

    //TaskBase *tInitOrigProblem = new TaskInitializeOriginalProblem(env, osInstance);
    //env->tasks->addTask(tInitOrigProblem, "InitOrigProb");

    TaskBase *tReformulateProblem = new TaskReformulateProblem(env);
    env->tasks->addTask(tReformulateProblem, "ReformlateProb");

    if (env->settings->getIntSetting("CutStrategy", "Dual") == (int)ES_HyperplaneCutStrategy::ESH && env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        TaskBase *tFindIntPoint = new TaskFindInteriorPoint(env);
        env->tasks->addTask(tFindIntPoint, "FindIntPoint");
    }

    TaskBase *tCreateDualProblem = new TaskCreateDualProblem(env);
    env->tasks->addTask(tCreateDualProblem, "CreateDualProblem");

    TaskBase *tInitializeLinesearch = new TaskInitializeLinesearch(env);
    env->tasks->addTask(tInitializeLinesearch, "InitializeLinesearch");

    TaskBase *tInitializeIteration = new TaskInitializeIteration(env);
    env->tasks->addTask(tInitializeIteration, "InitIter");

    TaskBase *tAddHPs = new TaskAddHyperplanes(env);
    env->tasks->addTask(tAddHPs, "AddHPs");

    TaskBase *tExecuteRelaxStrategy = new TaskExecuteRelaxationStrategy(env);
    env->tasks->addTask(tExecuteRelaxStrategy, "ExecRelaxStrategyInitial");

    if (static_cast<ES_MIPPresolveStrategy>(env->settings->getIntSetting("MIP.Presolve.Frequency", "Dual")) != ES_MIPPresolveStrategy::Never)
    {
        TaskBase *tPresolve = new TaskPresolve(env);
        env->tasks->addTask(tPresolve, "Presolve");
    }

    TaskBase *tSolveIteration = new TaskSolveIteration(env);
    env->tasks->addTask(tSolveIteration, "SolveIter");

    TaskBase *tSelectPrimSolPool = new TaskSelectPrimalCandidatesFromSolutionPool(env);
    env->tasks->addTask(tSelectPrimSolPool, "SelectPrimSolPool");
    dynamic_cast<TaskSequential *>(tFinalizeSolution)->addTask(tSelectPrimSolPool);

    if (env->settings->getBoolSetting("Linesearch.Use", "Primal") && env->reformulatedProblem->properties.numberOfNonlinearConstraints > 0)
    {
        TaskBase *tSelectPrimLinesearch = new TaskSelectPrimalCandidatesFromLinesearch(env);
        env->tasks->addTask(tSelectPrimLinesearch, "SelectPrimLinesearch");
        dynamic_cast<TaskSequential *>(tFinalizeSolution)->addTask(tSelectPrimLinesearch);
    }

    TaskBase *tPrintIterReport = new TaskPrintIterationReport(env);
    env->tasks->addTask(tPrintIterReport, "PrintIterReport");

    TaskBase *tCheckAbsGap = new TaskCheckAbsoluteGap(env, "FinalizeSolution");
    env->tasks->addTask(tCheckAbsGap, "CheckAbsGap");

    TaskBase *tCheckRelGap = new TaskCheckRelativeGap(env, "FinalizeSolution");
    env->tasks->addTask(tCheckRelGap, "CheckRelGap");

    TaskBase *tCheckIterLim = new TaskCheckIterationLimit(env, "FinalizeSolution");
    env->tasks->addTask(tCheckIterLim, "CheckIterLim");

    TaskBase *tCheckTimeLim = new TaskCheckTimeLimit(env, "FinalizeSolution");
    env->tasks->addTask(tCheckTimeLim, "CheckTimeLim");

    TaskBase *tCheckIterError = new TaskCheckIterationError(env, "FinalizeSolution");
    env->tasks->addTask(tCheckIterError, "CheckIterError");

    TaskBase *tCheckConstrTol = new TaskCheckConstraintTolerance(env, "FinalizeSolution");
    env->tasks->addTask(tCheckConstrTol, "CheckConstrTol");

    TaskBase *tCheckObjStag = new TaskCheckObjectiveStagnation(env, "FinalizeSolution");
    env->tasks->addTask(tCheckObjStag, "CheckObjStag");

    env->tasks->addTask(tInitializeIteration, "InitIter");

    if (static_cast<ES_HyperplaneCutStrategy>(env->settings->getIntSetting("CutStrategy", "Dual")) == ES_HyperplaneCutStrategy::ESH)
    {
        TaskBase *tUpdateInteriorPoint = new TaskUpdateInteriorPoint(env);
        env->tasks->addTask(tUpdateInteriorPoint, "UpdateInteriorPoint");

        TaskBase *tSelectHPPts = new TaskSelectHyperplanePointsESH(env);
        env->tasks->addTask(tSelectHPPts, "SelectHPPts");
    }
    else
    {
        TaskBase *tSelectHPPts = new TaskSelectHyperplanePointsECP(env);
        env->tasks->addTask(tSelectHPPts, "SelectHPPts");
    }

    if (env->reformulatedProblem->objectiveFunction->properties.classification > E_ObjectiveFunctionClassification::Quadratic)
    {
        TaskBase *tSelectObjectiveHPPts = new TaskSelectHyperplanePointsByObjectiveLinesearch(env);
        env->tasks->addTask(tSelectObjectiveHPPts, "SelectObjectiveHPPts");
    }

    env->tasks->addTask(tAddHPs, "AddHPs");

    TaskBase *tGoto = new TaskGoto(env, "SolveIter");
    env->tasks->addTask(tGoto, "Goto");

    env->tasks->addTask(tFinalizeSolution, "FinalizeSolution");
}

SolutionStrategyNLP::~SolutionStrategyNLP()
{
}

bool SolutionStrategyNLP::solveProblem()
{
    TaskBase *nextTask;

    while (env->tasks->getNextTask(nextTask))
    {
        env->output->outputInfo("┌─── Started task:  " + nextTask->getType());
        nextTask->run();
        env->output->outputInfo("└─── Finished task: " + nextTask->getType());
    }

    return (true);
}

void SolutionStrategyNLP::initializeStrategy()
{
}
} // namespace SHOT