/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "SolutionStrategyMultiTree.h"

SolutionStrategyMultiTree::SolutionStrategyMultiTree(EnvironmentPtr envPtr, OSInstance *osInstance)
{
    env = envPtr;

    env->process->createTimer("ProblemInitialization", " - problem initialization");
    env->process->createTimer("InteriorPointSearch", " - interior point search");

    env->process->createTimer("DualStrategy", " - dual strategy");
    env->process->createTimer("DualProblemsRelaxed", "   - solving relaxed problems");
    env->process->createTimer("DualProblemsIntegerFixed", "   - solving integer-fixed problems");
    env->process->createTimer("DualProblemsDiscrete", "   - solving MIP problems");
    env->process->createTimer("DualCutGenerationRootSearch", "   - performing root search for cuts");
    env->process->createTimer("DualObjectiveLiftRootSearch", "   - performing root search for objective lift");

    env->process->createTimer("PrimalStrategy", " - primal strategy");
    env->process->createTimer("PrimalBoundStrategyNLP", "   - solving NLP problems");
    env->process->createTimer("PrimalBoundStrategyRootSearch", "   - performing root searches");

    TaskBase *tFinalizeSolution = new TaskSequential(env);

    TaskBase *tInitMIPSolver = new TaskInitializeDualSolver(env, false);
    env->process->tasks->addTask(tInitMIPSolver, "InitMIPSolver");

    TaskBase *tInitOrigProblem = new TaskInitializeOriginalProblem(env, osInstance);
    env->process->tasks->addTask(tInitOrigProblem, "InitOrigProb");

    if (env->settings->getIntSetting("CutStrategy", "Dual") == (int)ES_HyperplaneCutStrategy::ESH && (env->process->originalProblem->getObjectiveFunctionType() != E_ObjectiveFunctionType::Quadratic || env->process->originalProblem->getNumberOfNonlinearConstraints() != 0))
    {
        TaskBase *tFindIntPoint = new TaskFindInteriorPoint(env);
        env->process->tasks->addTask(tFindIntPoint, "FindIntPoint");
    }

    TaskBase *tCreateDualProblem = new TaskCreateDualProblem(env);
    env->process->tasks->addTask(tCreateDualProblem, "CreateMILPProblem");

    TaskBase *tInitializeLinesearch = new TaskInitializeLinesearch(env);
    env->process->tasks->addTask(tInitializeLinesearch, "InitializeLinesearch");

    TaskBase *tInitializeIteration = new TaskInitializeIteration(env);
    env->process->tasks->addTask(tInitializeIteration, "InitIter");

    TaskBase *tAddHPs = new TaskAddHyperplanes(env);
    env->process->tasks->addTask(tAddHPs, "AddHPs");

    TaskBase *tExecuteRelaxStrategy = new TaskExecuteRelaxationStrategy(env);
    env->process->tasks->addTask(tExecuteRelaxStrategy, "ExecRelaxStrategyInitial");

    if (static_cast<ES_MIPPresolveStrategy>(env->settings->getIntSetting("MIP.Presolve.Frequency", "Dual")) != ES_MIPPresolveStrategy::Never)
    {
        TaskBase *tPresolve = new TaskPresolve(env);
        env->process->tasks->addTask(tPresolve, "Presolve");
    }

    TaskBase *tSolveIteration = new TaskSolveIteration(env);
    env->process->tasks->addTask(tSolveIteration, "SolveIter");

    if (env->process->originalProblem->isObjectiveFunctionNonlinear() && env->settings->getBoolSetting("ObjectiveLinesearch.Use", "Dual"))
    {
        TaskBase *tUpdateNonlinearObjectiveSolution = new TaskUpdateNonlinearObjectiveByLinesearch(env);
        env->process->tasks->addTask(tUpdateNonlinearObjectiveSolution, "UpdateNonlinearObjective");
    }

    TaskBase *tSelectPrimSolPool = new TaskSelectPrimalCandidatesFromSolutionPool(env);
    env->process->tasks->addTask(tSelectPrimSolPool, "SelectPrimSolPool");
    dynamic_cast<TaskSequential *>(tFinalizeSolution)->addTask(tSelectPrimSolPool);

    if (env->settings->getBoolSetting("Linesearch.Use", "Primal"))
    {
        TaskBase *tSelectPrimLinesearch = new TaskSelectPrimalCandidatesFromLinesearch(env);
        env->process->tasks->addTask(tSelectPrimLinesearch, "SelectPrimLinesearch");
        dynamic_cast<TaskSequential *>(tFinalizeSolution)->addTask(tSelectPrimLinesearch);
    }

    TaskBase *tPrintIterReport = new TaskPrintIterationReport(env);
    env->process->tasks->addTask(tPrintIterReport, "PrintIterReport");

    TaskBase *tCheckAbsGap = new TaskCheckAbsoluteGap(env, "FinalizeSolution");
    env->process->tasks->addTask(tCheckAbsGap, "CheckAbsGap");

    TaskBase *tCheckRelGap = new TaskCheckRelativeGap(env, "FinalizeSolution");
    env->process->tasks->addTask(tCheckRelGap, "CheckRelGap");

    TaskBase *tCheckIterLim = new TaskCheckIterationLimit(env, "FinalizeSolution");
    env->process->tasks->addTask(tCheckIterLim, "CheckIterLim");

    TaskBase *tCheckTimeLim = new TaskCheckTimeLimit(env, "FinalizeSolution");
    env->process->tasks->addTask(tCheckTimeLim, "CheckTimeLim");

    TaskBase *tCheckConstrTol = new TaskCheckConstraintTolerance(env, "FinalizeSolution");
    env->process->tasks->addTask(tCheckConstrTol, "CheckConstrTol");

    TaskBase *tCheckIterError = new TaskCheckIterationError(env, "FinalizeSolution");
    env->process->tasks->addTask(tCheckIterError, "CheckIterError");

    TaskBase *tCheckObjStag = new TaskCheckObjectiveStagnation(env, "FinalizeSolution");
    env->process->tasks->addTask(tCheckObjStag, "CheckObjStag");

    if (env->settings->getBoolSetting("FixedInteger.Use", "Dual"))
    {
        TaskBase *tSolveFixedLP = new TaskSolveFixedDualProblem(env);
        env->process->tasks->addTask(tSolveFixedLP, "SolveFixedLP");
        env->process->tasks->addTask(tCheckAbsGap, "CheckAbsGap");
        env->process->tasks->addTask(tCheckRelGap, "CheckRelGap");
    }

    if (env->settings->getBoolSetting("FixedInteger.Use", "Primal") && env->process->originalProblem->getNumberOfNonlinearConstraints() > 0 && env->process->originalProblem->getNumberOfDiscreteVariables() > 0)
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

    env->process->tasks->addTask(tInitializeIteration, "InitIter");

    env->process->tasks->addTask(tExecuteRelaxStrategy, "ExecRelaxStrategy");

    TaskBase *tExecuteSolLimStrategy = new TaskExecuteSolutionLimitStrategy(env);
    env->process->tasks->addTask(tExecuteSolLimStrategy, "ExecSolLimStrategy");

    if (static_cast<ES_HyperplaneCutStrategy>(env->settings->getIntSetting("CutStrategy", "Dual")) == ES_HyperplaneCutStrategy::ESH)
    {
        TaskBase *tUpdateInteriorPoint = new TaskUpdateInteriorPoint(env);
        env->process->tasks->addTask(tUpdateInteriorPoint, "UpdateInteriorPoint");

        if (static_cast<ES_RootsearchConstraintStrategy>(env->settings->getIntSetting(
                "ESH.Linesearch.ConstraintStrategy", "Dual")) == ES_RootsearchConstraintStrategy::AllAsMaxFunct)
        {
            TaskBase *tSelectHPPts = new TaskSelectHyperplanePointsLinesearch(env);
            env->process->tasks->addTask(tSelectHPPts, "SelectHPPts");
        }
        else
        {
            TaskBase *tSelectHPPts = new TaskSelectHyperplanePointsIndividualLinesearch(env);
            env->process->tasks->addTask(tSelectHPPts, "SelectHPPts");
        }
    }
    else
    {
        TaskBase *tSelectHPPts = new TaskSelectHyperplanePointsSolution(env);
        env->process->tasks->addTask(tSelectHPPts, "SelectHPPts");
    }

    env->process->tasks->addTask(tAddHPs, "AddHPs");

    if (env->settings->getBoolSetting("HyperplaneCuts.UseIntegerCuts", "Dual"))
    {
        TaskBase *tAddICs = new TaskAddIntegerCuts(env);
        env->process->tasks->addTask(tAddICs, "AddICs");
    }

    TaskBase *tGoto = new TaskGoto(env, "SolveIter");
    env->process->tasks->addTask(tGoto, "Goto");

    env->process->tasks->addTask(tFinalizeSolution, "FinalizeSolution");
}

SolutionStrategyMultiTree::~SolutionStrategyMultiTree()
{
}

bool SolutionStrategyMultiTree::solveProblem()
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

void SolutionStrategyMultiTree::initializeStrategy()
{
}
